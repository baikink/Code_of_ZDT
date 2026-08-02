#include "bt_tune.h"
#include "usart.h"
#include "pid.h"
#include "Emm_V5.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define BT_TUNE_MOTOR_ADDR 1u

static uint8_t bt_rx_byte;
static char bt_raw[BT_BUF_SIZE];
static uint8_t bt_raw_len = 0;
static char bt_pkt[BT_BUF_SIZE];
static volatile bool bt_pkt_ready = false;
static char bt_decimal_buf[16];

static void BT_Tune_SendReply(const char *reply)
{
    HAL_UART_Transmit(&huart4, (uint8_t *)reply, (uint16_t)strlen(reply), 20);
}

static void BT_Tune_SendOk(void)
{
    BT_Tune_SendReply("ok\r\n");
}

static void BT_Tune_SendErr(void)
{
    BT_Tune_SendReply("err\r\n");
}

static int32_t BT_Tune_FloatToMilli(float value)
{
    if(value >= 0.0f) {
        return (int32_t)(value * 1000.0f + 0.5f);
    }
    return (int32_t)(value * 1000.0f - 0.5f);
}

static const char *BT_Tune_MilliToString(int32_t value_milli)
{
    int index = 0;
    int32_t integer_part;
    int32_t decimal_part;

    if(value_milli < 0) {
        bt_decimal_buf[index++] = '-';
        value_milli = -value_milli;
    }

    integer_part = value_milli / 1000;
    decimal_part = value_milli % 1000;

    if(integer_part == 0) {
        bt_decimal_buf[index++] = '0';
    } else {
        char digits[10];
        int digit_count = 0;

        while(integer_part > 0 && digit_count < (int)sizeof(digits)) {
            digits[digit_count++] = (char)('0' + (integer_part % 10));
            integer_part /= 10;
        }

        while(digit_count > 0) {
            bt_decimal_buf[index++] = digits[--digit_count];
        }
    }

    bt_decimal_buf[index++] = '.';
    bt_decimal_buf[index++] = (char)('0' + (decimal_part / 100));
    bt_decimal_buf[index++] = (char)('0' + ((decimal_part / 10) % 10));
    bt_decimal_buf[index++] = (char)('0' + (decimal_part % 10));
    bt_decimal_buf[index] = '\0';

    return bt_decimal_buf;
}

static void BT_Tune_SendNameValue(const char *name, float value)
{
    BT_Tune_SendReply(name);
    BT_Tune_SendReply("=");
    BT_Tune_SendReply(BT_Tune_MilliToString(BT_Tune_FloatToMilli(value)));
    BT_Tune_SendReply("\r\n");
}

static char *BT_Tune_TrimToken(char *token)
{
    char *end;

    if(token == NULL) {
        return NULL;
    }

    while((*token == ' ') || (*token == '\t')) {
        token++;
    }

    end = token + strlen(token);
    while((end > token) && ((end[-1] == ' ') || (end[-1] == '\t'))) {
        end--;
    }
    *end = '\0';

    return token;
}

static bool BT_Tune_SetPidByKey(const char *key, float value)
{
    if((strcmp(key, "1") == 0) || (strcmp(key, "Pos_P") == 0)) {
        pidY.p = value;
        return true;
    }
    if((strcmp(key, "2") == 0) || (strcmp(key, "Pos_I") == 0)) {
        pidY.i = value;
        return true;
    }
    if((strcmp(key, "3") == 0) || (strcmp(key, "Pos_D") == 0)) {
        pidY.d = value;
        return true;
    }
    if((strcmp(key, "4") == 0) || (strcmp(key, "Spd_P") == 0)) {
        pidY_Speed.p = value;
        return true;
    }
    if((strcmp(key, "5") == 0) || (strcmp(key, "Spd_I") == 0)) {
        pidY_Speed.i = value;
        return true;
    }
    if((strcmp(key, "6") == 0) || (strcmp(key, "Spd_D") == 0)) {
        pidY_Speed.d = value;
        return true;
    }

    return false;
}

static void BT_Tune_SendPid(void)
{
    BT_Tune_SendNameValue("Pos_P", pidY.p);
    BT_Tune_SendNameValue("Pos_I", pidY.i);
    BT_Tune_SendNameValue("Pos_D", pidY.d);
    BT_Tune_SendNameValue("Spd_P", pidY_Speed.p);
    BT_Tune_SendNameValue("Spd_I", pidY_Speed.i);
    BT_Tune_SendNameValue("Spd_D", pidY_Speed.d);
    BT_Tune_SendReply("OK\r\n");
}

void BT_Tune_RestartRx(void)
{
    HAL_UART_Receive_IT(&huart4, &bt_rx_byte, 1);
}

void BT_Tune_Init(void)
{
    BT_Tune_RestartRx();
}

void BT_Tune_ByteReceived(void)
{
    char c = (char)bt_rx_byte;

    if((c == '\r') || (c == '\n')) {
        BT_Tune_RestartRx();
        return;
    }

    if(c == '[') {
        bt_raw_len = 0;
    } else if((c == ']') || (c == '#')) {
        if(bt_raw_len > 0 && bt_raw_len < BT_BUF_SIZE) {
            bt_raw[bt_raw_len] = '\0';
            memcpy(bt_pkt, bt_raw, bt_raw_len + 1);
            bt_pkt_ready = true;
        }
        bt_raw_len = 0;
    } else {
        if(bt_raw_len < (BT_BUF_SIZE - 1u)) {
            bt_raw[bt_raw_len++] = c;
        }
    }

    BT_Tune_RestartRx();
}

void BT_Tune_Process(void)
{
    if(!bt_pkt_ready) {
        return;
    }
    bt_pkt_ready = false;

    char tmp[BT_BUF_SIZE];
    strncpy(tmp, bt_pkt, BT_BUF_SIZE - 1);
    tmp[BT_BUF_SIZE - 1] = '\0';

    char *cmd = BT_Tune_TrimToken(strtok(tmp, ","));
    char *arg1 = BT_Tune_TrimToken(strtok(NULL, ","));
    char *arg2 = BT_Tune_TrimToken(strtok(NULL, ","));

    if(cmd == NULL) {
        BT_Tune_SendErr();
        return;
    }

    if(strcmp(cmd, "GETPID") == 0) {
        BT_Tune_SendPid();
        return;
    }

    if(strcmp(cmd, "GET_0") == 0) {
        pidY.iout = 0.0f;
        pidY_Speed.iout = 0.0f;
        motor_set_angle(0.0f);
        Emm_V5_Origin_Trigger_Return(BT_TUNE_MOTOR_ADDR, 0, 0);
        BT_Tune_SendOk();
        return;
    }

    if(arg1 == NULL) {
        BT_Tune_SendErr();
        return;
    }

    if((strcmp(cmd, "s") == 0) || (strcmp(cmd, "slider") == 0)) {
        if(arg2 == NULL) {
            BT_Tune_SendErr();
            return;
        }

        float val = (float)atof(arg2);
        if(!BT_Tune_SetPidByKey(arg1, val)) {
            BT_Tune_SendErr();
            return;
        }

        BT_Tune_SendOk();
        return;
    }

    if((strcmp(cmd, "k") == 0) || (strcmp(cmd, "key") == 0)) {
        if(strcmp(arg1, "rst") != 0) {
            BT_Tune_SendErr();
            return;
        }

        pidY.iout = 0.0f;
        pidY_Speed.iout = 0.0f;
        BT_Tune_SendOk();
        return;
    }

    BT_Tune_SendErr();
}
