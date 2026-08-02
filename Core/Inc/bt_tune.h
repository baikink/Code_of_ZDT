#ifndef __BT_TUNE_H__
#define __BT_TUNE_H__

#include "main.h"

#define BT_BUF_SIZE 64

void BT_Tune_Init(void);
void BT_Tune_RestartRx(void);
void BT_Tune_ByteReceived(void);
void BT_Tune_Process(void);

#endif /* __BT_TUNE_H__ */
