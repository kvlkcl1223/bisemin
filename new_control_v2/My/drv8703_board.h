#ifndef __DRV8703_BOARD_H
#define __DRV8703_BOARD_H

#include "drv8703.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DRV8703_BOARD_CHANNEL_COUNT 5U

typedef enum
{
    DRV8703_BOARD_CH1 = 0,
    DRV8703_BOARD_CH2 = 1,
    DRV8703_BOARD_CH3 = 2,
    DRV8703_BOARD_CH4 = 3,
    DRV8703_BOARD_CH5 = 4
} DRV8703_BoardChannel_t;

extern DRV8703_Handle_t g_drv8703_board[DRV8703_BOARD_CHANNEL_COUNT];

DRV8703_Status_t DRV8703_BoardInitAll(void);
DRV8703_Status_t DRV8703_BoardInitOne(DRV8703_BoardChannel_t ch);
DRV8703_Handle_t *DRV8703_BoardGet(DRV8703_BoardChannel_t ch);
DRV8703_Status_t DRV8703_BoardApplyDefaultConfig(DRV8703_BoardChannel_t ch);

#ifdef __cplusplus
}
#endif

#endif
