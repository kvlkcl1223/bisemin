#include "drv8703_board.h"
#include "dac.h"
#include "opamp.h"
#include "spi.h"
#include "tim.h"

DRV8703_Handle_t g_drv8703_board[DRV8703_BOARD_CHANNEL_COUNT];

static void DRV8703_BoardDeselectAll(void)
{
    HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CS3_GPIO_Port, CS3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CS4_GPIO_Port, CS4_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CS5_GPIO_Port, CS5_Pin, GPIO_PIN_SET);
}

static const DRV8703_Config_t g_drv8703_board_config[DRV8703_BOARD_CHANNEL_COUNT] = {
    {
        &hspi1,
        &htim5,
        TIM_CHANNEL_3,
        &hdac3,
        DAC_CHANNEL_1,
        &hopamp1,
        CS1_GPIO_Port,
        CS1_Pin,
        SLEEP1_GPIO_Port,
        SLEEP1_Pin,
        DIR1_GPIO_Port,
        DIR1_Pin,
        FAULT1_GPIO_Port,
        FAULT1_Pin,
        WDFLT1_GPIO_Port,
        WDFLT1_Pin,
        10U,
        2U,
        3300U,
        0.50f,
        1U,
        1U,
        1U,
    },
    {
        &hspi1,
        &htim1,
        TIM_CHANNEL_2,
        &hdac1,
        DAC_CHANNEL_1,
        &hopamp2,
        CS2_GPIO_Port,
        CS2_Pin,
        SLEEP2_GPIO_Port,
        SLEEP2_Pin,
        DIR2_GPIO_Port,
        DIR2_Pin,
        FAULT2_GPIO_Port,
        FAULT2_Pin,
        WDFLT2_GPIO_Port,
        WDFLT2_Pin,
        10U,
        2U,
        3300U,
        0.50f,
        1U,
        1U,
        1U,
    },
    {
        &hspi1,
        &htim1,
        TIM_CHANNEL_3,
        &hdac3,
        DAC_CHANNEL_2,
        &hopamp3,
        CS3_GPIO_Port,
        CS3_Pin,
        SLEEP3_GPIO_Port,
        SLEEP3_Pin,
        DIR3_GPIO_Port,
        DIR3_Pin,
        FAULT3_GPIO_Port,
        FAULT3_Pin,
        WDFLT3_GPIO_Port,
        WDFLT3_Pin,
        10U,
        2U,
        3300U,
        0.50f,
        1U,
        1U,
        1U,
    },
    {
        &hspi1,
        &htim1,
        TIM_CHANNEL_4,
        &hdac4,
        DAC_CHANNEL_1,
        &hopamp4,
        CS4_GPIO_Port,
        CS4_Pin,
        SLEEP4_GPIO_Port,
        SLEEP4_Pin,
        DIR4_GPIO_Port,
        DIR4_Pin,
        FAULT4_GPIO_Port,
        FAULT4_Pin,
        WDFLT4_GPIO_Port,
        WDFLT4_Pin,
        10U,
        2U,
        3300U,
        0.50f,
        1U,
        1U,
        1U,
    },
    {
        &hspi1,
        &htim2,
        TIM_CHANNEL_1,
        &hdac4,
        DAC_CHANNEL_2,
        &hopamp5,
        CS5_GPIO_Port,
        CS5_Pin,
        SLEEP5_GPIO_Port,
        SLEEP5_Pin,
        DIR5_GPIO_Port,
        DIR5_Pin,
        FAULT5_GPIO_Port,
        FAULT5_Pin,
        WDFLT5_GPIO_Port,
        WDFLT5_Pin,
        10U,
        2U,
        3300U,
        0.50f,
        1U,
        1U,
        1U,
    },
};

DRV8703_Handle_t *DRV8703_BoardGet(DRV8703_BoardChannel_t ch)
{
    if ((uint8_t)ch >= DRV8703_BOARD_CHANNEL_COUNT)
        return 0;
    return &g_drv8703_board[(uint8_t)ch];
}

DRV8703_Status_t DRV8703_BoardInitOne(DRV8703_BoardChannel_t ch)
{
    if ((uint8_t)ch >= DRV8703_BOARD_CHANNEL_COUNT)
        return DRV8703_ERROR_PARAM;

    DRV8703_BoardDeselectAll();

    return DRV8703_Init(&g_drv8703_board[(uint8_t)ch],
                        &g_drv8703_board_config[(uint8_t)ch]);
}

DRV8703_Status_t DRV8703_BoardInitAll(void)
{
    uint8_t i;
    DRV8703_Status_t ret;

    for (i = 0U; i < DRV8703_BOARD_CHANNEL_COUNT; i++)
    {
        ret = DRV8703_BoardInitOne((DRV8703_BoardChannel_t)i);
        if (ret != DRV8703_OK)
            return ret;
    }

    return DRV8703_OK;
}

DRV8703_Status_t DRV8703_BoardApplyDefaultConfig(DRV8703_BoardChannel_t ch)
{
    DRV8703_Handle_t *dev;
    DRV8703_DeviceConfig_t config;
    DRV8703_Status_t ret;

    dev = DRV8703_BoardGet(ch);
    if (dev == 0)
        return DRV8703_ERROR_PARAM;

    ret = DRV8703_Wake(dev);
    if (ret != DRV8703_OK)
        return ret;

    config = DRV8703_DefaultDeviceConfig();
    return DRV8703_Configure(dev, &config);
}
