#include "CS5552.h"
#include "filter.h"
#include <stdio.h>
#include <stdbool.h>

static uint32_t s_us_ticks = 0;
uint32_t raw = 0;
bool cs5552_ready = false;
cs5530_t cs5530;

/* 多芯片片选管理 */
static uint8_t  cs5552_chip = 0;
static GPIO_TypeDef* cs_ports[2];
static SlidingAvgFilter voltage_filter_ch0;
static SlidingAvgFilter voltage_filter_ch1;

/* Keep the legacy 3-slot buffer shape so upper layers can migrate incrementally. */
static void CS5552_CompatClearData(cs5552_compat_t *compat_ctx) {
    uint8_t i;

    if (compat_ctx == NULL) {
        return;
    }

    for (i = 0; i < cs5530ChannelNumMax; i++) {
        compat_ctx->Code[i] = 0;
        compat_ctx->Voltage[i] = 0.0f;
        compat_ctx->Value[i] = 0.0f;
    }
}

static uint16_t     cs_pins[2];

void CS5552_SelectChip(uint8_t chip) {
    cs5552_chip = (chip > 1) ? 0 : chip;
}

void CS5552_CS_LOW(void) {
    HAL_GPIO_WritePin(cs_ports[cs5552_chip], cs_pins[cs5552_chip], GPIO_PIN_RESET);
}

void CS5552_CS_HIGH(void) {
    HAL_GPIO_WritePin(cs_ports[cs5552_chip], cs_pins[cs5552_chip], GPIO_PIN_SET);
}

void CS5552_Delay_Init(void) {
    s_us_ticks = SystemCoreClock / 1000000U;
}

void CS5552_Delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}

void CS5552_Delay_us(uint32_t us) {
    if (us == 0 || s_us_ticks == 0) {
        return;
    }

    uint32_t max_us = 0xFFFFFFFFU / s_us_ticks;
    if (us > max_us) {
        us = max_us;
    }
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * s_us_ticks;

    while ((DWT->CYCCNT - start) < ticks);
}

static HAL_StatusTypeDef SPI_TxRxByte(uint8_t txData, uint8_t *rxData) {
    return HAL_SPI_TransmitReceive(CS5552_HSPI, &txData, rxData, 1, CS5552_SPI_TIMEOUT_MS);
}

bool SPI_SendByte(uint8_t data) {
    uint8_t dummy = 0;
    return (SPI_TxRxByte(data, &dummy) == HAL_OK);
}

uint8_t SPI_ReceiveByte(void) {
    uint8_t data = 0x00;
    (void)SPI_TxRxByte(0xFF, &data);
    return data;
}

static uint8_t CS5552_CalcParity(uint8_t cmd) {
    uint8_t temp = cmd & 0xFEu;

    uint8_t count = temp;
    count = count - ((count >> 1) & 0x55u);
    count = (count & 0x33u) + ((count >> 2) & 0x33u);
    count = (count + (count >> 4)) & 0x0Fu;
    return (count & 0x01u) ? (cmd | 0x01u) : (cmd & 0xFEu);
}

void CS5552_Reset_SPI(void) {
    CS5552_CS_LOW();
    CS5552_Delay_us(5);
    (void)SPI_SendByte(0x00);
    (void)SPI_SendByte(0xA5);
    (void)SPI_SendByte(0xFF);
    (void)SPI_SendByte(0x5A);
    CS5552_CS_HIGH();
    CS5552_Delay_ms(2);
}

#define CS5552_REG_24BIT_MODE   0
bool CS5552_WriteReg(uint8_t reg_addr, uint32_t data) {
    if (reg_addr > REG_MAX_ADDR) {
        return false;
    }
    uint8_t cmd = (reg_addr & 0x0Fu) << 3;
    cmd = CS5552_CalcParity(cmd);
#if CS5552_REG_24BIT_MODE

    uint8_t txBuf[4] = {
        cmd,
        (uint8_t)(data >> 16),
        (uint8_t)(data >> 8),
        (uint8_t)(data)
    };
    uint8_t rxBuf[4] = {0};
    CS5552_CS_LOW();
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(CS5552_HSPI, txBuf, rxBuf, 4, CS5552_SPI_TIMEOUT_MS);
#else

    uint8_t txBuf[5] = {
        cmd,
        (uint8_t)(data >> 24),
        (uint8_t)(data >> 16),
        (uint8_t)(data >> 8),
        (uint8_t)(data)
    };
    uint8_t rxBuf[5] = {0};
    CS5552_CS_LOW();
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(CS5552_HSPI, txBuf, rxBuf, 5, CS5552_SPI_TIMEOUT_MS);
#endif
    CS5552_CS_HIGH();

    CS5552_Delay_us(10);
    return (status == HAL_OK);
}

bool CS5552_ReadReg(uint8_t reg_addr, uint32_t *out_data) {
    if (reg_addr > REG_MAX_ADDR || out_data == NULL) {
        return false;
    }
    uint8_t cmd = ((reg_addr & 0x0Fu) << 3) | (1u << 2);
    cmd = CS5552_CalcParity(cmd);
#if CS5552_REG_24BIT_MODE

    uint8_t txBuf[4] = {cmd, 0xFF, 0xFF, 0xFF};
    uint8_t rxBuf[4] = {0};
    CS5552_CS_LOW();
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(CS5552_HSPI, txBuf, rxBuf, 4, CS5552_SPI_TIMEOUT_MS);
#else

    uint8_t txBuf[5] = {cmd, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t rxBuf[5] = {0};
    CS5552_CS_LOW();
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(CS5552_HSPI, txBuf, rxBuf, 5, CS5552_SPI_TIMEOUT_MS);
#endif
    CS5552_CS_HIGH();
    if (status != HAL_OK) {
        return false;
    }
#if CS5552_REG_24BIT_MODE
    *out_data = ((uint32_t)rxBuf[1] << 16) |
                ((uint32_t)rxBuf[2] << 8)  |
                ((uint32_t)rxBuf[3]);
#else
    *out_data = ((uint32_t)rxBuf[1] << 24) |
                ((uint32_t)rxBuf[2] << 16) |
                ((uint32_t)rxBuf[3] << 8)  |
                ((uint32_t)rxBuf[4]);
#endif
    return true;
}

/**
 * @brief 配置单个通道 (清零失调/增益 + 写入 CONV_CONF)
 * @param channel 0=CH0, 1=CH1
 * @return true 成功, false 失败
 * @note  统一使用 DR=12.5Hz, PGA=128x，便于电桥/称重等弱信号采集
 */
static bool CS5552_ConfigChannel(uint8_t channel) {
    uint8_t  reg_os, reg_gain, reg_conv;
    uint32_t conv_conf;

    if (channel == 0) {
        reg_os    = REG_OS_CH0;
        reg_gain  = REG_GAIN_CH0;
        reg_conv  = REG_CONV_CONF0;
        conv_conf = CONV_CONF_CHN_CH0 | CONV_CONF_DR_800HZ | CONV_CONF_GA_128X; /* 0x000000C1 */
    } else if (channel == 1) {
        reg_os    = REG_OS_CH1;
        reg_gain  = REG_GAIN_CH1;
        reg_conv  = REG_CONV_CONF1;
        conv_conf = CONV_CONF_CHN_CH1 | CONV_CONF_DR_800HZ | CONV_CONF_GA_128X; /* 0x000010C1 */
    } else {
        return false;
    }

    /* 清零失调寄存器 */
    if (!CS5552_WriteReg(reg_os, 0x00000000)) {
        printf("CS5552 Init: CH%u OS write failed\r\n", channel);
        return false;
    }
    /* 增益寄存器写入 1.0 (定点 2.22 格式) */
    if (!CS5552_WriteReg(reg_gain, 0x02000000)) {
        printf("CS5552 Init: CH%u GAIN write failed\r\n", channel);
        return false;
    }
    /* 转换参数寄存器：通道选择 + DR + PGA */
    if (!CS5552_WriteReg(reg_conv, conv_conf)) {
        printf("CS5552 Init: CH%u CONV_CONF write failed\r\n", channel);
        return false;
    }
    return true;
}



/**
 * @brief 初始化 CS5552 芯片 (双通道版本)
 * @return true 成功, false 失败
 * @note  完成：复位 → 双通道参数配置 → 系统寄存器配置 → 异常检查 →
 *        双通道 Offset 自校准 → 稳定延时
 */
bool CS5552_Init(void) {
    static bool pin_map_done = false;
    if (!pin_map_done) {
        cs_ports[CS5552_CHIP_0] = P_ADCCS2_GPIO_Port;
        cs_pins[CS5552_CHIP_0]  = P_ADCCS2_Pin;
        cs_ports[CS5552_CHIP_1] = P_ADCCS3_GPIO_Port;
        cs_pins[CS5552_CHIP_1]  = P_ADCCS3_Pin;
        pin_map_done = true;
    }

    /* 初始化 DWT 延时参数 */
    CS5552_Delay_Init();

    CS5552_Reset_SPI();

    /* 1. 触发全局软件复位 */
    if (!CS5552_WriteReg(REG_SYS_CONF0, 0x80000000)) {
        printf("CS5552 Init: REG_SYS_CONF0 write failed\r\n");
        return false;
    }
    CS5552_Delay_ms(50);  /* 复位稳定：内部基准源 + 振荡器启动 */

    /* 2. 双通道参数配置 (校准寄存器清零 + CONV_CONFx) */
    if (!CS5552_ConfigChannel(0)) {
        return false;
    }
    if (!CS5552_ConfigChannel(1)) {
        return false;
    }

    /* 3. 清除复位位，进入正常工作态 */
    if (!CS5552_WriteReg(REG_SYS_CONF0, 0x00010000)) {
        printf("CS5552 Init: REG_SYS_CONF0 clear failed\r\n");
        return false;
    }

    /* 4. 系统配置寄存器 (使能 ADC 模拟核心 / 电流模式 / 时钟) */
    uint32_t sys_conf1 = 0x80110210;
    if (!CS5552_WriteReg(REG_SYS_CONF1, sys_conf1)) {
        printf("CS5552 Init: REG_SYS_CONF1 write failed\r\n");
        return false;
    }
    if (!CS5552_WriteReg(REG_SYS_CONF2, 0x00020000)) {
        printf("CS5552 Init: REG_SYS_CONF2 write failed\r\n");
        return false;
    }

    /* 5. 异常标志检查 */
    uint32_t sys0 = 0;
    if (!CS5552_ReadReg(REG_SYS_CONF0, &sys0)) {
        printf("CS5552 Init: REG_SYS_CONF0 read failed\r\n");
        return false;
    }
    if (sys0 & (SYS_CONF0_ERR_CKS | SYS_CONF0_ERR_C)) {
        printf("CS5552 Init: error flags set, SYS_CONF0=%08X\r\n", (unsigned)sys0);
        return false;
    }

    /* 6. 回读 SYS_CONF1 验证写入完整性 */
    uint32_t readback = 0;
    if (!CS5552_ReadReg(REG_SYS_CONF1, &readback)) {
        printf("CS5552 Init: REG_SYS_CONF1 readback failed\r\n");
        return false;
    }
    if (readback != sys_conf1) {
        printf("CS5552 Init: REG_SYS_CONF1 mismatch W:%08X R:%08X\r\n",
               (unsigned)sys_conf1, (unsigned)readback);
        return false;
    }

    /* 7. 双通道 Offset 自校准 (上电后必须执行一次以消除内部失调) */
    // printf("CS5552 Init: CH0 offset calibrating...\r\n");
    if (!CS5552_Offset_SelfCalibration(CS5552_CONFIG_IDX_CH0)) {
        printf("CS5552 Init: CH0 offset cal failed\r\n");
        return false;
    }
    // printf("CS5552 Init: CH1 offset calibrating...\r\n");
    if (!CS5552_Offset_SelfCalibration(CS5552_CONFIG_IDX_CH1)) {
        printf("CS5552 Init: CH1 offset cal failed\r\n");
        return false;
    }

    /* 8. 校准完成后稳定延时，确保后续转换数据有效 */
    CS5552_Delay_ms(20);

    // printf("CS5552 Init: dual-channel ready\r\n");
    return true;
}

static bool CS5552_WaitSDO_Low(uint32_t timeout_ms) {
    uint32_t tickstart = HAL_GetTick();
    while (CS5552_SDO_READ() != GPIO_PIN_RESET) {
        if ((HAL_GetTick() - tickstart) > timeout_ms) {
            return false;
        }
    }
    return true;
}

bool CS5552_Offset_SelfCalibration(uint8_t conv_conf_idx) {
    if (conv_conf_idx > 0x03) {
        return false;
    }
    uint8_t cmd = (1u << 7) | ((conv_conf_idx & 0x03u) << 4) | (CMD_MOD_OFFSET_CAL << 1);
    cmd = CS5552_CalcParity(cmd);
    CS5552_CS_LOW();
    if (!SPI_SendByte(cmd)) {
        CS5552_CS_HIGH();
        return false;
    }
    if (!CS5552_WaitSDO_Low(CS5552_CALIB_TIMEOUT_MS)) {
        CS5552_CS_HIGH();
        return false;
    }

    CS5552_SDO_READ() != GPIO_PIN_RESET;

    uint8_t txBuf[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t rxBuf[5] = {0};
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(CS5552_HSPI, txBuf, rxBuf, 5, CS5552_SPI_TIMEOUT_MS);
    CS5552_CS_HIGH();
    return (status == HAL_OK);
}

bool CS5552_StartContConv(uint8_t conv_conf_idx) {
    if (conv_conf_idx > 0x03) {
        return false;
    }
    uint8_t cmd = (1u << 7) | ((conv_conf_idx & 0x03u) << 4) | (CMD_MOD_CONT_CONV << 1);
    cmd = CS5552_CalcParity(cmd);
    CS5552_CS_LOW();
    CS5552_Delay_us(10);
    if (!SPI_SendByte(cmd)) {
        CS5552_CS_HIGH();
        printf("ContConv: cmd send failed\r\n");
        return false;
    }
    CS5552_Delay_us(20);

    /* CSHIGH_MODE=1 时，命令锁存后 CS 可释放，ADC 内部继续转换 */
    CS5552_CS_HIGH();
    return true;
}

/**
 * @brief  Poll one CS5552 chip for a fresh continuous-conversion sample
 * @param  chip        Selected chip index
 * @param  pga_gain    Gain used for voltage conversion
 * @param  out_data    Output signed ADC code
 * @param  out_voltage Output filtered voltage value
 * @retval true Sample accepted, false no fresh/valid sample
 */
bool CS5552_ReadChipContDataNonBlocking(uint8_t chip,
                                        uint8_t pga_gain,
                                        int32_t *out_data,
                                        float *out_voltage) {
    uint32_t adc_raw = 0;
    uint32_t top3;
    int32_t adc_24bit;
    float voltage_mv;
    SlidingAvgFilter *filter_ctx;

    if (out_data == NULL || out_voltage == NULL) {
        return false;
    }

    if (chip == CS5552_CHIP_0) {
        filter_ctx = &voltage_filter_ch0;
    } else if (chip == CS5552_CHIP_1) {
        filter_ctx = &voltage_filter_ch1;
    } else {
        return false;
    }

    /* Poll-only read: return quickly if this chip has no fresh conversion yet. */
    CS5552_SelectChip(chip);

    if (!CS5552_ReadReg(REG_CONV_DATA, &adc_raw)) {
        return false;
    }

    top3 = (adc_raw >> 29) & 0x7u;
    if (!(top3 == 0x2u || top3 == 0x3u || top3 == 0x4u || top3 == 0x5u ||
          adc_raw == 0xFFFFFFFFu || adc_raw == 0xFFFFFFFEu || adc_raw == 0x0u)) {
        return false;
    }

    /* Convert the packed conversion register into a signed 24-bit sample. */
    adc_24bit = (int32_t)(adc_raw & 0xFFFFFFFEu) >> 7;
    voltage_mv = CS5552_ConvertToVoltage(adc_24bit, pga_gain);

    *out_data = adc_24bit;
    /* A light moving average keeps the old sampling path stable during migration. */
    *out_voltage = Filter_Update(filter_ctx, voltage_mv);
    return true;
}
float CS5552_ConvertToVoltage(int32_t raw_code, uint8_t pga_gain) {
    if (pga_gain == 0) {
        return 0.0f;
    }

    float full_scale_mv = (float)CS5552_VREF_MV / (float)pga_gain;
    return ((float)raw_code * full_scale_mv) / (float)(1L << 23);
}


bool CS5552_CompatDualInit(void) {
    uint8_t chip0_ready = 0;
    uint8_t chip1_ready = 0;

    cs5552_ready = false;

    /* The compatibility mapping is fixed for now:
     * chip0.ch1 -> force, chip1.ch0 -> strain2, slot1 stays unused.
     */
    CS5552_SelectChip(CS5552_CHIP_0);
    if (CS5552_Init()) {
        printf("CS5552-0 Init OK\r\n");
        if (CS5552_StartContConv(CS5552_CONFIG_IDX_CH1)) {
            chip0_ready = 1;
        } else {
            printf("CS5552-0 StartContConv Fail!\r\n");
        }
    } else {
        printf("CS5552-0 Init Fail!\r\n");
    }

    CS5552_SelectChip(CS5552_CHIP_1);
    if (CS5552_Init()) {
        printf("CS5552-1 Init OK\r\n");
        if (CS5552_StartContConv(CS5552_CONFIG_IDX_CH0)) {
            chip1_ready = 1;
        } else {
            printf("CS5552-1 StartContConv Fail!\r\n");
        }
    } else {
        printf("CS5552-1 Init Fail!\r\n");
    }

    Filter_Init(&voltage_filter_ch0);
    Filter_Init(&voltage_filter_ch1);

    cs5552_ready = (chip0_ready && chip1_ready) ? true : false;
    return cs5552_ready;
}

void CS5552_CompatInit(cs5552_compat_t *compat_ctx)
{
    if (compat_ctx == NULL) {
        return;
    }

    compat_ctx->runState = cs5530NoStart;
    compat_ctx->csChannel = cs5530Channel1;
    CS5552_CompatClearData(compat_ctx);

    cs5552_ready = CS5552_CompatDualInit();
    compat_ctx->runState = cs5552_ready ? cs5530RunNormal : cs5530RunAbnormal;
}

void CS5552_CompatDataGet(cs5552_compat_t *compat_ctx)
{
    int32_t code = 0;
    float voltage = 0.0f;

    if (compat_ctx == NULL) {
        return;
    }

    if (!cs5552_ready || compat_ctx->runState != cs5530RunNormal) {
        return;
    }

    /* Preserve the original round-robin order expected by the old control path. */
    switch (compat_ctx->csChannel) {
    case cs5530Channel1:
        compat_ctx->Code[cs5530Channel1] = 0;
        compat_ctx->Voltage[cs5530Channel1] = 0.0f;
        compat_ctx->Value[cs5530Channel1] = 0.0f;
        compat_ctx->csChannel = cs5530Channel2;
        break;

    case cs5530Channel2:
        if (CS5552_ReadChipContDataNonBlocking(CS5552_CHIP_0, 128, &code, &voltage)) {
            compat_ctx->Code[cs5530Channel2] = code;
            compat_ctx->Voltage[cs5530Channel2] = voltage;
            compat_ctx->Value[cs5530Channel2] = voltage;
            compat_ctx->csChannel = cs5530Channel3;
            printf("cs5552CompatForce value=%f\r\n", compat_ctx->Value[cs5530Channel2]);
        }
        break;

    case cs5530Channel3:
        if (CS5552_ReadChipContDataNonBlocking(CS5552_CHIP_1, 128, &code, &voltage)) {
            compat_ctx->Code[cs5530Channel3] = code;
            compat_ctx->Voltage[cs5530Channel3] = voltage;
            compat_ctx->Value[cs5530Channel3] = voltage;
            compat_ctx->csChannel = cs5530Channel1;
            printf("cs5552CompatStrain2 value=%f\r\n", compat_ctx->Value[cs5530Channel3]);
        }
        break;

    default:
        compat_ctx->csChannel = cs5530Channel1;
        break;
    }
}

