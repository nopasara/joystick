/**
  ******************************************************************************
  * @file    hw_config.c
  * @author  MCD Application Team
  * @version V4.1.0
  * @date    26-May-2017
  * @brief   Hardware Configuration & Setup
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "hw_config.h"

/* Private typedef -----------------------------------------------------------*/
enum calib_cmd_type {
    CALIBRATE_RANGE,
    CALIBRATE_RANGE_STOP,
    CALIBRATE_AVG,
    CALIBRATE_AVG_STOP,
    CALIBRATE_MANUAL,
    CALIBRATE_SHOW,
};

/* Private define ------------------------------------------------------------*/
#define AXIS_MAX_VAL                    (127)
/* ln(128) */
#define EXP_COEF                        (4.852)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ErrorStatus HSEStartUpStatus;
const uint16_t key_row_sel[KEY_NUM_ROWS] = {KEY_ROW_SEL_VAL};
uint16_t X, Y;
int JoystickCalibrated;

/* Extern variables ----------------------------------------------------------*/
extern __IO uint8_t PrevXferComplete;

/* Private function prototypes -----------------------------------------------*/
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len);
static void Delay(__IO uint32_t nTime);
static void ADC_Configuration(void);
/* Private functions ---------------------------------------------------------*/

/**
  * Function Name  : Set_System
  * Description    : Configures Main system clocks & power.
  * Input          : None.
  * Return         : None.
  */
void Set_System(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;  
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32xxx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32xxx.c file
     */ 

  /******************************************/
  /*           Enable the PWR clock         */
  /******************************************/
  /* Enable all GPIOs Clock*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ALLGPIO, ENABLE);

  /********************************************/
  /*  Configure USB DM/DP pins                */
  /********************************************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /********************************************/
  /*  Enable joystich pins                    */
  /********************************************/
  GPIO_InitStructure.GPIO_Pin = JS_ADC1_PIN | JS_ADC2_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(JS_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = JS_BTN_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(JS_PORT, &GPIO_InitStructure);

  /********************************************/
  /*  Enable keyboard pins                    */
  /********************************************/
  GPIO_InitStructure.GPIO_Pin = KEY_ROW_SEL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(KEY_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = KEY_ROW;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(KEY_PORT, &GPIO_InitStructure);

  /********************************************/
  /* Enable the USB PULL UP                   */
  /********************************************/
  /* USB_DISCONNECT used as USB pull-up */
  GPIO_InitStructure.GPIO_Pin = USB_DISCONNECT_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_Init(USB_DISCONNECT, &GPIO_InitStructure);

  GPIO_WriteBit(USB_DISCONNECT, USB_DISCONNECT_PIN, Bit_SET);

  /********************************************/
  /*   configures the hardware resources      */
  /********************************************/

  ADC_Configuration();
  CalData = flash_claib_find((calib_data_t *)&_calibdata);
  JoystickCalibrated = flash_check_calibrated(CalData);
}

/*******************************************************************************
* Function Name : ADC_Configuration.
* Description   : Configure the ADC1 and ADC2 in double mode.
* Input         : None.
* Output        : None.
* Return value  : The direction value.
*******************************************************************************/
void ADC_Configuration(void)
{
  ADC_InitTypeDef ADC_settings = {  .ADC_Mode = ADC_Mode_RegSimult,
                                    .ADC_ScanConvMode = DISABLE,
                                    .ADC_ContinuousConvMode = DISABLE,
                                    .ADC_ExternalTrigConv = ADC_ExternalTrigConv_None,
                                    .ADC_DataAlign = ADC_DataAlign_Right,
                                    .ADC_NbrOfChannel = ADC_CH_COUNT};

  RCC_ADCCLKConfig(RCC_PCLK2_Div6);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
  for (register int i = 1; i <= ADC_CH_COUNT; i++) {
    ADC_RegularChannelConfig(ADC1, JS_ADC1, i, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC2, JS_ADC2, i, ADC_SampleTime_239Cycles5);
  }
  ADC_ExternalTrigConvCmd(ADC2, ENABLE);
  ADC_Init(ADC1, &ADC_settings);
  ADC_Init(ADC2, &ADC_settings);
  ADC_Cmd(ADC1, ENABLE);
  ADC_Cmd(ADC2, ENABLE);
  /* >2 ADC clock delay */
  ADC_CYC_DELAY(3);

  ADC_ResetCalibration(ADC1);
  ADC_ResetCalibration(ADC2);
  while(ADC_GetResetCalibrationStatus(ADC2));

  ADC_StartCalibration(ADC1);
  ADC_StartCalibration(ADC2);

  while(ADC_GetCalibrationStatus(ADC2) != RESET) {}
}

void ADC_avg_read(uint32_t iter, uint16_t *adc1_val, uint16_t *adc2_val)
{
  register int tmp_1 = *adc1_val, tmp_2 = *adc2_val;

  for (register int j = 0; j < iter; j++) {
    /* >14 ADC clock delay */
    ADC_CYC_DELAY(15);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    for (register int i = 0; i < ADC_CH_COUNT; i++) {
      while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) != SET) {}
      int count = (j * ADC_CH_COUNT) + i + 1;
      uint32_t c_val1 = ADC_GetConversionValue(ADC1);
      uint32_t c_val2 = ADC_GetConversionValue(ADC2);

      tmp_1 += ((int)(c_val1 & 0xffff) - tmp_1) / count;
      tmp_2 += ((int)(c_val2 & 0xffff) - tmp_2) / count;
    }
  }

  *adc1_val = tmp_1;
  *adc2_val = tmp_2;
}
/**
  * Function Name  : Joystick_Setup
  * Description    : Fill the joystick setup structure.
  * Input          : cal_data - calibration data structure
  *                  set - joystick settings structure
  * Output         : None.
  * Return         : None.
  */
void Joystick_setup(calib_data_t *cal_data, JoySettings_t *set)
{
    if (!JoystickCalibrated) {
        set->X_map = &X;
        set->Y_map = &Y;
        return;
    }

    if (cal_data->st.axis_rev) {
        set->X_map = &Y;
        set->Y_map = &X;
    } else {
        set->X_map = &X;
        set->Y_map = &Y;
    }

    set->R_norm_coef = AXIS_MAX_VAL - cal_data->st.R;

    set->X_pos_step = (float)AXIS_MAX_VAL
                        / (cal_data->st.Xmax - cal_data->st.Xavg);
    set->X_neg_step = (float)AXIS_MAX_VAL
                        / (cal_data->st.Xmin - cal_data->st.Xavg);
    set->Y_pos_step = (float)AXIS_MAX_VAL
                        / (cal_data->st.Ymax - cal_data->st.Yavg);
    set->Y_neg_step = (float)AXIS_MAX_VAL
                        / (cal_data->st.Ymin - cal_data->st.Yavg);
}

/**
  * Function Name  : Set_USBClock
  * Description    : Configures USB Clock input (48MHz).
  * Input          : None.
  * Output         : None.
  * Return         : None.
  */
void Set_USBClock(void)
{
  /* Select USBCLK source */
  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
  /* Enable the USB clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
}

/**
  * Function Name  : Leave_LowPowerMode.
  * Description    : Restores system clocks and power while exiting suspend mode.
  * Input          : None.
  * Output         : None.
  * Return         : None.
  */
void Leave_LowPowerMode(void)
{
  DEVICE_INFO *pInfo = &Device_Info;

  /* Set the device state to the correct state */
  if (pInfo->Current_Configuration != 0)
  {
    /* Device configured */
    bDeviceState = CONFIGURED;
  }
  else
  {
    bDeviceState = ATTACHED;
  }

#ifdef USB_LOW_PWR_MGMT_SUPPORT
  /* Reset SLEEPDEEP bit of Cortex System Control Register */
  SCB->SCR &= (uint32_t)~((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));

  /*Enable SystemCoreClock*/
  SystemInit();

#endif /* USB_LOW_PWR_MGMT_SUPPORT */
}

/**
  * Function Name  : USB_Interrupts_Config.
  * Description    : Configures the USB interrupts.
  * Input          : None.
  * Output         : None.
  * Return         : None.
  */
void USB_Interrupts_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* 2 bit for pre-emption priority, 2 bits for subpriority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  /* Enable the USB interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

#ifdef USB_LOW_PWR_MGMT_SUPPORT
  /* Enable the USB Wake-up interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USBWakeUp_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_Init(&NVIC_InitStructure); 
#endif /* USB_LOW_PWR_MGMT_SUPPORT */ 
}

/**
  * Function Name  : USB_Cable_Config.
  * Description    : Software Connection/Disconnection of USB Cable.
  * Input          : NewState: new state.
  * Output         : None.
  * Return         : None
  */
void USB_Cable_Config (FunctionalState NewState)
{
  if (NewState != DISABLE)
  {
    GPIO_ResetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
  }
  else
  {
    GPIO_SetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
  }
}

/**
  * Function Name : joy_btn_state.
  * Description   : Get joystick button state
  * Input         : None.
  * Output        : None.
  * Return value  : 1 if button is pushed,
  *                 0 if button released
  */
uint8_t joy_btn_state(void)
{
    return (GPIO_ReadInputDataBit(JS_PORT, JS_BTN_PIN)) ? 0 : 1;
}

/**
  * Function Name : Joystick_Send.
  * Description   : prepares buffer to be sent containing Joystick event infos.
  * Input         : set - joystick settings.
  * Output        : None.
  * Return value  : None.
  */
void Joystick_Send(calib_data_t *cal, JoySettings_t *set)
{
  uint8_t Mouse_Buffer[5] = {1, 0, 0, 0};
  uint8_t X_norm, Y_norm;
  uint8_t Rvect;
  int8_t  X_sign, Y_sign;
  static uint8_t btn_last;

  ADC_avg_read(ADC_SAMPLE_COUNT, set->X_map, set->Y_map);
  /* Center and adjust AXIS */
  if (X < cal->st.Xavg) {
    X = MAX(X, cal->st.Xmin);
    X_norm = (int8_t)((X - cal->st.Xavg) * set->X_neg_step);
    X_sign = (cal->st.l_r_rev) ? 1 : -1;
  } else {
    X = MIN(X, cal->st.Xmax);
    X_norm = (int8_t)((X - cal->st.Xavg) * set->X_pos_step);
    X_sign = (cal->st.l_r_rev) ? -1 : 1;
  }
  if (Y < cal->st.Yavg) {
    Y = MAX(Y, cal->st.Ymin);
    Y_norm = (int8_t)((Y - cal->st.Yavg) * set->Y_neg_step);
    Y_sign = (cal->st.u_d_rev) ? 1 : -1;
  } else {
    Y = MIN(Y, cal->st.Ymax);
    Y_norm = (int8_t)((Y - cal->st.Yavg) * set->Y_pos_step);
    Y_sign = (cal->st.u_d_rev) ? -1 : 1;
  }
  /* Calculate radial vector */
  Rvect = sqrt((X_norm * X_norm) + (Y_norm * Y_norm));
  /* Fix bad calibration values */
  Rvect = (Rvect > AXIS_MAX_VAL) ? AXIS_MAX_VAL : Rvect;
  /* Imply the dead zone */
  if (Rvect > cal->st.R) {
    Rvect -= cal->st.R;
    /* Imply the exponenitional acceleration */
    Rvect = exp((Rvect / set->R_norm_coef) * EXP_COEF) - 1;
    /* Convert radial to square acceleration */
    if (X_norm > Y_norm) {
        float coef = (float)Y_norm / X_norm;
        X_norm = Rvect * X_sign;
        Y_norm = Rvect * coef * Y_sign;
    } else {
        float coef = (float)X_norm / Y_norm;
        Y_norm = Rvect * Y_sign;
        X_norm = Rvect * coef * X_sign;
    }
  } else {
    X_norm = 0;
    Y_norm = 0;
  }
  if (X_norm != 0 || Y_norm != 0 || btn_last != joy_btn_state()) {
    btn_last = joy_btn_state();
    Mouse_Buffer[1] = btn_last;
    Mouse_Buffer[2] = X_norm;
    Mouse_Buffer[3] = Y_norm;

    /* Reset the control token to inform upper layer that a transfer is ongoing */
    PrevXferComplete = 0;

    /* Copy mouse position info in ENDP1 Tx Packet Memory Area*/
    USB_SIL_Write(EP1_IN, Mouse_Buffer, 4);

    /* Enable endpoint for transmission */
    SetEPTxValid(ENDP1);
  }
}

/**
  * Function Name : Keyboard_Send.
  * Description   : prepares buffer to be sent containing keybaord event infos.
  * Input         : KeyMap mapping of the key values.
  * Output        : None.
  * Return value  : None.
  */
void Keyboard_Send(uint8_t *KeyMap)
{
    assert_param(KEY_BTNS_ROW * KEY_NUM_ROWS < 32);

    static uint32_t btn_state;
    register uint32_t data = 0;

    for (register int i = 0; i < KEY_NUM_ROWS; ++i) {
        uint32_t shift = i * KEY_BTNS_ROW;

        GPIO_WriteBit(KEY_PORT, key_row_sel[i], Bit_SET);
        while (GPIO_ReadInputDataBit(KEY_PORT, key_row_sel[i]) == Bit_RESET);
        data |= ((GPIO_ReadInputData(KEY_PORT) & KEY_ROW) >> KEY_ROW_SHIFT) << shift;
        GPIO_WriteBit(KEY_PORT, key_row_sel[i], Bit_RESET);
    }
    if (btn_state != data) {
        int idx;
        int c_idx = 0;
        int i = 0;
        uint8_t Keyboard_Buffer[2 + KEY_USB_DESC_MAX] = {2, };

        btn_state = data;
        while ((idx = __builtin_ffs(data)) && (i < KEY_USB_DESC_MAX)) {
            data >>= idx;
            c_idx += (idx - 1);
            Keyboard_Buffer[2 + i++] = KeyMap[c_idx];
        }
        /* Reset the control token to inform upper layer that a transfer is ongoing */
        PrevXferComplete = 0;

        /* Copy mouse position info in ENDP1 Tx Packet Memory Area*/
        USB_SIL_Write(EP1_IN, Keyboard_Buffer, 2 + KEY_USB_DESC_MAX);

        /* Enable endpoint for transmission */
        SetEPTxValid(ENDP1);
    }
}
/**
  * Function Name : calibrate_cmd_process.
  * Description   : Process calibration command.
  * Input         : cmdBuff - command buffer,
  *                 CalData - calibration data structure,
  *                 JoySet - joystick settings structure
  * Output        : None.
  * Return value  : None.
  */
void calibrate_cmd_process(volatile uint8_t *cmdBuff, calib_data_t **CalData, JoySettings_t *JoySet)
{
    uint8_t buf[16] = {3, };
    calib_data_t *buf_w = (calib_data_t *)&buf[1];
    static uint16_t count = 1,
                    CalibXmin = 0xffff,
                    CalibXmax = 0,
                    CalibYmax = 0,
                    CalibYmin = 0xffff;
    static int16_t  CalibXavg = 0,
                    CalibYavg = 0;

    switch (cmdBuff[1]) {
    case CALIBRATE_RANGE:
        while ((cmdBuff[0] == REP_CALIBRATE) && (cmdBuff[1] != CALIBRATE_RANGE_STOP)) {
            ADC_avg_read(ADC_SAMPLE_COUNT, JoySet->X_map, JoySet->Y_map);
            CalibXmax = MAX(CalibXmax, X);
            CalibXmin = MIN(CalibXmin, X);
            CalibYmax = MAX(CalibYmax, Y);
            CalibYmin = MIN(CalibYmin, Y);
        }
        buf_w->st = (*CalData)->st;
        buf_w->st.Xmax = CalibXmax;
        buf_w->st.Xmin = CalibXmin;
        buf_w->st.Ymax = CalibYmax;
        buf_w->st.Ymin = CalibYmin;
        *CalData = flash_claib_write(*CalData, buf_w);
        JoystickCalibrated = flash_check_calibrated(*CalData);
        Joystick_setup(*CalData, JoySet);
        CalibXmax = 0;
        CalibYmax = 0;
        CalibXmin = 0xffff;
        CalibYmin = 0xffff;
        if (cmdBuff[0] == REP_CALIBRATE)
            cmdBuff[0] = 0;
        break;
    case CALIBRATE_AVG:
        while ((cmdBuff[0] == REP_CALIBRATE) && (cmdBuff[1] != CALIBRATE_AVG_STOP)) {
            ADC_avg_read(ADC_SAMPLE_COUNT, JoySet->X_map, JoySet->Y_map);
            CalibXavg += ((int16_t)X - CalibXavg) / count;
            CalibYavg += ((int16_t)Y - CalibYavg) / count;
            count++;
        }
        buf_w->st = (*CalData)->st;
        buf_w->st.Xavg = CalibXavg;
        buf_w->st.Yavg = CalibYavg;
        *CalData = flash_claib_write(*CalData, buf_w);
        JoystickCalibrated = flash_check_calibrated(*CalData);
        Joystick_setup(*CalData, JoySet);
        CalibXavg = 0;
        CalibYavg = 0;
        count = 1;
        if (cmdBuff[0] == REP_CALIBRATE)
            cmdBuff[0] = 0;
        break;
    case CALIBRATE_MANUAL:
        *CalData = flash_claib_write(*CalData, (calib_data_t *)&cmdBuff[2]);
        JoystickCalibrated = flash_check_calibrated(*CalData);
        Joystick_setup(*CalData, JoySet);
        if (cmdBuff[0] == REP_CALIBRATE)
            cmdBuff[0] = 0;
        break;
    case CALIBRATE_SHOW:
        buf_w->st = (*CalData)->st;
        USB_SIL_Write(EP1_IN, buf, 15);
        SetEPTxValid(ENDP1);
        if (cmdBuff[0] == REP_CALIBRATE)
            cmdBuff[0] = 0;
        break;
    }
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
static void Delay(__IO uint32_t nTime)
{
  for(; nTime != 0; nTime--);
}

/**
  * Function Name  : Get_SerialNum.
  * Description    : Create the serial number string descriptor.
  * Input          : None.
  * Output         : None.
  * Return         : None.
  */
void Get_SerialNum(void)
{
  uint32_t Device_Serial0, Device_Serial1, Device_Serial2;

  Device_Serial0 = *(uint32_t*)ID1;
  Device_Serial1 = *(uint32_t*)ID2;
  Device_Serial2 = *(uint32_t*)ID3;

  Device_Serial0 += Device_Serial2;

  if (Device_Serial0 != 0)
  {
    IntToUnicode (Device_Serial0, &Joystick_StringSerial[2] , 8);
    IntToUnicode (Device_Serial1, &Joystick_StringSerial[18], 4);
  }
}

/**
  * Function Name  : HexToChar.
  * Description    : Convert Hex 32Bits value into char.
  * Input          : None.
  * Output         : None.
  * Return         : None.
  */
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len)
{
  uint8_t idx = 0;
  
  for( idx = 0 ; idx < len ; idx ++)
  {
    if( ((value >> 28)) < 0xA )
    {
      pbuf[ 2* idx] = (value >> 28) + '0';
    }
    else
    {
      pbuf[2* idx] = (value >> 28) + 'A' - 10; 
    }

    value = value << 4;

    pbuf[ 2* idx + 1] = 0;
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
