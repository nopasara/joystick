/**
  ******************************************************************************
  * @file    hw_config.h
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


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "platform_config.h"
#include "usb_type.h"
#include "calib.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include "usb_istr.h"
#include <errno.h>
#include <math.h>
/* Exported types ------------------------------------------------------------*/
typedef struct JoySettings {
    uint16_t *X_map;
    uint16_t *Y_map;
    float    step_coef;
    float    x_norm_coef;
    float    y_norm_coef;
} JoySettings_t;

/* Exported constants --------------------------------------------------------*/
/* Exported varables  --------------------------------------------------------*/
extern calib_data_t *CalData;
extern int JoystickCalibrated;
/* Exported macro ------------------------------------------------------------*/
#define ALIGN(val, byte)    ((((val) + (byte) - 1) / (byte)) * (byte))
#define MAX(a, b)           (((a) > (b)) ?  (a) : (b))
#define MIN(a, b)           (((a) < (b)) ?  (a) : (b))
#define ADC_CYC_DELAY(time) {                          \
    register int i = time * BOARD_ADC_DELAY_RATIO;     \
    __asm__ __volatile__ ("1:SUBS %0, #1\n"            \
                            "BNE  1b\n" ::"r" (i):);}
/* Exported define -----------------------------------------------------------*/
#define REP_CALIBRATE           (0x4)
#define ADC_CH_COUNT            (1)
#define BOARD_ADC_DELAY_RATIO   (6 / 2)
#define ADC_SAMPLE_COUNT        (100)
/* Exported functions ------------------------------------------------------- */
void Set_System(void);
void Set_ADCClock(void);
void Set_USBClock(void);
void GPIO_AINConfig(void);
void Enter_LowPowerMode(void);
void Leave_LowPowerMode(void);
void USB_Interrupts_Config(void);
void USB_Cable_Config (FunctionalState NewState);
void Joystick_Send(calib_data_t *cal, JoySettings_t *set);
void Keyboard_Send(calib_data_t *CalibData);
void Get_SerialNum(void);
void ADC_avg_read(uint32_t iter, uint16_t *adc1_val, uint16_t *adc2_val);
void Joystick_setup(calib_data_t *cal_data, JoySettings_t *set);
void calibrate_cmd_process(volatile uint8_t *cmdBuff, calib_data_t **CalData, JoySettings_t *JoySet);
void Signal_Unconfigured(uint8_t enable);
#endif  /*__HW_CONFIG_H*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
