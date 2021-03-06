/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CALIB_H
#define __CALIB_H
/* Private Macros ------------------------------------------------------------*/
#define U8_KEY_SIZE  (NUM_KEYS)
#define U16_KEY_SIZE ((NUM_KEYS + 1) / 2)
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdlib.h>
#include "stm32f10x_conf.h"
#include "platform_config.h"
/* External variables --------------------------------------------------------*/
extern char _calibdata;
extern char _calibdatasize;
extern char _calibdataend;
/* Exported types ------------------------------------------------------------*/
typedef union calib_data {
        struct {
            uint16_t Xmax;
            uint16_t Xmin;
            uint16_t Xavg;
            uint16_t Ymax;
            uint16_t Ymin;
            uint16_t Yavg;
            uint16_t R        : 10;
            uint16_t axis_rev : 1;
            uint16_t l_r_rev  : 1;
            uint16_t u_d_rev  : 1;
            uint16_t rsvd     : 3;
            uint8_t  KeyMap[U8_KEY_SIZE];
        } st;
        uint16_t ar[7 + U16_KEY_SIZE];
} calib_data_t;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
calib_data_t *flash_claib_write(calib_data_t *st_addr, calib_data_t *data);
calib_data_t *flash_claib_find(calib_data_t *st_addr);
int flash_check_calibrated(calib_data_t *data);
#endif /* _CALIB_H */
