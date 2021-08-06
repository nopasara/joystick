/**
  ******************************************************************************
  * @file    colib.c
  * @author  Maksim Kramarenko, nopasara@gmail.com
  * @version V1
  * @date    09-JUL-2021
  * @brief   Joystick calibration helper functions
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "calib.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define flash_calib_next(addr, pend)      ((++(addr) <= (pend)) ? (addr) : NULL)
#define ADDR_IN_RANGE(addr, start, end) ((addr < start)                        \
                                         ? 0 : (addr > end) ? 0 : 1)
/* Private variables ---------------------------------------------------------*/
calib_data_t *CalData;

/**
  * Function Name  : flash_calib_find
  * Description    : Find calibration data address in the flash.
  * Input          : None.
  * Output         : None.
  * Return         : Address of the calibration data structure,
  *                  NULL if the page limit is reached.
  */
calib_data_t *flash_claib_find(calib_data_t *st_addr)
{
    calib_data_t *pagestart = (calib_data_t *)&_calibdata;
    calib_data_t *pageend = (calib_data_t *)&_calibdataend - sizeof(calib_data_t);

    assert_param(ADDR_IN_RANGE(st_addr, pagestart, pageend));

    while (st_addr->st.Xmax == 0
            && (st_addr = flash_calib_next(st_addr, pageend)));

    return st_addr;
}

/**
  * Function Name  : flash_calib_next_wr
  * Description    : Find calibration data address in the flash that can be
  *                  written.
  * Input          : None.
  * Output         : None.
  * Return         : Address to a writable calibration structure.
  */
calib_data_t *flash_claib_next_wr(calib_data_t *st_addr)
{
    calib_data_t *pagestart = (calib_data_t *)&_calibdata;
    calib_data_t *pageend = (calib_data_t *)&_calibdataend - sizeof(calib_data_t);

    assert_param(ADDR_IN_RANGE(st_addr, pagestart, pageend));

    if (!(st_addr = flash_calib_next(st_addr,
                    (calib_data_t *)&_calibdataend - sizeof(calib_data_t)))) {
        st_addr = (calib_data_t *)&_calibdata;
        FLASH_Unlock();
        FLASH_Status st = FLASH_ErasePage((uint32_t)st_addr);
        FLASH_Lock();
    }

    return st_addr;
}

/**
  * Function Name  : flash_calib_write
  * Description    : Zero out calibration data and writtable new data
  *                  to the next position in the flash
  * Input          : None.
  * Output         : None.
  * Return         : Address to a writable calibration structure.
  */
calib_data_t *flash_claib_write(calib_data_t *st_addr, calib_data_t *data)
{
    calib_data_t *pagestart = (calib_data_t *)&_calibdata;
    calib_data_t *pageend = (calib_data_t *)&_calibdataend - sizeof(calib_data_t);

    assert_param(ADDR_IN_RANGE(st_addr, pagestart, pageend));
    assert_param(sizeof(CalData.ar[0] == sizeof(uint16_t)));

    uint8_t halfword_co = sizeof(calib_data_t) / sizeof (uint16_t);
    FLASH_Status st;

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR | FLASH_FLAG_EOP | FLASH_FLAG_OPTERR);
    for (register uint32_t i = 0; i < halfword_co; ++i) {
        FLASH_ProgramHalfWord((uint32_t)&st_addr->ar[i], 0);
    }
    FLASH_Lock();

    st_addr = flash_claib_next_wr(st_addr);

    FLASH_Unlock();
    for (register uint32_t i = 0; i < halfword_co; ++i) {
        st = FLASH_ProgramHalfWord((uint32_t)&st_addr->ar[i], data->ar[i]);
        assert_param(st == FLASH_COMPLETE);
    }
    FLASH_Lock();

    return st_addr;
}

/**
  * Function Name  : flash_check_calibrated
  * Description    : Check whether the calibration data is valid
  * Input          : None.
  * Output         : None.
  * Return         : 1 - calibration data is valid,
  *                  0 - calibration data is invalid.
  */

int flash_check_calibrated(calib_data_t *data)
{
    assert_param(sizeof(data->ar[0] == sizeof(uint16_t)));
    register int i = 0;
    register int count = (sizeof(data->ar) / sizeof(data->ar[0]));

    while(i < count && data->ar[i] != 0 && data->ar[i] != 0xffff)
        i++;
    return (i == count) ? 1 : 0;
}

