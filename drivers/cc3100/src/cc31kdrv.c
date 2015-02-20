// --------------------------------------------------------------------------------------
// Module     : CC31KDRV
// Author     : N. El-Fata
// --------------------------------------------------------------------------------------
// Pulse Innovation, Inc. www.pulseinnovation.com
// Copyright (c) 2014, Pulse Innovation, Inc. All rights reserved.
// --------------------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>

#include "py/mpconfig.h"
#include "stm32f4xx_hal.h"
#include "log.h"

#include "py/nlr.h"
#include "py/misc.h"
#include "py/qstr.h"
#include "py/obj.h"
#include "py/runtime.h"
#include "pin.h"
#include "genhdr/pins.h"
#include "led.h"
#include "extint.h"
#include "spi.h"


#include "cc31kdrv.h"

// IRQ on PA14, input, pulled up, active low
// EN on PC7, output, active high
// CS on PC6, output, active low
// SPI2 on PB15=MOSI, PB14=MISO, PB13=SCK
// SCK for CC3000: max 16MHz, low when idle, data sampled on falling edge

#define PIN_CS pin_B12 // Y5
#define PIN_EN pin_B9 // Y4
#define PIN_IRQ pin_B8 // Y3
#define SPI_HANDLE SPIHandle2

STATIC volatile irq_handler_t cc31k_IrqHandler = 0;
STATIC mp_obj_t irq_callback(mp_obj_t line);

static int cc31k_transceive(unsigned char *data, uint16_t size);

static void cc31k_en(int val)
{   // use the nHIB signal
    if(val)
    {
        PIN_EN.gpio->BSRRL = PIN_EN.pin_mask; // set pin high
        // it takes 50ms to wake up the chip from sleep, so we add a delay
        // in order to make sure the chip will not be accessed during that time
        HAL_Delay(50);
    }
    else
    {
        PIN_EN.gpio->BSRRH = PIN_EN.pin_mask; // set pin low
    }
}

void cc31k_enable(void)
{
    cc31k_en(1);
}

void cc31k_disable(void)
{
    cc31k_en(0);
}

void cc31k_cs(int val)
{
    if(val)
    {
        PIN_CS.gpio->BSRRL = PIN_CS.pin_mask; // set pin high
    }
    else
    {
        PIN_CS.gpio->BSRRH = PIN_CS.pin_mask; // set pin low
    }
}

void cc31k_cs_assert(void)
{
    cc31k_cs(0);
}

void cc31k_cs_deassert(void)
{
    cc31k_cs(1);
}

STATIC mp_obj_t irq_callback(mp_obj_t line)
{
    if(cc31k_IrqHandler)
    {
        cc31k_IrqHandler(0);
    }
    return(mp_const_none);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(irq_callback_obj, irq_callback);

Fd_t cc31k_open(char *ifName, unsigned long flags)
{
    SPI_HANDLE.Init.Mode                = SPI_MODE_MASTER;
    SPI_HANDLE.Init.Direction           = SPI_DIRECTION_2LINES;
    SPI_HANDLE.Init.DataSize            = SPI_DATASIZE_8BIT; // should be correct
    SPI_HANDLE.Init.CLKPolarity         = SPI_POLARITY_LOW; // clock is low when idle
    SPI_HANDLE.Init.CLKPhase            = SPI_PHASE_1EDGE;  // data latched on second edge, which is rising edge for low-idle
    SPI_HANDLE.Init.NSS                 = SPI_NSS_SOFT; // software control
    SPI_HANDLE.Init.BaudRatePrescaler   = SPI_BAUDRATEPRESCALER_4; // clock freq = f_PCLK / this_prescale_value
    SPI_HANDLE.Init.FirstBit            = SPI_FIRSTBIT_MSB; // should be correct
    SPI_HANDLE.Init.TIMode              = SPI_TIMODE_DISABLED;
    SPI_HANDLE.Init.CRCCalculation      = SPI_CRCCALCULATION_DISABLED;
    SPI_HANDLE.Init.CRCPolynomial       = 7; // unused

    spi_init(&SPI_HANDLE, false);

    // configure wlan CS and EN pins
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Speed        = GPIO_SPEED_FAST;
    GPIO_InitStructure.Mode         = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull         = GPIO_NOPULL;
    GPIO_InitStructure.Alternate    = 0;

    GPIO_InitStructure.Pin = PIN_CS.pin_mask;
    HAL_GPIO_Init(PIN_CS.gpio, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = PIN_EN.pin_mask;
    HAL_GPIO_Init(PIN_EN.gpio, &GPIO_InitStructure);

    cc31k_cs(1); // de-assert CS
    cc31k_en(0); // disable wlan

    // delay 50ms
    HAL_Delay(50);

    // configure EXTI for PIN_IRQ and enable it
    extint_register((mp_obj_t)&PIN_IRQ, GPIO_MODE_IT_RISING, GPIO_NOPULL, (mp_obj_t)&irq_callback_obj, true);

    return(0);
}

int cc31k_close(Fd_t fd)
{
    // disable the interupt from WLAN
#if 0
    // this will fail, need to fix if to be used, the PIN_IRQ is not an int so follow
    // extint_register() internal implementation
    uint v_line = mp_obj_get_int((mp_obj_t)&PIN_IRQ);
    extint_disable(v_line);
#else
    // disable the interrupt by registering a null callback
    extint_register((mp_obj_t)&PIN_IRQ, GPIO_MODE_IT_RISING, GPIO_NOPULL, mp_const_none, true);
#endif
    return 0;
}

int cc31k_read(Fd_t fd, unsigned char *pBuff, int len)
{
    int rc;

    memset(pBuff, 0xFF, len);

    rc = cc31k_transceive(pBuff, len);
    if(rc < 0)
    {
        return(0);
    }
    return(len);
}

int cc31k_write(Fd_t fd, unsigned char *pBuff, int len)
{
    int rc;

    rc = cc31k_transceive(pBuff, len);
    if(rc < 0)
    {
        return(0);
    }
    return(len);
}

int cc31k_registerIrqHandler(irq_handler_t handler, void *pVal)
{
    cc31k_IrqHandler = handler;

    // The following could be implemented for performance issues
    if(handler)
    { // enable interrupt
    }
    else
    { // disable interrupt
    }

    return 0;
}

static int cc31k_transceive(unsigned char *data, uint16_t size)
{
    #define SPI_TIMEOUT_MS                     (20) // in ms
    HAL_StatusTypeDef hal_status;

    __disable_irq();
    cc31k_cs_assert();
    hal_status = HAL_SPI_TransmitReceive(&SPI_HANDLE, data, data, size, SPI_TIMEOUT_MS);
    cc31k_cs_deassert();
    __enable_irq();

    if(hal_status != HAL_OK)
    {
        LOG_ERR(0);
        return(-1);
    }
    return(size);
}




