/*
 * This file is part of the Micro Python project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013, 2014 Damien P. George
 * Copyright (c) 2015 Bryan Morrissey
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <stdio.h>
#include <string.h>

#include "py/nlr.h"
#include "py/runtime.h"
#include "irq.h"
#include "pin.h"
#include "genhdr/pins.h"
#include "bufhelper.h"
#include "i2s.h"
#include MICROPY_HAL_H

/// \moduleref pyb
/// \class I2S - Inter-IC-Sound, a protocol to transfer isochronous audio data
/// 
/// TODO:
/// - Detail synopsis of I2S, see SPI for example
/// - Usage example
/// - Methods list

#if MICROPY_HW_ENABLE_I2S2
I2S_HandleTypeDef I2SHandle2 = {.Instance = NULL};
#endif
#if MICROPY_HW_ENABLE_I2S3
I2S_HandleTypeDef I2SHandle3 = {.Instance = NULL};
#endif


// TODO - DMA defs here, see spi.c

void i2s_init0(void) {
    // reset the I2S handles
#if MICROPY_HW_ENABLE_I2S2
    memset(&I2SHandle2, 0, sizeof(I2S_HandleTypeDef));
    I2SHandle2.Instance = SPI2;
#endif
#if MICROPY_HW_ENABLE_I2S3
    memset(&I2SHandle3, 0, sizeof(I2S_HandleTypeDef));
    I2SHandle3.Instance = SPI3;
#endif
}

void i2s_init(I2S_HandleTypeDef *i2s) {
    // init the GPIO lines
    // start with a hard-wired init using Myriad2 I2S pins:
    //       PB10 (af5), PB12 (af5), PC2 (af6), PC3 (af5)
    // TODO: accept list of pins to use as I2S port,
    // parse list, check that they provide a valid I2S port without
    // conflicting with if

    /* GPIO_InitTypeDef GPIO_InitStructure; */
    /* GPIO_InitStructure.Mode = GPIO_MODE_AF_PP; */
    /* GPIO_InitStructure.Speed = GPIO_SPEED_FAST; */
    /* GPIO_InitStructure.Pull = GPIO_PULLUP; */

    /* const pin_obj_t *pins[4]; */

    //////////////////
    // start with just the config for I2S2/SPI2, with Myriad pin config
    // note that I2S3 is mostly going to target Discovery board;
    // should make sure that it works on the discovery's codec
    // will be important to accept a list of pins eventually, and the
    // option for the MCLK
    // Need master/slave options as well

    // Accept a list of pins -
    // - If two data pins are given, we know we need full duplex
    // - if only one is given, we determine whether it is send or recv from
    //   its position in the list
    // - how do we make sure that a certain pin is configured as send or receive
    //   in full-duplex mode? Answer - a pin's alt function is fixed, either I2S
    //   or I2Sext; we set the direction as tx or rx in the I2S/I2Sext config
    //
    //   need to create an I2Sext handle!

    I2S_HandleTypeDef *i2s
	
    GPIO_InitTypeDef GPIO_InitStructure;
    if(i2s->Instance==SPI2) {
	__SPI2_CLK_ENABLE();

	/**I2S2 GPIO Configuration    
	   PC2     ------> I2S2_ext_SD
	   PC3     ------> I2S2_SD
	   PB10     ------> I2S2_CK
	   PB12     ------> I2S2_WS 
	*/
	GPIO_InitStructure.Pin = GPIO_PIN_2;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
	GPIO_InitStructure.Alternate = GPIO_AF6_I2S2ext;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = GPIO_PIN_3;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
	GPIO_InitStructure.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = GPIO_PIN_10|GPIO_PIN_12;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
	GPIO_InitStructure.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

    	i2s->Init.Mode = I2S_MODE_SLAVE_TX;
	i2s->Init.Standard = I2S_STANDARD_PHILLIPS;
	i2s->Init.DataFormat = I2S_DATAFORMAT_16B_EXTENDED;
	i2s->Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
	i2s->Init.AudioFreq = I2S_AUDIOFREQ_48K;
	i2s->Init.CPOL = I2S_CPOL_LOW;
	i2s->Init.ClockSource = I2S_CLOCK_EXTERNAL;
	i2s->Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
    }  

    // init the I2S device
    if (HAL_I2S_Init(i2s) != HAL_OK) {
        // init error
        // TODO should raise an exception, but this function is not necessarily going to be
        // called via Python, so may not be properly wrapped in an NLR handler
        printf("OSError: HAL_I2S_Init failed\n");
        return;
    }

}

void i2s_deinit(I2S_HandleTypeDef *i2s) {
    HAL_I2S_DeInit(i2s);
    if (0) {    // why is this here?
#if MICROPY_HW_ENABLE_I2S2
    } else if (i2s->Instance == SPI2) {
        __SPI2_FORCE_RESET();
        __SPI2_RELEASE_RESET();
        __SPI2_CLK_DISABLE();
#endif
#if MICROPY_HW_ENABLE_SPI3
    } else if (i2s->Instance == SPI3) {
        __SPI3_FORCE_RESET();
        __SPI3_RELEASE_RESET();
        __SPI3_CLK_DISABLE();
#endif
    }
}



/******************************************************************************/
/* Micro Python bindings                                                      */

typedef struct _pyb_i2s_obj_t {
    mp_obj_base_t base;
    I2S_HandleTypeDef *i2s;
} pyb_i2s_obj_t;

STATIC const pyb_i2s_obj_t pyb_i2s_obj[] = {
#if MICROPY_HW_ENABLE_I2S2
    {{&pyb_i2s_type}, &I2SHandle2},
#else
    {{&pyb_i2s_type}, NULL},
#endif
#if MICROPY_HW_ENABLE_I2S3
    {{&pyb_i2s_type}, &I2SHandle3},
#else
    {{&pyb_i2s_type}, NULL},
#endif
};
#define PYB_NUM_I2S MP_ARRAY_SIZE(pyb_i2s_obj)

I2S_HandleTypeDef *i2s_get_handle(mp_obj_t o) {
    if (!MP_OBJ_IS_TYPE(o, &pyb_i2s_type)) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, "expecting an I2S object"));
    }
    pyb_i2s_obj_t *self = o;
    return self->i2s;
}


////**** Focus on this as a debugging resource - try to implement printing of
//// parameters from stm32f4xx_hal_i2s.h as a reference
//////////////////////////////////////////////////////////////////////////
STATIC void pyb_i2s_print(void (*print)(void *env, const char *fmt, ...),
			  void *env, mp_obj_t self_in, mp_print_kind_t kind) {
    pyb_i2s_obj_t *self = self_in;

    uint i2s_num;
    if (self->i2s->Instance == SPI2) { i2s_num = 2; }
    else { i2s_num = 3; }
    // keep it minimal for now, see spi.c
    // print(env, "I2S(%u)", i2s_num);

    // from spi.c: 
    if (self->i2s->State == HAL_I2S_STATE_RESET) {
        print(env, "I2S(%u)", i2s_num);
    } else {
        if (self->i2s->Init.Mode == I2S_MODE_MASTER_TX) {
            // compute baudrate
            uint i2s_clock;
	    i2s_clock = HAL_RCC_GetPCLK1Freq();
            // uint log_prescaler = (self->i2s->Init.BaudRatePrescaler >> 3) + 1;
            uint log_prescaler = 4; // just need a plausible number for testing
            uint baudrate = i2s_clock >> log_prescaler;
            print(env, "I2S(%u, SPI.MASTER, baudrate=%u, prescaler=%u", i2s_num, baudrate, 1 << log_prescaler);
        } else {
            print(env, "I2S(%u, SPI.SLAVE", i2s_num);
        }
        print(env, ", mode=%u, standard=%u, bits=%u", self->i2s->Init.Mode, self->i2s->Init.Standard, self->i2s->Init.DataFormat);
        print(env, ")");
    }

}

/////////////////////////////////////////

/// \method init(mode, baudrate=328125, *, polarity=1, phase=0, bits=8, firstbit=SPI.MSB, ti=False, crc=None)
///
/// Initialise the SPI bus with the given parameters:
///
///   - `mode` must be either `SPI.MASTER` or `SPI.SLAVE`.
///   - `baudrate` is the SCK clock rate (only sensible for a master).
STATIC mp_obj_t pyb_i2s_init_helper(const pyb_i2s_obj_t *self, mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_mode,     MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_baudrate, MP_ARG_INT, {.u_int = 328125} },
        { MP_QSTR_prescaler, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0xffffffff} },
        { MP_QSTR_polarity, MP_ARG_KW_ONLY | MP_ARG_INT,  {.u_int = 1} },
        { MP_QSTR_phase,    MP_ARG_KW_ONLY | MP_ARG_INT,  {.u_int = 0} },
        { MP_QSTR_dir,      MP_ARG_KW_ONLY | MP_ARG_INT,  {.u_int = SPI_DIRECTION_2LINES} },
        { MP_QSTR_bits,     MP_ARG_KW_ONLY | MP_ARG_INT,  {.u_int = 8} },
        { MP_QSTR_nss,      MP_ARG_KW_ONLY | MP_ARG_INT,  {.u_int = SPI_NSS_SOFT} },
        { MP_QSTR_firstbit, MP_ARG_KW_ONLY | MP_ARG_INT,  {.u_int = SPI_FIRSTBIT_MSB} },
        { MP_QSTR_ti,       MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false} },
        { MP_QSTR_crc,      MP_ARG_KW_ONLY | MP_ARG_OBJ,  {.u_obj = mp_const_none} },
    };

    // parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // set the I2S configuration values
    I2S_InitTypeDef *init = &self->i2s->Init;
    init->Mode = args[0].u_int;

    // init the SPI bus
    i2s_init(self->i2s /*, init->NSS != SPI_NSS_SOFT */);

    return mp_const_none;
}

/// \classmethod \constructor(bus, ...)
///
/// Construct an SPI object on the given bus.  `bus` can be 1 or 2.
/// With no additional parameters, the SPI object is created but not
/// initialised (it has the settings from the last initialisation of
/// the bus, if any).  If extra arguments are given, the bus is initialised.
/// See `init` for parameters of initialisation.
///
/// The physical pins of the SPI busses are:
///
///   - `SPI(1)` is on the X position: `(NSS, SCK, MISO, MOSI) = (X5, X6, X7, X8) = (PA4, PA5, PA6, PA7)`
///   - `SPI(2)` is on the Y position: `(NSS, SCK, MISO, MOSI) = (Y5, Y6, Y7, Y8) = (PB12, PB13, PB14, PB15)`
///
/// At the moment, the NSS pin is not used by the SPI driver and is free
/// for other use.
STATIC mp_obj_t pyb_i2s_make_new(mp_obj_t type_in, mp_uint_t n_args, mp_uint_t n_kw, const mp_obj_t *args) {
    // check arguments
    mp_arg_check_num(n_args, n_kw, 1, MP_OBJ_FUN_ARGS_MAX, true);

    // get I2S number
    mp_int_t i2s_id = mp_obj_get_int(args[0]) - 1;

    // check I2S number
    if (!(0 <= i2s_id && i2s_id < PYB_NUM_I2S && pyb_i2s_obj[i2s_id].i2s != NULL)) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "I2S bus %d does not exist", i2s_id + 1));
    }

    // get I2S object
    const pyb_i2s_obj_t *i2s_obj = &pyb_i2s_obj[i2s_id];

    if (n_args > 1 || n_kw > 0) {
        // start the peripheral
        mp_map_t kw_args;
        mp_map_init_fixed_table(&kw_args, n_kw, args + n_args);
        pyb_i2s_init_helper(i2s_obj, n_args - 1, args + 1, &kw_args);
    }

    return (mp_obj_t)i2s_obj;
}


//////////////////////////////////////

STATIC mp_obj_t pyb_i2s_init(mp_uint_t n_args, const mp_obj_t *args, mp_map_t *kw_args) {
    return pyb_i2s_init_helper(args[0], n_args - 1, args + 1, kw_args);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_i2s_init_obj, 1, pyb_i2s_init);

/// \method deinit()
/// Turn off the I2S bus.
STATIC mp_obj_t pyb_i2s_deinit(mp_obj_t self_in) {
    pyb_i2s_obj_t *self = self_in;
    i2s_deinit(self->i2s);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pyb_i2s_deinit_obj, pyb_i2s_deinit);



//// These are just skeletal methods to test that I can make uPy method bindings
STATIC mp_obj_t pyb_i2s_send(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    // skeleton copied from spi.c
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_send,    MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_timeout, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 5000} },
    };

    // parse args
    // pyb_i2s_obj_t *self = pos_args[0];
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    printf("I2S Send\n");
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_i2s_send_obj, 1, pyb_i2s_send);

STATIC mp_obj_t pyb_i2s_recv(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    // TODO assumes transmission size is 8-bits wide

    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_recv,    MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_timeout, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 5000} },
    };

    // parse args
    // pyb_i2s_obj_t *self = pos_args[0];
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    printf("I2S Receive\n");
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_i2s_recv_obj, 1, pyb_i2s_recv);

STATIC mp_obj_t pyb_i2s_send_recv(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    // TODO assumes transmission size is 8-bits wide

    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_send,    MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_recv,    MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_timeout, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 5000} },
    };

    // parse args
    // pyb_i2s_obj_t *self = pos_args[0];
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    printf("I2S Send-Receive\n");
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_i2s_send_recv_obj, 1, pyb_i2s_send_recv);
/////////////

STATIC const mp_map_elem_t pyb_i2s_locals_dict_table[] = {
    // instance methods
    { MP_OBJ_NEW_QSTR(MP_QSTR_init), (mp_obj_t)&pyb_i2s_init_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_deinit), (mp_obj_t)&pyb_i2s_deinit_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_send), (mp_obj_t)&pyb_i2s_send_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_recv), (mp_obj_t)&pyb_i2s_recv_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_send_recv), (mp_obj_t)&pyb_i2s_send_recv_obj },

    // class constants
    /// \constant MASTER - for initialising the bus to master mode
    /// \constant SLAVE - for initialising the bus to slave mode
    /// \constant MSB - set the first bit to MSB
    /// \constant LSB - set the first bit to LSB

    { MP_OBJ_NEW_QSTR(MP_QSTR_MASTER), MP_OBJ_NEW_SMALL_INT(SPI_MODE_MASTER) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_SLAVE),  MP_OBJ_NEW_SMALL_INT(SPI_MODE_SLAVE) },
    /*
    { MP_OBJ_NEW_QSTR(MP_QSTR_MSB),    MP_OBJ_NEW_SMALL_INT(SPI_FIRSTBIT_MSB) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_LSB),    MP_OBJ_NEW_SMALL_INT(SPI_FIRSTBIT_LSB) },
    */

    /* TODO
    { MP_OBJ_NEW_QSTR(MP_QSTR_DIRECTION_2LINES             ((uint32_t)0x00000000)
    { MP_OBJ_NEW_QSTR(MP_QSTR_DIRECTION_2LINES_RXONLY      SPI_CR1_RXONLY
    { MP_OBJ_NEW_QSTR(MP_QSTR_DIRECTION_1LINE              SPI_CR1_BIDIMODE
    { MP_OBJ_NEW_QSTR(MP_QSTR_NSS_SOFT                    SPI_CR1_SSM
    { MP_OBJ_NEW_QSTR(MP_QSTR_NSS_HARD_INPUT              ((uint32_t)0x00000000)
    { MP_OBJ_NEW_QSTR(MP_QSTR_NSS_HARD_OUTPUT             ((uint32_t)0x00040000)
    */
};

STATIC MP_DEFINE_CONST_DICT(pyb_i2s_locals_dict, pyb_i2s_locals_dict_table);

const mp_obj_type_t pyb_i2s_type = {
    { &mp_type_type },
    .name = MP_QSTR_I2S,
    .print = pyb_i2s_print,
    .make_new = pyb_i2s_make_new,
    .locals_dict = (mp_obj_t)&pyb_i2s_locals_dict,
};






/*
Need to figure out an API for setting the I2S parameters.


Use default SPI pins as default I2S pins; allow option to pass
alternate pin list.
I2S_CKIN, I2S2_MCK and I2S3_MCK are either enabled or not, only relevant to
Master mode, and have no alternate pin locations

Passing a list of pin names or objects (alt_pins = ['PC3', 'PB9'] will be unambiguous;
otherwise simply initialize I2S(2) or I2S(3) and then have the init function to set master,
slave, frequency and other parameters

It would be rare to insist on using only an EXT instance for a simplex application, if it is
even possible. More complicated to allow such a configuration, if it is possible. There should be
an option to configure simplex without the EXT instance, though.
Duplex requires both standard and EXT instances.

Pin options

I2S_CKIN : PC9 --> This corresponds to SDIO_D1 (SD Card) therefore unavailable on pyboard

I2S2:
CK -    PB13/Y6,  PB10/Y9      PI1      SCK
WS -    PB12/Y5,  PB9/Y4       PI0      NSS
SD -    PB15/Y8,  PC3/X22       PI3      MOSI
SDext - PB14/Y7,  PC2/X21       PI2      MISO
MCK -   PC6 (Y1)


I2S3: Note that SPI3 is disabled by default on the pyboard as its pins are used for other functions;
the SPI3/I2S3 peripheral isn't accessible in any configuration - this restriction may not apply to other
boards based on higher pin-count STM32F4xx chips
**** NOTE THAT THE STM32F4DISC IS AN EXISTING PORT! It is the only higher pin count port in the main tree;
OpenMV also uses a higher pin count STM32F4 but it isn't clear that I2S would ever have a place there

CK -    PC10 / NA (SDIO),  PB3  / X17 (USR)              SCK
WS -    PA4  / X5,         PA15 / P3  (LED)              NSS
SD -    PC12 / NA (SDIO),  PB5  / NA  (MMA_AVDD)         MOSI
SDext - PC11 / NA (SDIO),  PB4  / P2  (LED)              MISO
MCK -   PC7 (Y2)



*** METHOD NAMING ***

SPI, I2C, and CAN use [send, recv]
I2C has mem_read and mem_write
UART uses [read, write]
DAC, LCD use write
ADC uses read
USB_VCP uses both!


SPI(1) is on the X position: (NSS, SCK, MISO, MOSI) = (X5, X6, X7, X8) = (PA4, PA5, PA6, PA7)
SPI(2) is on the Y position: (NSS, SCK, MISO, MOSI) = (Y5, Y6, Y7, Y8) = (PB12, PB13, PB14, PB15)


When creating an SPI object:
spi = pyb.SPI(bus, ...)
Construct an SPI object on the given bus. bus can be 1 or 2. With no additional parameters,
the SPI object is created but not initialised (it has the settings from the last initialisation
of the bus, if any). If extra arguments are given, the bus is initialised.


spi.init(mode, baudrate=328125, *, prescaler, polarity=1, phase=0, bits=8, firstbit=SPI.MSB, ti=False, crc=None)
Initialise the SPI bus with the given parameters:

mode must be either SPI.MASTER or SPI.SLAVE.
baudrate is the SCK clock rate (only sensible for a master).
prescaler is the prescaler to use to derive SCK from the APB bus frequency; use of prescaler overrides baudrate.
polarity can be 0 or 1, and is the level the idle clock line sits at.
phase can be 0 or 1 to sample data on the first or second clock edge respectively.
firstbit can be SPI.MSB or SPI.LSB.
crc can be None for no CRC, or a polynomial specifier.



 */



/*
While the I2S peripheral is technically a specific configuration of the SPI hardware,
the usage calls for a distinct API and therefore a separate class definition.
In addition, I2S has relatively little use in typical robotics or control applications; it
is an application-specific data link that is only used for transferring isochronous audio data
between audio ADC's, DAC's, codecs and processing hardware. It operates continuously at a specified
bit rate / sample rate without buffering or error checking. There is no concept of re-tranmission,
the idea is simply to read or write a stream of audio data samples at a fixed rate.


 */


/*
step 1: create a simple module that accepts pin arguments and returns an I2S object that can be written to and read from
start with slave, then move to master and configuring PLL params
but even before Master, go for DMA!
*/
