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
#include "py/objstr.h"
#include "py/objlist.h"
#include "irq.h"
#include "pin.h"
#include "genhdr/pins.h"
#include "bufhelper.h"
#include "i2s.h"
#include MICROPY_HAL_H

/// \moduleref pyb
/// \class I2S - Inter-IC-Sound, a protocol to transfer isochronous audio data
///
/// I2S is a serial protocol for sending and receiving audio 
/// TODO:
/// - Detail synopsis of I2S, see SPI for example
/// - Usage example
/// - Methods list

// I2S Duplex operation requires a handle for both the base and extended
// instances, with one instance operating in transmit mode and the other in
// receive mode. When HAL_I2S_Init is called, it initializes the base instance
// in the selected mode (master or slave) and direction (transmit or receive).
// If duplex mode is selected then the extended instance is automatically
// initialized as a slave in the opposite direction as the base instance.


// Need to add second I2S Handle to pyb_i2s_obj_t to point to EXT instance if
// it is enabled - the EXT instance is always a slave to the base instance, so
// they need to be enabled together.
// Question is now whether base i2s instance is either a master or a slave, and
// which direction it is configured in - consider adjusting struct variables here
// to deal with this more sensibly, i.e. bool is_master
typedef struct _pyb_i2s_obj_t {
    mp_obj_base_t base;
    I2S_HandleTypeDef i2s;
    I2S_HandleTypeDef i2s_ext;
    // pins are Bit Clock, Word Select, TX Data, RX Data, and MCK Out
    const pin_obj_t *pins[5];
    mp_int_t i2s_id : 8;
    bool is_enabled : 1;
    bool is_master : 1;
    bool is_duplex : 1;
    bool base_is_tx : 1; // base instance is either tx or rx
} pyb_i2s_obj_t;

STATIC mp_obj_t pyb_i2s_deinit(mp_obj_t self_in);

// assumes init parameters are set up correctly
STATIC bool i2s_init(pyb_i2s_obj_t *i2s_obj) {
    // init the GPIO lines
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
    GPIO_InitStructure.Pull = GPIO_PULLUP; // see spi.c for dealing with CPOL

    // DMA will be needed sooner than later, see spi.c:
    /* DMA_HandleTypeDef *rx_dma, *tx_dma; */
    /* IRQn_Type rx_dma_irqn, tx_dma_irqn; */

    // TODO:
    // Use STM32F4DISC to make sure that I2S3 works on that board's codec
    // Decide whether to to enable I2S_CLKIN if it available (used by SDIO
    // on pyboard)

    // MCK for I2S2 is PC6, AF5
    // MCK for I2S3 is PC7, AF6

    // - If I2Sx_SD (SPIx) and I2SxEXT_SD pins are both given, init
    // full duplex mode. If only I2Sx_SD is given, init simplex mode
    // - SD pin is configured as either Rx or Tx in pyb_i2s_init_helper
    // based on postion in list; if ext_SD pin is present we set full-duplex
    // option, and HAL_I2S_Init automatically sets up the ext I2Sxext
    // instance as opposite direction (Rx or Tx) to I2Sx

    if (0) {
#if MICROPY_HW_ENABLE_I2S2
    } else if (i2s_obj->i2s_id == 2) {
	i2s_obj->i2s.Instance = SPI2;
	__SPI2_CLK_ENABLE();
	i2s_obj->pins[4] = (i2s_obj->i2s.Init.MCLKOutput ==
			    I2S_MCLKOUTPUT_ENABLE) ? &pin_C6 : MP_OBJ_NULL;
	for (uint i = 0; i < 5; i++) {
	    if (i2s_obj->pins[i] != MP_OBJ_NULL) {
		GPIO_InitStructure.Pin = i2s_obj->pins[i]->pin_mask;
		if (i2s_obj->pins[i] == &pin_B14 || i2s_obj->pins[i] == &pin_C2) {
		    GPIO_InitStructure.Alternate = GPIO_AF6_I2S2ext;
		} else {
		    GPIO_InitStructure.Alternate = GPIO_AF5_SPI2;
		}
		HAL_GPIO_Init(i2s_obj->pins[i]->gpio, &GPIO_InitStructure);
	    }    
	}
	
#endif
#if MICROPY_HW_ENABLE_I2S3
    } else if (i2s_obj->i2s_id == 3) {
	i2s_obj->i2s.Instance = SPI3;
	__SPI3_CLK_ENABLE();
	i2s_obj->pins[4] = (i2s_obj->i2s.Init.MCLKOutput ==
			    I2S_MCLKOUTPUT_ENABLE) ? &pin_C7 : MP_OBJ_NULL;
	for (uint i = 0; i < 5; i++) {
	    if (i2s_obj->pins[i] != MP_OBJ_NULL) {	    
		GPIO_InitStructure.Pin = i2s_obj->pins[i]->pin_mask;
		if (i2s_obj->pins[i] == &pin_B4) {
		    GPIO_InitStructure.Alternate = GPIO_AF7_I2S3ext;
		} else if (i2s_obj->pins[i] == &pin_C11) {
		    GPIO_InitStructure.Alternate = GPIO_AF5_I2S3ext;
		} else {
		    GPIO_InitStructure.Alternate = GPIO_AF6_SPI3;
		HAL_GPIO_Init(i2s_obj->pins[i]->gpio, &GPIO_InitStructure);
		}
	    }
	}   
#endif
    } else {
	// invalid i2s_id number
	return false;
    }

    // Configure and enable I2SPLL:
    // TODO: This is probably not the correct method to initialize and activate
    // the I2SPLL, but it was the quickest way that I could find to get master
    // mode up and running
    __HAL_RCC_PLLI2S_DISABLE();
    if (i2s_obj->i2s.Init.Mode == I2S_MODE_MASTER_TX ||
	i2s_obj->i2s.Init.Mode == I2S_MODE_MASTER_RX ) {
	// TODO - calculate values based on available parameters
	__HAL_RCC_PLLI2S_CONFIG(384, 5);
	__HAL_RCC_PLLI2S_ENABLE();
    }

    if (HAL_I2S_Init(&i2s_obj->i2s) != HAL_OK) {
        printf("OSError: HAL_I2S_Init failed\n");
        return false;
    } else {
	i2s_obj->is_enabled = true;
	// __HAL_I2S_ENABLE eventually gets called by HAL_I2S_Transmit_xx or
	// HAL_I2S_Receive_xx, but those aren't yet implemented so we call
	// it here to activate I2S hardware for testing
	__HAL_I2S_ENABLE(&i2s_obj->i2s);
	return true;
    }
}

// init0 and deinit are direct cribs from can.c:
void i2s_init0(void) {
    for (int i = 0; i < MP_ARRAY_SIZE(MP_STATE_PORT(pyb_i2s_obj_all)); i++) {
        MP_STATE_PORT(pyb_i2s_obj_all)[i] = NULL;
    }
}

// unregister all interrupt sources
void i2s_deinit(void) {
    for (int i = 0; i < MP_ARRAY_SIZE(MP_STATE_PORT(pyb_i2s_obj_all)); i++) {
        pyb_i2s_obj_t *i2s_obj = MP_STATE_PORT(pyb_i2s_obj_all)[i];
        if (i2s_obj != NULL) {
            pyb_i2s_deinit(i2s_obj);
        }
    }
}

/******************************************************************************/
/* Micro Python bindings for pyb.I2S                                          */

STATIC void pyb_i2s_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    pyb_i2s_obj_t *self = self_in;

    mp_printf(print, "I2S(%u on [", self->i2s_id);
    for (int i = 0; i < 4; i++) {
	if (self->pins[i] != MP_OBJ_NULL) {
	    mp_printf(print, "%q ", self->pins[i]->name);
	} else {
	    mp_print_str(print, "None ");
	}	
    }
    mp_print_str(print, "\b]");
    if (self->is_enabled) {
        if (self->i2s.Init.Mode == I2S_MODE_MASTER_TX ||
	    self->i2s.Init.Mode == I2S_MODE_MASTER_RX) {
            mp_print_str(print, ", I2S.MASTER, MCLK ");
	    if (self->i2s.Init.MCLKOutput == I2S_MCLKOUTPUT_ENABLE) {
		mp_printf(print, "on %q", self->pins[4]->name);
	    } else {
		mp_print_str(print, "off");
	    }
	    mp_printf(print, ", freq=%u", self->i2s.Init.AudioFreq);
        } else if (self->i2s.Init.Mode == I2S_MODE_SLAVE_TX ||
		   self->i2s.Init.Mode == I2S_MODE_SLAVE_RX) {
            mp_print_str(print, ", I2S.SLAVE");
        } else {
	    // Shouldn't get here if self->is_enabled=true
	}
        mp_printf(print, ", standard=%u, format=%u, polarity=%u",
	      self->i2s.Init.Standard, self->i2s.Init.DataFormat,
	      self->i2s.Init.CPOL);
    }
    mp_print_str(print, ")");
}

/// \method init(mode, standard=I2S.PHILIPS, dataformat=I2S._16B_EXTENDED,
///              polarity=I2S.LOW, audiofreq=48000,
///              clksrc=I2S.PLL, mclkout=I2S.DISABLE)
///
/// Initialise the SPI bus with the given parameters:
///
///   - `mode` must be either `I2S.MASTER` or `I2S.SLAVE`.
///   - `standard` can be `PHILIPS`, `MSB`, `LSB`, `PCM_SHORT`, or `PCM_LONG`.
///   - `dataformat` can be `_16B`, `_16B_EXTENDED`, `_24B`, or `_32B`.
///   - `polarity` can be `HIGH` or `LOW`.
///   - Options only relevant to master mode:
///   - `audiofreq` can be any common audio sampling frequency, default is 48000.
///   - `clksrc` can be `PLL` or `EXTERNAL`.
///   - `mclkout` can be `ENABLE` or `DISABLE`

STATIC mp_obj_t pyb_i2s_init_helper(pyb_i2s_obj_t *self,
				    mp_uint_t n_args,
				    const mp_obj_t *pos_args,
				    mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_mode,     MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
	{ MP_QSTR_standard, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = I2S_STANDARD_PHILIPS} },
	{ MP_QSTR_dataformat, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = I2S_DATAFORMAT_16B_EXTENDED} },
        { MP_QSTR_polarity, MP_ARG_KW_ONLY | MP_ARG_INT,  {.u_int = I2S_CPOL_LOW} },
	// Include option for setting I2SPLL parameters directly?
        { MP_QSTR_audiofreq, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = I2S_AUDIOFREQ_48K} },
        { MP_QSTR_clksrc, MP_ARG_KW_ONLY | MP_ARG_INT,  {.u_int = I2S_CLOCK_PLL} },
	{ MP_QSTR_mclkout, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = I2S_MCLKOUTPUT_DISABLE} },
    };
   
    // parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args,
		     MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // set the I2S configuration values
    memset(&self->i2s, 0, sizeof(self->i2s));
    I2S_InitTypeDef *init = &self->i2s.Init;

    // init->Mode can be MASTER_TX, MASTER_RX, SLAVE_TX, or SLAVE_RX;
    if (args[0].u_int == 0) {
	init->Mode = self->base_is_tx ? I2S_MODE_MASTER_TX : I2S_MODE_MASTER_RX;
    } else {
	init->Mode = self->base_is_tx ? I2S_MODE_SLAVE_TX : I2S_MODE_SLAVE_RX;
    }
	
    init->Standard = args[1].u_int;
    init->DataFormat = args[2].u_int;
    init->CPOL = args[3].u_int;
    init->AudioFreq = args[4].u_int;
    init->MCLKOutput = args[6].u_int;
    init->FullDuplexMode = self->is_duplex ? \
	I2S_FULLDUPLEXMODE_ENABLE : I2S_FULLDUPLEXMODE_DISABLE;

    // -------------Possible bug in HAL-------------------
    // According to the datasheet (RM0090, Sec 28.4.6 - I2S Slave Mode) in Slave
    // mode there is no need to enable I2SPLL to generate a clock for the I2S
    // interface; the CLK and WS pins receive clock signals from another master
    // to drive the data transfer.
    // HOWEVER, HAL_I2S_Init returns an error if the I2SPLL is not enabled and
    // ClockSource != I2S_CLOCK_EXTERNAL
    // This is incorrect behavior; the parameter I2S_CLOCK_EXTERNAL is intended
    // to set pin PC9 as I2S_CKIN, an external source for I2SPLL.
    // So we implement a work around that isn't really correct here:
    if (args[0].u_int) {
	// Slave mode
	init->ClockSource = I2S_CLOCK_EXTERNAL;
    } else {
	init->ClockSource = args[5].u_int;
    }
    
    // init the I2S bus
    if (!i2s_init(self)) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError,
    						"I2S port %d init failed", self->i2s_id));
    }

    return mp_const_none;
}

/// \classmethod \constructor(bus, ...)
///
/// Construct an I2S object with the given list of pins.
/// I2S requires a clock pin (CK), a word select pin (WS) and either
/// one or two data pins.
/// The pins can be passed as either a list or a tuple with the format
/// [Clock, Word Select, TX Data, RX Data]
/// If two data pins are provided, one must be an SD pin and the other must
/// be a corresponding SDext pin; the I2S port will be initialized in duplex
/// (TX + RX) mode.
/// If only one data pin is provided, it must be an SD (not SDext) pin. The
/// I2S port will be initialized in simplex mode, with the direction determined
/// by the position of the data pin in the list.
/// Examples:
/// pin_list = ['B10','B12','C3','C2'] - Duplex mode, C3 as Tx, C2 as Rx
/// pin_list = ['B10','B12',0,'C2'] - Simplex mode, C2 as RX
/// pin_list = ['Y6','Y5','Y7','Y8'] - Use board name strings
/// pin_list = ('Y6','Y5','Y7','Y8') - Tuple works as well as list
/// Mixed list of strings and objects:
/// pin_list = [pyb.Pin.cpu.B10,'B12','Y7','pyb.Pin.board.Y8']
///
/// Valid pins for I2S2 on the pyboard:
/// CK -    PB13 / Y6,  PB10 / Y9     (SPI2 SCK)
/// WS -    PB12 / Y5,  PB9  / Y4     (SPI2 NSS)
/// SD -    PB15 / Y8,  PC3  / X22    (SPI2 MOSI)
/// SDext - PB14 / Y7,  PC2  / X21    (SPI2 MISO)
/// MCK -   PC6 / Y1
///
/// The I2S3 port is disabled by default on the pyboard, as its pins would
/// conflict with the SD Card and other pyboard functions. It can be enabled
/// for the STM32F4DISCOVERY port, which has an I2S codec on I2S3.
///
/// CK -    PC10 / NA (SD_D2),  PB3  / X17 (USR SW)       (SPI3 SCK)
/// WS -    PA4  / X5,          PA15 / P3  (YELLOW LED)   (SPI3 NSS)
/// SD -    PC12 / NA (SD_CK),  PB5  / NA  (MMA_AVDD)     (SPI3 MOSI)
/// SDext - PC11 / NA (SD_D3),  PB4  / P2  (BLUE LED)     (SPI3 MISO)
/// MCK -   PC7 (Y2)
///
/// With no additional parameters, the I2S object is created but not
/// initialised (it has the settings from the last initialisation of
/// the bus, if any).  If extra arguments are given, the bus is initialised.
/// See `init` for parameters of initialisation.

STATIC mp_obj_t pyb_i2s_make_new(mp_obj_t type_in, mp_uint_t n_args,
				 mp_uint_t n_kw, const mp_obj_t *args) {

    // TODO- if no arguments are given and #ifdef PYBV10, set default
    // pin list to the SPI2 pins on the pyboard: Y6, Y5, Y7, Y8

#ifdef PYBV10
    // check arguments -  n_args == 0 is acceptable for the pyboard;
    // in that case I2S is initialized on the pyboard's SPI2 pins 
    mp_arg_check_num(n_args, n_kw, 0, MP_OBJ_FUN_ARGS_MAX, true);
#else
    mp_arg_check_num(n_args, n_kw, 1, MP_OBJ_FUN_ARGS_MAX, true);
#endif
    
    // get array of pin identifiers, could be name strings or pin objects
    mp_uint_t array_len = 0;
    mp_obj_t *pin_names;
    if (n_args == 0) {
#ifdef PYBV10	
	const pin_obj_t *dflt_pins[4] = {&pin_B13, &pin_B12, &pin_B15, &pin_B14};
	mp_obj_t dflt_pin_list = mp_obj_new_list(4, (mp_obj_t)dflt_pins);
	mp_obj_get_array(dflt_pin_list, &array_len, &pin_names);
#endif
    } else {
	mp_obj_get_array(args[0], &array_len, &pin_names);
    }
    if (array_len != 4) {
	nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError,
						"Pin list requires 4 items, \
                                                 %d given", array_len));
    }

    // Get array of pin objects; False / empty values are valid for pins[2] and
    // pins[3]; empty values get set to MP_OBJ_NULL
    const pin_obj_t *pins[4];
    for (int i = 0; i < 4; i++) {
        if (mp_obj_is_true(pin_names[i])) {
	    pins[i] = pin_find(pin_names[i]);
	} else {
	    pins[i] = MP_OBJ_NULL;
	}
	// also get this information in uPy: pyb.Pin.debug(True)
	if (0) { // DEBUG - print list of pin objects
	    mp_obj_print((mp_obj_t)pins[i], PRINT_STR);
	    printf("\n");
	}
    }

    // Logic to check if pins represents a valid I2S port configuration:
    // Each entry (pins[0]: CK, 1: WS, 2: SD-TX, 3: SD-RX) is checked to
    // whether it is valid for that function.
    // Of the two entries for data pins (2-TX and 3-RX) exactly one
    // of them be a valid base SD pin for the same I2S port as CK and WS;
    // the other can be a valid SDext pin to select duplex mode, or it
    // can be empty to selct simplex mode.
    // Is there a more elegant way to do this? Maybe by directly querying each
    // pin to see if there is an AF for I2S associated with it?
    mp_uint_t i2s_id = 0;
    mp_uint_t err_code = 0;
    bool is_duplex;
    bool base_is_tx;
    if (0) {
#if MICROPY_HW_ENABLE_I2S2
    } else if ((pins[0] == &pin_B10 || pins[0] == &pin_B13) &&
	       (pins[1] == &pin_B12 || pins[1] == &pin_B9)) {
	// pins[0:1] are valid CLK and WS pins of I2S2, set i2s_id
	i2s_id = 2;
	if (pins[2] == &pin_B15 || pins[2] == &pin_C3) {
	    // pins[2] is valid SD pin; config as TX
	    base_is_tx = true;
	    if (pins[3] == &pin_B14 || pins[3] == &pin_C2) {
		// pins[3] is valid SDext pin; duplex mode		
		is_duplex = true;
	    } else if (pins[3] == MP_OBJ_NULL) {
		is_duplex = false;
	    } else {
		err_code = 2; // pins[3] is invalid
	    }
	} else if (pins[3] == &pin_B15 || pins[3] == &pin_C3) {
	    // pins[3] is valid SD pin; config as RX
	    base_is_tx = false;
	    if (pins[2] == &pin_B14 || pins[2] == &pin_C2) {
		// pins[2] is valid SDext pin; duplex mode
		is_duplex = true;
	    } else if (pins[2] == MP_OBJ_NULL) {
		is_duplex = false;
	    } else {
		err_code = 3; // pins[2] is invalid
	    }
	} else {
	    err_code = 4; // No valid SD pin for I2S2
	}
#endif
#if MICROPY_HW_ENABLE_I2S3	
    } else if ((pins[0] == &pin_B3 || pins[0] == &pin_C10) &&
	       (pins[1] == &pin_A4 || pins[1] == &pin_A15)) {
	// pins[0:1] are valid CLK and WS pins of I2S3, set i2s_id
	i2s_id = 3;
	if (pins[2] == &pin_B5 || pins[2] == &pin_C12) {
	    // pins[2] is valid SD pin; config as TX
	    base_is_tx = true;
	    if (pins[3] == &pin_B4 || pins[3] == &pin_C11) {
		is_duplex = true;
	    } else if (pins[3] == MP_OBJ_NULL) {
		is_duplex = false;
	    } else {
		err_code = 5; // pins[3] is invalid
	    }
	} else if (pins[3] == &pin_B5 || pins[3] == &pin_C12) {
	    // pins[3] is valid SD pin; config as RX
	    base_is_tx = false;
	    if (pins[2] == &pin_B4 || pins[2] == &pin_C11) {
		is_duplex = true;
	    } else if (pins[2] == MP_OBJ_NULL) {
		is_duplex = false;
	    } else {
		err_code = 6; // pins[2] is invalid
	    }
	} else {
	    err_code = 7; // No valid SD pin for I2S3
	}
#endif    
    } else {
	err_code = 1; // pins[0:1] not valid clock and WS for available I2S ports 
    }
    if (err_code) {
	nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError,
						"Invalid pins for I2S, err: %d", err_code));
    }

    // get I2S object
    pyb_i2s_obj_t *i2s_obj;
    if (MP_STATE_PORT(pyb_i2s_obj_all)[i2s_id - 2] == NULL) {
	// create new I2S object
	i2s_obj = m_new_obj(pyb_i2s_obj_t);
	i2s_obj->base.type = &pyb_i2s_type;
	i2s_obj->i2s_id = i2s_id;
	i2s_obj->is_enabled = false;
	MP_STATE_PORT(pyb_i2s_obj_all)[i2s_id - 2] = i2s_obj;
    } else {
	i2s_obj = MP_STATE_PORT(pyb_i2s_obj_all)[i2s_id - 2];	    
    }

    i2s_obj->is_duplex = is_duplex;
    i2s_obj->base_is_tx = base_is_tx;
    for (int i = 0; i < 4; i++) {
	i2s_obj->pins[i] = pins[i];
    }

    if (n_args > 1 || n_kw > 0) {
        // start the peripheral
        mp_map_t kw_args;
        mp_map_init_fixed_table(&kw_args, n_kw, args + n_args);
        pyb_i2s_init_helper(i2s_obj, n_args - 1, args + 1, &kw_args);
    }

    return (mp_obj_t)i2s_obj;
}


//////////////////////////////////////

STATIC mp_obj_t pyb_i2s_init(mp_uint_t n_args, const mp_obj_t *args,
			     mp_map_t *kw_args) {
    return pyb_i2s_init_helper(args[0], n_args - 1, args + 1, kw_args);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_i2s_init_obj, 1, pyb_i2s_init);

/// \method deinit()
/// Turn off the I2S bus.
STATIC mp_obj_t pyb_i2s_deinit(mp_obj_t self_in) {
    pyb_i2s_obj_t *self = self_in;
    self->is_enabled = false;
    HAL_I2S_DeInit(&self->i2s);
    if (0) {
#if MICROPY_HW_ENABLE_I2S2
    } else if (self->i2s.Instance == SPI2) {
	__SPI2_FORCE_RESET();
        __SPI2_RELEASE_RESET();
        __SPI2_CLK_DISABLE();
#endif
#if MICROPY_HW_ENABLE_SPI3
    }
    else if (self->i2s.Instance == SPI3) {
	__SPI3_FORCE_RESET();
        __SPI3_RELEASE_RESET();
        __SPI3_CLK_DISABLE();
#endif
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pyb_i2s_deinit_obj, pyb_i2s_deinit);

// These are just placeholder methods until proper send and receive are
// implemented
STATIC mp_obj_t pyb_i2s_send(mp_uint_t n_args, const mp_obj_t *pos_args,
			     mp_map_t *kw_args) {
    // skeleton copied from spi.c
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_send,    MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_timeout, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 5000} },
    };

    // parse args
    // pyb_i2s_obj_t *self = pos_args[0];
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args,
		     MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    printf("I2S Send\n");
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_i2s_send_obj, 1, pyb_i2s_send);

STATIC mp_obj_t pyb_i2s_recv(mp_uint_t n_args, const mp_obj_t *pos_args,
			     mp_map_t *kw_args) {
    // TODO assumes transmission size is 8-bits wide

    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_recv,    MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_timeout, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 5000} },
    };

    // parse args
    // pyb_i2s_obj_t *self = pos_args[0];
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args,
		     MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    printf("I2S Receive\n");
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_i2s_recv_obj, 1, pyb_i2s_recv);

STATIC mp_obj_t pyb_i2s_send_recv(mp_uint_t n_args, const mp_obj_t *pos_args,
				  mp_map_t *kw_args) {
    // TODO assumes transmission size is 8-bits wide

    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_send,    MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_recv,    MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_timeout, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 5000} },
    };

    // parse args
    // pyb_i2s_obj_t *self = pos_args[0];
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args,
		     MP_ARRAY_SIZE(allowed_args), allowed_args, args);

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
    { MP_OBJ_NEW_QSTR(MP_QSTR_MASTER), MP_OBJ_NEW_SMALL_INT(0) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_SLAVE),  MP_OBJ_NEW_SMALL_INT(1) },
    
    // set format standard for I2S data
    { MP_OBJ_NEW_QSTR(MP_QSTR_PHILIPS),   MP_OBJ_NEW_SMALL_INT(I2S_STANDARD_PHILIPS) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_MSB),       MP_OBJ_NEW_SMALL_INT(I2S_STANDARD_MSB) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_LSB),       MP_OBJ_NEW_SMALL_INT(I2S_STANDARD_LSB) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_PCM_SHORT), MP_OBJ_NEW_SMALL_INT(I2S_STANDARD_PCM_SHORT) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_PCM_LONG),  MP_OBJ_NEW_SMALL_INT(I2S_STANDARD_PCM_LONG) },

    // set data and frame length
    { MP_OBJ_NEW_QSTR(MP_QSTR__16B),          MP_OBJ_NEW_SMALL_INT(I2S_DATAFORMAT_16B) },
    { MP_OBJ_NEW_QSTR(MP_QSTR__16B_EXTENDED), MP_OBJ_NEW_SMALL_INT(I2S_DATAFORMAT_16B_EXTENDED) },
    { MP_OBJ_NEW_QSTR(MP_QSTR__24B),          MP_OBJ_NEW_SMALL_INT(I2S_DATAFORMAT_24B) },
    { MP_OBJ_NEW_QSTR(MP_QSTR__32B),          MP_OBJ_NEW_SMALL_INT(I2S_DATAFORMAT_32B) },

    // set CLK and WS polarity, clock source, and Master Clock output (enable/disable)
    { MP_OBJ_NEW_QSTR(MP_QSTR_HIGH),     MP_OBJ_NEW_SMALL_INT(I2S_CPOL_HIGH) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_LOW),      MP_OBJ_NEW_SMALL_INT(I2S_CPOL_LOW) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_PLL),      MP_OBJ_NEW_SMALL_INT(I2S_CLOCK_PLL) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_EXTERNAL), MP_OBJ_NEW_SMALL_INT(I2S_CLOCK_EXTERNAL) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_ENABLE),   MP_OBJ_NEW_SMALL_INT(I2S_MCLKOUTPUT_ENABLE) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_DISABLE),  MP_OBJ_NEW_SMALL_INT(I2S_MCLKOUTPUT_DISABLE) },

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

Use default SPI pins as default I2S pins; allow option to pass alternate pin 
list. I2S_CKIN, I2S2_MCK and I2S3_MCK are either enabled or not, only relevant 
to Master mode, and have no alternate pin locations.

Passing a list of pin names or objects (alt_pins = ['PC3', 'PB9'] will be 
unambiguous; otherwise simply initialize I2S(2) or I2S(3) and then have the 
init function to set master, slave, frequency and other parameters

It would be rare to insist on using only an EXT instance for a simplex 
application, if it is even possible. More complicated to allow such a 
configuration, if it is possible. There should be an option to configure 
simplex without the EXT instance, though. Duplex requires both standard and 
EXT instances.

Pin options

I2S_CKIN : PC9 --> This corresponds to SDIO_D1 (SD Card) therefore unavailable 
on pyboard, or any STM32F4x5 board with an SD Card - both I2S_CKIN and SDIO_D1 
are available only on PC9 even for high pin-count parts! Seems a rather strange 
oversight.

I2S2:
CK -    PB13/Y6,  PB10/Y9       PI1      SCK
WS -    PB12/Y5,  PB9/Y4        PI0      NSS
SD -    PB15/Y8,  PC3/X22       PI3      MOSI
SDext - PB14/Y7,  PC2/X21       PI2      MISO
MCK -   PC6 (Y1)


I2S3: Note that SPI3 is disabled by default on the pyboard as its pins are 
used for other functions; the SPI3/I2S3 peripheral isn't accessible in any 
configuration - this restriction may not apply to other boards based on higher 
pin-count STM32F4xx chips.

**** NOTE THAT THE STM32F4DISC IS AN EXISTING PORT! It is the only higher pin 
count port in the main tree; OpenMV also uses a higher pin count STM32F4 but 
it isn't clear that I2S would ever have a place there

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
