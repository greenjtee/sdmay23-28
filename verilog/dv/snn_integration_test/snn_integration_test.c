// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// SPDX-License-Identifier: Apache-2.0

// This include is relative to $CARAVEL_PATH (see Makefile)
#include <defs.h>
#include <stub.c>
#include <irq_vex.h>

#define reg_snn_control (*(volatile uint32_t *)0x30004000)

#define image_write(data, index) (*(volatile uint32_t *)(0x30000000 + (index << 2))) = data
#define weight_write(data, index) (*(volatile uint32_t *)(0x30001000 + (index << 2))) = data

#define WIDTH 9
#define HEIGHT 9
#define OUTPUTS 10
#define NUM_PIXELS HEIGHT*WIDTH
#define NUM_WEIGHTS NUM_PIXELS*OUTPUTS

extern uint16_t flag;

// this is supposed to be a 7
uint8_t image[NUM_PIXELS] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x08, 0x36, 0x34, 0x2b, 0x2b, 0x24, 0x04, 0x00, 
    0x00, 0x02, 0x10, 0x1b, 0x24, 0x33, 0x4c, 0x07, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x01, 0x36, 0x2a, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x13, 0x41, 0x08, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x06, 0x41, 0x22, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x2a, 0x45, 0x04, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x01, 0x34, 0x24, 0x00, 0x00, 0x00, 0x00,
};

uint8_t weights[NUM_WEIGHTS] = {
	0x00, 0xfd, 0x05, 0xfd, 0xea, 0x0f, 0xfb, 0x0a, 0x07, 0xf6, 
	0x03, 0x05, 0x0f, 0xf7, 0xfc, 0x03, 0x06, 0xff, 0x13, 0x00, 
	0xf7, 0xfa, 0x01, 0xeb, 0xe5, 0x14, 0x14, 0x07, 0x17, 0x08, 
	0xfb, 0x05, 0x04, 0x04, 0xd4, 0x0c, 0x2a, 0x1a, 0x06, 0x08, 
	0xf5, 0x07, 0xf4, 0x07, 0xd8, 0x05, 0x38, 0x15, 0xf7, 0x0a, 
	0xd8, 0xdc, 0xe7, 0x1e, 0xd9, 0xfe, 0x38, 0x25, 0xf5, 0xf9, 
	0xdc, 0xd6, 0xe6, 0x30, 0xe7, 0xe8, 0x41, 0x0a, 0xfd, 0x0b, 
	0xee, 0xe8, 0xe1, 0x28, 0xeb, 0xde, 0x2e, 0x10, 0xf8, 0x12, 
	0xfd, 0xe3, 0x00, 0x0f, 0xf1, 0x05, 0x0d, 0xfa, 0x02, 0x11, 
	0xfa, 0x00, 0xf3, 0x10, 0xff, 0xfb, 0xfd, 0xfb, 0x03, 0x0c, 
	0xf6, 0x01, 0xf9, 0x19, 0x03, 0xeb, 0xfd, 0x05, 0xfe, 0x0d, 
	0xfd, 0xef, 0x0f, 0x11, 0x08, 0xfb, 0x06, 0xf7, 0xf2, 0x0c, 
	0xf6, 0xfa, 0x09, 0x17, 0xf8, 0x00, 0xf5, 0xfd, 0xfd, 0x09, 
	0x00, 0x00, 0x12, 0x0e, 0x02, 0xfb, 0x02, 0x00, 0x06, 0xfe, 
	0x02, 0x06, 0x05, 0x03, 0x03, 0xf9, 0x07, 0x00, 0x01, 0xff, 
	0xfb, 0x05, 0xff, 0xfa, 0x03, 0x04, 0x0d, 0x0a, 0x04, 0xfe, 
	0xf2, 0x04, 0xfa, 0xf5, 0x14, 0x24, 0x05, 0x09, 0xf4, 0x01, 
	0xf0, 0xf9, 0xf9, 0x09, 0x0b, 0x2d, 0x09, 0x0a, 0xf1, 0x0c, 
	0xf5, 0x15, 0xf9, 0xfa, 0x06, 0xfd, 0x0a, 0x1d, 0xf9, 0xf4, 
	0x0e, 0xfd, 0x09, 0x0b, 0x04, 0xfb, 0x00, 0x05, 0xfa, 0xef, 
	0xfc, 0xf9, 0xfa, 0x0e, 0x08, 0xfe, 0xfe, 0x11, 0x05, 0xed, 
	0x02, 0xf5, 0x03, 0x02, 0xfe, 0x04, 0x03, 0x08, 0x02, 0xf8, 
	0x06, 0x04, 0xfe, 0x00, 0xe7, 0xfd, 0xfa, 0x02, 0xfe, 0x1b, 
	0x0d, 0xff, 0x07, 0x00, 0xf5, 0x00, 0xf5, 0xfb, 0xff, 0x0e, 
	0x02, 0x02, 0xfa, 0x06, 0x09, 0x02, 0xf4, 0x00, 0x00, 0xfe, 
	0xf8, 0xf5, 0x07, 0xff, 0x02, 0x18, 0xf5, 0xfe, 0x08, 0xf3, 
	0xcf, 0xec, 0xfc, 0x11, 0x04, 0x4a, 0x0f, 0x0d, 0xff, 0xf6, 
	0xfb, 0xff, 0x0f, 0xfb, 0xf9, 0x05, 0x03, 0x14, 0xea, 0xe5, 
	0xf5, 0xfb, 0x01, 0x03, 0xfe, 0x00, 0xfe, 0x1b, 0x0b, 0xed, 
	0xff, 0x06, 0xf9, 0xe6, 0xf5, 0x0b, 0x05, 0x04, 0x04, 0x06, 
	0x02, 0xed, 0xf5, 0xee, 0x08, 0x07, 0xff, 0x04, 0x14, 0x07, 
	0x01, 0x14, 0xf6, 0x08, 0xfc, 0x04, 0x06, 0x0e, 0xfa, 0xf5, 
	0x00, 0x02, 0x00, 0x0f, 0x0b, 0xf3, 0xf0, 0x13, 0xf9, 0x06, 
	0x08, 0xf7, 0x08, 0xff, 0x02, 0xf1, 0xf3, 0x05, 0x0f, 0x03, 
	0x11, 0x05, 0xf9, 0xfe, 0xf4, 0xeb, 0xfc, 0x00, 0x0f, 0x06, 
	0xf4, 0xe7, 0x09, 0x0f, 0xfc, 0x40, 0x02, 0xfe, 0x11, 0x0d, 
	0xe6, 0x03, 0x17, 0x05, 0xe4, 0xf8, 0xf7, 0x2a, 0x06, 0xfa, 
	0x02, 0xf9, 0xf1, 0x01, 0x0f, 0x05, 0x04, 0xfa, 0x03, 0x05, 
	0x09, 0x12, 0xf8, 0x06, 0x10, 0x00, 0xfd, 0xf5, 0xec, 0x11, 
	0x07, 0xe1, 0xfa, 0x04, 0x0a, 0x12, 0x0b, 0xf3, 0xfe, 0x04, 
	0xf0, 0x1b, 0xf6, 0x0d, 0xff, 0xfe, 0x00, 0xe4, 0x16, 0x07, 
	0xec, 0xf3, 0xff, 0x06, 0x0a, 0xfb, 0x07, 0x06, 0x00, 0x0c, 
	0xfc, 0x05, 0xf9, 0x02, 0x03, 0xf8, 0x07, 0xfe, 0xfd, 0x09, 
	0x15, 0x03, 0xe9, 0xf8, 0x0e, 0xe6, 0x15, 0x01, 0xfd, 0x01, 
	0x05, 0xe7, 0x18, 0x22, 0x00, 0x00, 0x0d, 0xf9, 0x08, 0xf6, 
	0xdc, 0xfd, 0x0b, 0x1d, 0x05, 0x04, 0xfe, 0x05, 0x16, 0xf1, 
	0x0d, 0xf7, 0x0a, 0x03, 0x03, 0xfb, 0xf8, 0xfe, 0xff, 0x00, 
	0x0e, 0x04, 0x09, 0xf8, 0x0f, 0xf4, 0xfa, 0xff, 0xf9, 0xf8, 
	0xfd, 0xf3, 0x01, 0xf3, 0x00, 0xf3, 0x13, 0xf7, 0x11, 0x03, 
	0xf2, 0x17, 0x14, 0xf3, 0x10, 0xff, 0xfe, 0xfd, 0x0c, 0xff, 
	0xf7, 0xf0, 0x0b, 0x09, 0x10, 0xff, 0xf9, 0x08, 0xfc, 0x06, 
	0x06, 0x05, 0xfd, 0x0f, 0xff, 0x06, 0x0a, 0x03, 0xf4, 0xef, 
	0xfe, 0x02, 0xfd, 0x07, 0xff, 0x05, 0x06, 0x00, 0xfd, 0xfe, 
	0xfd, 0xe7, 0x2a, 0xfc, 0xf3, 0x09, 0xfb, 0xf8, 0x07, 0xfd, 
	0xe2, 0xeb, 0xf2, 0x24, 0xf6, 0xf2, 0x00, 0xf6, 0xfd, 0xfc, 
	0xf9, 0xfc, 0x16, 0x0f, 0xf6, 0x06, 0xfc, 0x00, 0xfb, 0xfc, 
	0x04, 0xf1, 0x10, 0xfc, 0xf2, 0x12, 0xfa, 0xfc, 0x04, 0xff, 
	0x06, 0x05, 0x09, 0xf3, 0xeb, 0x00, 0x09, 0x00, 0x0a, 0xf0, 
	0x09, 0x0c, 0xff, 0xf5, 0x00, 0x02, 0x15, 0x0a, 0xf5, 0xfd, 
	0x01, 0xf4, 0x07, 0x00, 0xfc, 0x02, 0x0f, 0xf6, 0xfa, 0xfa, 
	0xfd, 0xfe, 0x0a, 0x03, 0xfd, 0x03, 0x01, 0xfa, 0xfa, 0x01, 
	0xf8, 0xf4, 0x26, 0xee, 0xfd, 0x03, 0xee, 0x00, 0xf8, 0x01, 
	0x0e, 0x05, 0x38, 0xe3, 0x02, 0xff, 0xdf, 0x08, 0xff, 0x00, 
	0xeb, 0xeb, 0x02, 0x23, 0xfe, 0xff, 0x1c, 0xe2, 0xf4, 0xec, 
	0x03, 0x01, 0x05, 0x18, 0xfc, 0xf1, 0xff, 0xfd, 0xf7, 0x00, 
	0x00, 0x05, 0x00, 0x08, 0xfb, 0xfd, 0xfc, 0xf5, 0x04, 0xfa, 
	0x09, 0xf9, 0xf7, 0x0b, 0x04, 0x0a, 0x00, 0x00, 0x07, 0xfd, 
	0x01, 0x09, 0xf7, 0x06, 0x02, 0xfd, 0xf3, 0xfa, 0x17, 0xf3, 
	0x07, 0x07, 0xfb, 0x04, 0x09, 0x07, 0xf9, 0x00, 0x0a, 0xf8, 
	0xf7, 0xfe, 0x04, 0xf6, 0x08, 0x06, 0xfe, 0xf2, 0x09, 0x06, 
	0x05, 0xfc, 0x1b, 0xeb, 0xf4, 0x0e, 0xfb, 0xf4, 0xf7, 0x02, 
	0xff, 0x0a, 0x18, 0xfa, 0x11, 0x05, 0xf1, 0xf8, 0xf1, 0x06, 
	0xec, 0xe6, 0xf2, 0x0c, 0x00, 0xff, 0x15, 0x0b, 0xfa, 0xf6, 
	0x00, 0xee, 0xe8, 0x14, 0x08, 0xfa, 0x2b, 0x28, 0xec, 0x07, 
	0xe7, 0xf5, 0x00, 0x0e, 0xf7, 0x05, 0x1a, 0x2e, 0xef, 0xfc, 
	0xeb, 0xeb, 0x13, 0xfc, 0xe2, 0x00, 0x12, 0x2a, 0xf2, 0x0a, 
	0xf6, 0xf5, 0x15, 0xf8, 0xe0, 0x05, 0x15, 0x25, 0xf8, 0x13, 
	0xfa, 0x04, 0x0f, 0xf6, 0xe2, 0xf4, 0x0e, 0x1e, 0x0b, 0x28, 
	0x0b, 0x07, 0x00, 0xfd, 0xe1, 0xfd, 0xfd, 0x08, 0x0c, 0x29, 
	0xfa, 0xfd, 0xf5, 0x11, 0xfd, 0x04, 0xfe, 0x00, 0x1b, 0x1d, 
	0x06, 0x05, 0x03, 0x04, 0x05, 0xfc, 0xf9, 0xfc, 0x03, 0x1c,
};

void main()
{
	int j;

	irq_setmask(0);
    irq_setie(1);
    flag = 0;

    irq_setmask(irq_getmask() | (1 << USER_IRQ_0_INTERRUPT));


	reg_mprj_io_31 = GPIO_MODE_MGMT_STD_OUTPUT;
	reg_mprj_io_30 = GPIO_MODE_MGMT_STD_OUTPUT;
	reg_mprj_io_29 = GPIO_MODE_MGMT_STD_OUTPUT;
	reg_mprj_io_28 = GPIO_MODE_MGMT_STD_OUTPUT;
	reg_mprj_io_27 = GPIO_MODE_MGMT_STD_OUTPUT;
	reg_mprj_io_26 = GPIO_MODE_MGMT_STD_OUTPUT;
	reg_mprj_io_25 = GPIO_MODE_MGMT_STD_OUTPUT;
	reg_mprj_io_24 = GPIO_MODE_MGMT_STD_OUTPUT;
	reg_mprj_io_23 = GPIO_MODE_MGMT_STD_OUTPUT;
	reg_mprj_io_22 = GPIO_MODE_MGMT_STD_OUTPUT;
	reg_mprj_io_21 = GPIO_MODE_MGMT_STD_OUTPUT;
	reg_mprj_io_20 = GPIO_MODE_MGMT_STD_OUTPUT;
	reg_mprj_io_19 = GPIO_MODE_MGMT_STD_OUTPUT;
	reg_mprj_io_18 = GPIO_MODE_MGMT_STD_OUTPUT;
	reg_mprj_io_17 = GPIO_MODE_MGMT_STD_OUTPUT;
	reg_mprj_io_16 = GPIO_MODE_MGMT_STD_OUTPUT;

	reg_mprj_io_15 = GPIO_MODE_USER_STD_OUTPUT;
	reg_mprj_io_14 = GPIO_MODE_USER_STD_OUTPUT;
	reg_mprj_io_13 = GPIO_MODE_USER_STD_OUTPUT;
	reg_mprj_io_12 = GPIO_MODE_USER_STD_OUTPUT;
	reg_mprj_io_11 = GPIO_MODE_USER_STD_OUTPUT;
	reg_mprj_io_10 = GPIO_MODE_USER_STD_OUTPUT;
	reg_mprj_io_9 = GPIO_MODE_USER_STD_OUTPUT;
	reg_mprj_io_8 = GPIO_MODE_USER_STD_OUTPUT;
	reg_mprj_io_7 = GPIO_MODE_USER_STD_OUTPUT;
	reg_mprj_io_5 = GPIO_MODE_USER_STD_OUTPUT;
	reg_mprj_io_4 = GPIO_MODE_USER_STD_OUTPUT;
	reg_mprj_io_3 = GPIO_MODE_USER_STD_OUTPUT;
	reg_mprj_io_2 = GPIO_MODE_USER_STD_OUTPUT;
	reg_mprj_io_1 = GPIO_MODE_USER_STD_OUTPUT;
	reg_mprj_io_0 = GPIO_MODE_USER_STD_OUTPUT;

	reg_mprj_io_6 = GPIO_MODE_MGMT_STD_OUTPUT;

	reg_wb_enable = 1;

	// Now, apply the configuration
	reg_mprj_xfer = 1;
	while (reg_mprj_xfer == 1)
		;

    reg_mprj_irq = 0b111;
	reg_user_irq_enable = 1;
    reg_user0_irq_en = 1;

	uint8_t beta = 0xE6;
	uint8_t vth = 0x70;
	uint16_t timesteps = 0x64;

	uint32_t control_data = beta | (vth << 8) | (timesteps << 16); // set beta to EF, set vth to DC, 1 timestep

	// Flag start of the test
	reg_mprj_datal = 0xAB400000;

	// start initializing user area -------------------------------------------------- 
	reg_snn_control = control_data;

	uint32_t i;
	for (i = 0; i < NUM_PIXELS; i++) // send image data
	{
		image_write(image[i], i);
	}

	for (i = 0; i < NUM_WEIGHTS; i++) // send weight data
	{
		weight_write(weights[i], i);
	}

	reg_snn_control = (control_data | 0x10000000); // start inference
	// done initializing user area -------------------------------------------------- 

	// wait for interrupt to say we are done
	while (!flag) {}

	reg_mprj_datal = 0xAB410000;
	reg_mprj_datal = 0xAB510000;
}
