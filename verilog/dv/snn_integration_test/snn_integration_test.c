/*
 * SPDX-FileCopyrightText: 2020 Efabless Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * SPDX-License-Identifier: Apache-2.0
 */

// This include is relative to $CARAVEL_PATH (see Makefile)
#include <defs.h>
#include <stub.c>

#define reg_snn_control (*(volatile uint32_t *)0x30004000)

#define image_write(data, index) (*(volatile uint32_t *)(0x30000000 + (index << 2))) = data
#define weight0_write(data, index) (*(volatile uint32_t *)(0x30001000 + (index << 2))) = data
#define weight1_write(data, index) (*(volatile uint32_t *)(0x30002000 + (index << 2))) = data

#define WIDTH 14
#define HEIGHT 14
#define OUTPUTS 10
#define NUM_PIXELS HEIGHT *WIDTH
#define NUM_WEIGHTS NUM_PIXELS *OUTPUTS

uint8_t image[NUM_PIXELS];

// uint8_t image[NUM_PIXELS] = {
// 	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
// 	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
// 	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
// 	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
// 	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
// 	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
// 	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
// 	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
// 	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
// 	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
// 	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
// 	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
// 	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
// 	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
// };

void main()
{
	int j;

	uint8_t weights[NUM_WEIGHTS];

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

	// Flag start of the test
	reg_mprj_datal = 0xAB400000;

	// initialize user area
	reg_snn_control = 0x0001DCEF; // set beta to EF, set vth to DC, 1 timestep

	uint32_t i;

	for (i = 0; i < NUM_PIXELS; i++)
	{
		image_write(image[i], i);
	}

	for (i = 0; i < NUM_WEIGHTS / 2; i++)
	{
		weight0_write(weights[i], i);
	}

	for (i = 0; i < NUM_WEIGHTS / 2; i++)
	{
		weight1_write(weights[i + NUM_WEIGHTS / 2], i);
	}

	reg_snn_control |= 0x01000000; // (0x1 << 29);

	// done initializing user area

	reg_mprj_datal = 0xAB410000;
	reg_mprj_datal = 0xAB510000;
}
