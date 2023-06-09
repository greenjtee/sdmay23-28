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
0x00, 0x00, 0x04, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x10, 0x6c, 0x69, 0x57, 0x57, 0x48, 0x08, 0x00, 
0x00, 0x05, 0x20, 0x36, 0x48, 0x67, 0x99, 0x0e, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x03, 0x6c, 0x54, 0x01, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x26, 0x83, 0x10, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x0c, 0x83, 0x44, 0x01, 0x00, 0x00, 
0x00, 0x00, 0x01, 0x54, 0x8b, 0x08, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x03, 0x69, 0x49, 0x01, 0x00, 0x00, 0x00,
};

uint8_t weights[NUM_WEIGHTS] = {
	0x7e, 0x90, 0x7a, 0x65, 0x7b, 0x39, 0x3f, 0x46, 0x89, 
0x7b, 0x8c, 0x65, 0x7b, 0x7e, 0x83, 0x7c, 0x63, 0x5a, 
0x8f, 0x79, 0x84, 0x73, 0x8d, 0x8f, 0x82, 0x62, 0x39, 
0x5d, 0x74, 0x7d, 0x89, 0x84, 0x84, 0x8f, 0xae, 0x53, 
0x5a, 0x87, 0x89, 0x8a, 0x5e, 0x5e, 0x6f, 0xa1, 0x93, 
0x63, 0x85, 0xa5, 0x7a, 0x6c, 0x71, 0x8e, 0x7b, 0x72, 
0x40, 0x7a, 0x8d, 0x80, 0x8c, 0x7f, 0x76, 0x7a, 0x81, 
0x72, 0x78, 0x74, 0xa0, 0x87, 0x8a, 0x7c, 0x76, 0x81, 
0x92, 0x98, 0x61, 0x47, 0x70, 0x7c, 0x86, 0x6e, 0x8e, 
0x96, 0x93, 0x61, 0x8b, 0x95, 0x43, 0x24, 0x36, 0x51, 
0x93, 0x7f, 0x71, 0x63, 0x8e, 0x7f, 0x98, 0x89, 0x74, 
0x9e, 0x77, 0x6b, 0x6b, 0x82, 0x89, 0x7c, 0x76, 0x56, 
0x87, 0x70, 0x91, 0x57, 0xa4, 0x85, 0x74, 0x85, 0x57, 
0x91, 0x68, 0x98, 0x4b, 0xa8, 0x6a, 0x86, 0x8d, 0x60, 
0x69, 0x6e, 0x81, 0x69, 0xb0, 0x54, 0x94, 0x72, 0x6a, 
0x60, 0x77, 0x64, 0x82, 0x94, 0x6c, 0x6e, 0x77, 0x7a, 
0x71, 0x73, 0x84, 0x73, 0x8a, 0x90, 0x7b, 0x78, 0x8d, 
0x6d, 0x75, 0x6b, 0x59, 0x60, 0x80, 0x7b, 0x99, 0x7f, 
0x87, 0x90, 0x7e, 0x7e, 0x78, 0x5f, 0x3e, 0x4d, 0x6d, 
0x66, 0x7b, 0x98, 0x9a, 0x96, 0x8f, 0x7f, 0x74, 0x68, 
0x8e, 0x90, 0x6e, 0x8e, 0x83, 0x85, 0x7d, 0x86, 0x8f, 
0xb6, 0x7f, 0x72, 0x66, 0x6c, 0x84, 0x84, 0x7e, 0x7a, 
0xaf, 0x6c, 0x76, 0x72, 0x6b, 0x82, 0x72, 0x51, 0x9a, 
0x7a, 0x91, 0x83, 0x8b, 0x95, 0x92, 0x74, 0x77, 0xdb, 
0x67, 0xa2, 0xa1, 0x86, 0x88, 0x8b, 0x8a, 0xc2, 0xe5, 
0x69, 0x82, 0x77, 0x77, 0x64, 0x70, 0x99, 0x99, 0xa4, 
0x59, 0x6b, 0x7d, 0x91, 0xad, 0x91, 0x60, 0x55, 0x6d, 
0x82, 0x5e, 0x6a, 0x5e, 0x86, 0x9d, 0xcd, 0xaa, 0x7e, 
0xb5, 0xa1, 0xa0, 0xa1, 0x9a, 0x82, 0x6e, 0x83, 0x75, 
0x8d, 0x9a, 0x89, 0x7e, 0x7e, 0x7e, 0x86, 0x76, 0x9c, 
0x80, 0x6e, 0x64, 0x53, 0x88, 0x9d, 0x72, 0x80, 0xa0, 
0x9b, 0x8c, 0x7a, 0x90, 0x97, 0x8d, 0x81, 0x81, 0x96, 
0xa7, 0x85, 0x7d, 0x6b, 0x64, 0x8e, 0x98, 0x7e, 0x8b, 
0xc3, 0xa5, 0x6b, 0x65, 0x7c, 0x83, 0x8f, 0x64, 0x45, 
0x9f, 0x99, 0x95, 0xa0, 0x81, 0x89, 0x77, 0x58, 0x62, 
0xba, 0xb7, 0x98, 0x75, 0x8c, 0x76, 0x7f, 0x7f, 0x8e, 
0x76, 0x3b, 0x4b, 0x1c, 0x3b, 0x34, 0x3a, 0x6e, 0x52, 
0x85, 0x8c, 0x8b, 0x86, 0x70, 0x88, 0x7c, 0xa1, 0x9f, 
0x91, 0x9a, 0x7c, 0x7c, 0x5e, 0x64, 0x9a, 0x84, 0x79, 
0x7b, 0x61, 0x7c, 0x82, 0x74, 0x90, 0x78, 0x78, 0x68, 
0x80, 0x90, 0x9c, 0x91, 0x7a, 0x9e, 0x8b, 0x85, 0x87, 
0x56, 0x99, 0x95, 0x87, 0x94, 0x8d, 0x80, 0x81, 0x5f, 
0x7f, 0x61, 0x63, 0x5b, 0x7f, 0x83, 0x66, 0x84, 0x89, 
0x67, 0x76, 0x75, 0x82, 0x7f, 0x8a, 0x97, 0x70, 0x88, 
0x68, 0x5e, 0x6b, 0x4e, 0x4a, 0x42, 0x2f, 0x6d, 0x6c, 
0x7b, 0x6e, 0x9a, 0x97, 0x82, 0x73, 0x40, 0x47, 0x6c, 
0x72, 0x4b, 0x78, 0x80, 0x74, 0x74, 0x8d, 0xa9, 0xc7, 
0x6a, 0x72, 0x7f, 0x83, 0x79, 0x7f, 0x80, 0xbc, 0xf8, 
0x80, 0x89, 0x8f, 0x92, 0x87, 0x67, 0x5c, 0x53, 0xc5, 
0x98, 0x77, 0x86, 0x9d, 0x80, 0x72, 0x6f, 0x55, 0x76, 
0x97, 0x83, 0x6b, 0x5f, 0x7c, 0x88, 0x82, 0x84, 0x82, 
0x76, 0x8c, 0x94, 0x91, 0x76, 0x81, 0x85, 0x8c, 0x6a, 
0x87, 0x5c, 0x8e, 0x81, 0x8e, 0x78, 0x9a, 0x85, 0x7f, 
0x76, 0x78, 0x81, 0x8e, 0x7f, 0x80, 0x74, 0x75, 0x95, 
0xa5, 0xa2, 0xb1, 0xba, 0xe1, 0xfc, 0xf9, 0xba, 0xae, 
0x92, 0x68, 0x6d, 0x79, 0x67, 0x93, 0x96, 0x79, 0x85, 
0x80, 0x7a, 0x80, 0x72, 0x80, 0x5e, 0x5a, 0x74, 0x7f, 
0x91, 0x7c, 0x79, 0x7e, 0x7c, 0x68, 0x68, 0x71, 0x6e, 
0x7a, 0x8e, 0x7e, 0x90, 0x81, 0x89, 0x8a, 0xa6, 0xa6, 
0x8e, 0x60, 0x75, 0x9c, 0x78, 0x75, 0x8f, 0x9e, 0x56, 
0x97, 0x6e, 0x74, 0x9e, 0xa1, 0x9b, 0x8b, 0x5f, 0x5d, 
0x85, 0x96, 0x7d, 0x6f, 0x76, 0x79, 0x77, 0x74, 0x7a, 
0x88, 0xa3, 0xb9, 0xb3, 0x93, 0x8f, 0x89, 0xab, 0x79, 
0x89, 0x70, 0x6d, 0x89, 0xa9, 0xa2, 0xaf, 0xab, 0x9b, 
0x97, 0x77, 0x85, 0x7f, 0x77, 0x88, 0x88, 0x8e, 0x8f, 
0xad, 0x91, 0x91, 0x94, 0x84, 0x78, 0x86, 0x85, 0x72, 
0xa3, 0xab, 0x92, 0x8b, 0x96, 0xa1, 0x80, 0x8c, 0x5e, 
0xbd, 0x6f, 0x5a, 0x67, 0x4b, 0x89, 0x89, 0x6c, 0xa3, 
0x73, 0x7f, 0x81, 0x70, 0x75, 0x88, 0x7b, 0x8a, 0x6e, 
0x68, 0x75, 0x6e, 0x7d, 0x8d, 0x6b, 0x7c, 0x76, 0x7e, 
0x67, 0x64, 0x78, 0x73, 0x6e, 0x70, 0x61, 0x62, 0x93, 
0x91, 0xb0, 0xcc, 0xca, 0xc2, 0xab, 0x87, 0x70, 0x80, 
0x9b, 0x7a, 0xbb, 0xa4, 0x82, 0x57, 0x7c, 0x9a, 0x88, 
0x76, 0x60, 0x89, 0x74, 0x89, 0x95, 0x7c, 0x71, 0x81, 
0x5a, 0x87, 0x71, 0x92, 0x80, 0x73, 0x8a, 0x88, 0x8b, 
0x76, 0x89, 0x96, 0x93, 0x80, 0x6e, 0x98, 0xa5, 0x93, 
0x96, 0x78, 0x5a, 0x77, 0xa5, 0x89, 0x75, 0x80, 0x81, 
0xaa, 0x75, 0x77, 0x9a, 0x96, 0x72, 0x6a, 0x78, 0x88, 
0x6f, 0x74, 0x85, 0x8f, 0x60, 0x7c, 0x76, 0x67, 0x78, 
0x7f, 0x65, 0x7a, 0x8f, 0x9b, 0x8f, 0x8c, 0x88, 0x56, 
0x79, 0x59, 0x57, 0x65, 0x7e, 0x86, 0x7c, 0x9a, 0x76, 
0x71, 0x8f, 0xb0, 0x97, 0x88, 0x69, 0x7e, 0x86, 0xa3, 
0xa8, 0x95, 0x96, 0x7f, 0x80, 0x79, 0x83, 0x75, 0x8e, 
0x57, 0x57, 0x56, 0x7e, 0xa9, 0x9a, 0x74, 0x6d, 0x78, 
0x5e, 0x79, 0x7c, 0x81, 0x6c, 0x88, 0x87, 0x88, 0x7a, 
0x76, 0x86, 0xab, 0x87, 0x85, 0x9c, 0x7f, 0x87, 0x64, 
0x6b, 0x81, 0x72, 0x88, 0x77, 0x87, 0x6b, 0x72, 0x7d, 
0x73, 0x7c, 0x7a, 0x66, 0x7b, 0x71, 0x74, 0x85, 0x79, 
0x67, 0x8a, 0x6e, 0x74, 0x62, 0x7f, 0x84, 0x96, 0x80, 
0x70, 0x93, 0x87, 0x9d, 0xa4, 0xb7, 0xd1, 0xc0, 0xa6,
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

	uint32_t control_data = 0x0001DCEF; // set beta to EF, set vth to DC, 1 timestep

	// Flag start of the test
	reg_mprj_datal = 0xAB400000;

	// initialize user area
	reg_snn_control = control_data;

	// uint32_t i;
	// for (i = 0; i < NUM_PIXELS; i++)
	// {
	// 	image_write(image[i], i);
	// }

	// for (i = 0; i < NUM_WEIGHTS; i++)
	// {
	// 	weight_write(weights[i], i);
	// }

	reg_snn_control = (control_data | 0x10000000); // (0x1 << 29);

	// done initializing user area

	// wait for interrupt to say we are done
	// while (!(reg_snn_control & 0x01000000)) {
	// i = 0;
	// while (i < 100000) {
		// i++;
    // }

	while (!flag) {}

	reg_mprj_datal = 0xAB410000;
	reg_mprj_datal = 0xAB510000;
}
