// SPDX-FileCopyrightText: 2020 Efabless Corporation
//
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

`default_nettype none

`timescale 1 ns / 1 ps

module snn_simple_integration_test_tb;

	parameter PERIOD = 10;

	reg clock;
	reg rst;
	
	reg insert_i;
	reg read_i;
    reg [7:0] data_i;

    wire valid_o;
    wire [7:0] data_o;

	integer i;

	initial begin 
		clock = 0;
		forever begin
		#5 clock = ~clock;
	end end

	initial
	begin
		$dumpfile("snn_simple_integration_test.vcd");
		$dumpvars(0, snn_simple_integration_test_tb);

		// initialize image, this is a 1
dut.image.mem[0] = 8'h00;
dut.image.mem[1] = 8'h00;
dut.image.mem[2] = 8'h00;
dut.image.mem[3] = 8'h00;
dut.image.mem[4] = 8'h00;
dut.image.mem[5] = 8'h01;
dut.image.mem[6] = 8'h00;
dut.image.mem[7] = 8'h00;
dut.image.mem[8] = 8'h00;
dut.image.mem[9] = 8'h00;
dut.image.mem[10] = 8'h00;
dut.image.mem[11] = 8'h00;
dut.image.mem[12] = 8'h00;
dut.image.mem[13] = 8'h02;
dut.image.mem[14] = 8'h24;
dut.image.mem[15] = 8'h07;
dut.image.mem[16] = 8'h00;
dut.image.mem[17] = 8'h00;
dut.image.mem[18] = 8'h00;
dut.image.mem[19] = 8'h00;
dut.image.mem[20] = 8'h00;
dut.image.mem[21] = 8'h00;
dut.image.mem[22] = 8'h0d;
dut.image.mem[23] = 8'h33;
dut.image.mem[24] = 8'h03;
dut.image.mem[25] = 8'h00;
dut.image.mem[26] = 8'h00;
dut.image.mem[27] = 8'h00;
dut.image.mem[28] = 8'h00;
dut.image.mem[29] = 8'h00;
dut.image.mem[30] = 8'h00;
dut.image.mem[31] = 8'h22;
dut.image.mem[32] = 8'h2c;
dut.image.mem[33] = 8'h00;
dut.image.mem[34] = 8'h00;
dut.image.mem[35] = 8'h00;
dut.image.mem[36] = 8'h00;
dut.image.mem[37] = 8'h00;
dut.image.mem[38] = 8'h00;
dut.image.mem[39] = 8'h02;
dut.image.mem[40] = 8'h39;
dut.image.mem[41] = 8'h16;
dut.image.mem[42] = 8'h00;
dut.image.mem[43] = 8'h00;
dut.image.mem[44] = 8'h00;
dut.image.mem[45] = 8'h00;
dut.image.mem[46] = 8'h00;
dut.image.mem[47] = 8'h00;
dut.image.mem[48] = 8'h0c;
dut.image.mem[49] = 8'h40;
dut.image.mem[50] = 8'h06;
dut.image.mem[51] = 8'h00;
dut.image.mem[52] = 8'h00;
dut.image.mem[53] = 8'h00;
dut.image.mem[54] = 8'h00;
dut.image.mem[55] = 8'h00;
dut.image.mem[56] = 8'h00;
dut.image.mem[57] = 8'h20;
dut.image.mem[58] = 8'h3d;
dut.image.mem[59] = 8'h01;
dut.image.mem[60] = 8'h00;
dut.image.mem[61] = 8'h00;
dut.image.mem[62] = 8'h00;
dut.image.mem[63] = 8'h00;
dut.image.mem[64] = 8'h00;
dut.image.mem[65] = 8'h00;
dut.image.mem[66] = 8'h1f;
dut.image.mem[67] = 8'h1a;
dut.image.mem[68] = 8'h00;
dut.image.mem[69] = 8'h00;
dut.image.mem[70] = 8'h00;
dut.image.mem[71] = 8'h00;
dut.image.mem[72] = 8'h00;
dut.image.mem[73] = 8'h00;
dut.image.mem[74] = 8'h00;
dut.image.mem[75] = 8'h01;
dut.image.mem[76] = 8'h00;
dut.image.mem[77] = 8'h00;
dut.image.mem[78] = 8'h00;
dut.image.mem[79] = 8'h00;
dut.image.mem[80] = 8'h00;

		// initialize weights
		dut.weights.mem[0] = 8'b0;
		dut.weights.mem[1] = 8'b11111101;
		dut.weights.mem[2] = 8'b101;
		dut.weights.mem[3] = 8'b11111101;
		dut.weights.mem[4] = 8'b11101010;
		dut.weights.mem[5] = 8'b1111;
		dut.weights.mem[6] = 8'b11111011;
		dut.weights.mem[7] = 8'b1010;
		dut.weights.mem[8] = 8'b111;
		dut.weights.mem[9] = 8'b11110110;
		dut.weights.mem[10] = 8'b11;
		dut.weights.mem[11] = 8'b101;
		dut.weights.mem[12] = 8'b1111;
		dut.weights.mem[13] = 8'b11110111;
		dut.weights.mem[14] = 8'b11111100;
		dut.weights.mem[15] = 8'b11;
		dut.weights.mem[16] = 8'b110;
		dut.weights.mem[17] = 8'b11111111;
		dut.weights.mem[18] = 8'b10011;
		dut.weights.mem[19] = 8'b0;
		dut.weights.mem[20] = 8'b11110111;
		dut.weights.mem[21] = 8'b11111010;
		dut.weights.mem[22] = 8'b1;
		dut.weights.mem[23] = 8'b11101011;
		dut.weights.mem[24] = 8'b11100101;
		dut.weights.mem[25] = 8'b10100;
		dut.weights.mem[26] = 8'b10100;
		dut.weights.mem[27] = 8'b111;
		dut.weights.mem[28] = 8'b10111;
		dut.weights.mem[29] = 8'b1000;
		dut.weights.mem[30] = 8'b11111011;
		dut.weights.mem[31] = 8'b101;
		dut.weights.mem[32] = 8'b100;
		dut.weights.mem[33] = 8'b100;
		dut.weights.mem[34] = 8'b11010100;
		dut.weights.mem[35] = 8'b1100;
		dut.weights.mem[36] = 8'b101010;
		dut.weights.mem[37] = 8'b11010;
		dut.weights.mem[38] = 8'b110;
		dut.weights.mem[39] = 8'b1000;
		dut.weights.mem[40] = 8'b11110101;
		dut.weights.mem[41] = 8'b111;
		dut.weights.mem[42] = 8'b11110100;
		dut.weights.mem[43] = 8'b111;
		dut.weights.mem[44] = 8'b11011000;
		dut.weights.mem[45] = 8'b101;
		dut.weights.mem[46] = 8'b111000;
		dut.weights.mem[47] = 8'b10101;
		dut.weights.mem[48] = 8'b11110111;
		dut.weights.mem[49] = 8'b1010;
		dut.weights.mem[50] = 8'b11011000;
		dut.weights.mem[51] = 8'b11011100;
		dut.weights.mem[52] = 8'b11100111;
		dut.weights.mem[53] = 8'b11110;
		dut.weights.mem[54] = 8'b11011001;
		dut.weights.mem[55] = 8'b11111110;
		dut.weights.mem[56] = 8'b111000;
		dut.weights.mem[57] = 8'b100101;
		dut.weights.mem[58] = 8'b11110101;
		dut.weights.mem[59] = 8'b11111001;
		dut.weights.mem[60] = 8'b11011100;
		dut.weights.mem[61] = 8'b11010110;
		dut.weights.mem[62] = 8'b11100110;
		dut.weights.mem[63] = 8'b110000;
		dut.weights.mem[64] = 8'b11100111;
		dut.weights.mem[65] = 8'b11101000;
		dut.weights.mem[66] = 8'b1000001;
		dut.weights.mem[67] = 8'b1010;
		dut.weights.mem[68] = 8'b11111101;
		dut.weights.mem[69] = 8'b1011;
		dut.weights.mem[70] = 8'b11101110;
		dut.weights.mem[71] = 8'b11101000;
		dut.weights.mem[72] = 8'b11100001;
		dut.weights.mem[73] = 8'b101000;
		dut.weights.mem[74] = 8'b11101011;
		dut.weights.mem[75] = 8'b11011110;
		dut.weights.mem[76] = 8'b101110;
		dut.weights.mem[77] = 8'b10000;
		dut.weights.mem[78] = 8'b11111000;
		dut.weights.mem[79] = 8'b10010;
		dut.weights.mem[80] = 8'b11111101;
		dut.weights.mem[81] = 8'b11100011;
		dut.weights.mem[82] = 8'b0;
		dut.weights.mem[83] = 8'b1111;
		dut.weights.mem[84] = 8'b11110001;
		dut.weights.mem[85] = 8'b101;
		dut.weights.mem[86] = 8'b1101;
		dut.weights.mem[87] = 8'b11111010;
		dut.weights.mem[88] = 8'b10;
		dut.weights.mem[89] = 8'b10001;
		dut.weights.mem[90] = 8'b11111010;
		dut.weights.mem[91] = 8'b0;
		dut.weights.mem[92] = 8'b11110011;
		dut.weights.mem[93] = 8'b10000;
		dut.weights.mem[94] = 8'b11111111;
		dut.weights.mem[95] = 8'b11111011;
		dut.weights.mem[96] = 8'b11111101;
		dut.weights.mem[97] = 8'b11111011;
		dut.weights.mem[98] = 8'b11;
		dut.weights.mem[99] = 8'b1100;
		dut.weights.mem[100] = 8'b11110110;
		dut.weights.mem[101] = 8'b1;
		dut.weights.mem[102] = 8'b11111001;
		dut.weights.mem[103] = 8'b11001;
		dut.weights.mem[104] = 8'b11;
		dut.weights.mem[105] = 8'b11101011;
		dut.weights.mem[106] = 8'b11111101;
		dut.weights.mem[107] = 8'b101;
		dut.weights.mem[108] = 8'b11111110;
		dut.weights.mem[109] = 8'b1101;
		dut.weights.mem[110] = 8'b11111101;
		dut.weights.mem[111] = 8'b11101111;
		dut.weights.mem[112] = 8'b1111;
		dut.weights.mem[113] = 8'b10001;
		dut.weights.mem[114] = 8'b1000;
		dut.weights.mem[115] = 8'b11111011;
		dut.weights.mem[116] = 8'b110;
		dut.weights.mem[117] = 8'b11110111;
		dut.weights.mem[118] = 8'b11110010;
		dut.weights.mem[119] = 8'b1100;
		dut.weights.mem[120] = 8'b11110110;
		dut.weights.mem[121] = 8'b11111010;
		dut.weights.mem[122] = 8'b1001;
		dut.weights.mem[123] = 8'b10111;
		dut.weights.mem[124] = 8'b11111000;
		dut.weights.mem[125] = 8'b0;
		dut.weights.mem[126] = 8'b11110101;
		dut.weights.mem[127] = 8'b11111101;
		dut.weights.mem[128] = 8'b11111101;
		dut.weights.mem[129] = 8'b1001;
		dut.weights.mem[130] = 8'b0;
		dut.weights.mem[131] = 8'b0;
		dut.weights.mem[132] = 8'b10010;
		dut.weights.mem[133] = 8'b1110;
		dut.weights.mem[134] = 8'b10;
		dut.weights.mem[135] = 8'b11111011;
		dut.weights.mem[136] = 8'b10;
		dut.weights.mem[137] = 8'b0;
		dut.weights.mem[138] = 8'b110;
		dut.weights.mem[139] = 8'b11111110;
		dut.weights.mem[140] = 8'b10;
		dut.weights.mem[141] = 8'b110;
		dut.weights.mem[142] = 8'b101;
		dut.weights.mem[143] = 8'b11;
		dut.weights.mem[144] = 8'b11;
		dut.weights.mem[145] = 8'b11111001;
		dut.weights.mem[146] = 8'b111;
		dut.weights.mem[147] = 8'b0;
		dut.weights.mem[148] = 8'b1;
		dut.weights.mem[149] = 8'b11111111;
		dut.weights.mem[150] = 8'b11111011;
		dut.weights.mem[151] = 8'b101;
		dut.weights.mem[152] = 8'b11111111;
		dut.weights.mem[153] = 8'b11111010;
		dut.weights.mem[154] = 8'b11;
		dut.weights.mem[155] = 8'b100;
		dut.weights.mem[156] = 8'b1101;
		dut.weights.mem[157] = 8'b1010;
		dut.weights.mem[158] = 8'b100;
		dut.weights.mem[159] = 8'b11111110;
		dut.weights.mem[160] = 8'b11110010;
		dut.weights.mem[161] = 8'b100;
		dut.weights.mem[162] = 8'b11111010;
		dut.weights.mem[163] = 8'b11110101;
		dut.weights.mem[164] = 8'b10100;
		dut.weights.mem[165] = 8'b100100;
		dut.weights.mem[166] = 8'b101;
		dut.weights.mem[167] = 8'b1001;
		dut.weights.mem[168] = 8'b11110100;
		dut.weights.mem[169] = 8'b1;
		dut.weights.mem[170] = 8'b11110000;
		dut.weights.mem[171] = 8'b11111001;
		dut.weights.mem[172] = 8'b11111001;
		dut.weights.mem[173] = 8'b1001;
		dut.weights.mem[174] = 8'b1011;
		dut.weights.mem[175] = 8'b101101;
		dut.weights.mem[176] = 8'b1001;
		dut.weights.mem[177] = 8'b1010;
		dut.weights.mem[178] = 8'b11110001;
		dut.weights.mem[179] = 8'b1100;
		dut.weights.mem[180] = 8'b11110101;
		dut.weights.mem[181] = 8'b10101;
		dut.weights.mem[182] = 8'b11111001;
		dut.weights.mem[183] = 8'b11111010;
		dut.weights.mem[184] = 8'b110;
		dut.weights.mem[185] = 8'b11111101;
		dut.weights.mem[186] = 8'b1010;
		dut.weights.mem[187] = 8'b11101;
		dut.weights.mem[188] = 8'b11111001;
		dut.weights.mem[189] = 8'b11110100;
		dut.weights.mem[190] = 8'b1110;
		dut.weights.mem[191] = 8'b11111101;
		dut.weights.mem[192] = 8'b1001;
		dut.weights.mem[193] = 8'b1011;
		dut.weights.mem[194] = 8'b100;
		dut.weights.mem[195] = 8'b11111011;
		dut.weights.mem[196] = 8'b0;
		dut.weights.mem[197] = 8'b101;
		dut.weights.mem[198] = 8'b11111010;
		dut.weights.mem[199] = 8'b11101111;
		dut.weights.mem[200] = 8'b11111100;
		dut.weights.mem[201] = 8'b11111001;
		dut.weights.mem[202] = 8'b11111010;
		dut.weights.mem[203] = 8'b1110;
		dut.weights.mem[204] = 8'b1000;
		dut.weights.mem[205] = 8'b11111110;
		dut.weights.mem[206] = 8'b11111110;
		dut.weights.mem[207] = 8'b10001;
		dut.weights.mem[208] = 8'b101;
		dut.weights.mem[209] = 8'b11101101;
		dut.weights.mem[210] = 8'b10;
		dut.weights.mem[211] = 8'b11110101;
		dut.weights.mem[212] = 8'b11;
		dut.weights.mem[213] = 8'b10;
		dut.weights.mem[214] = 8'b11111110;
		dut.weights.mem[215] = 8'b100;
		dut.weights.mem[216] = 8'b11;
		dut.weights.mem[217] = 8'b1000;
		dut.weights.mem[218] = 8'b10;
		dut.weights.mem[219] = 8'b11111000;
		dut.weights.mem[220] = 8'b110;
		dut.weights.mem[221] = 8'b100;
		dut.weights.mem[222] = 8'b11111110;
		dut.weights.mem[223] = 8'b0;
		dut.weights.mem[224] = 8'b11100111;
		dut.weights.mem[225] = 8'b11111101;
		dut.weights.mem[226] = 8'b11111010;
		dut.weights.mem[227] = 8'b10;
		dut.weights.mem[228] = 8'b11111110;
		dut.weights.mem[229] = 8'b11011;
		dut.weights.mem[230] = 8'b1101;
		dut.weights.mem[231] = 8'b11111111;
		dut.weights.mem[232] = 8'b111;
		dut.weights.mem[233] = 8'b0;
		dut.weights.mem[234] = 8'b11110101;
		dut.weights.mem[235] = 8'b0;
		dut.weights.mem[236] = 8'b11110101;
		dut.weights.mem[237] = 8'b11111011;
		dut.weights.mem[238] = 8'b11111111;
		dut.weights.mem[239] = 8'b1110;
		dut.weights.mem[240] = 8'b10;
		dut.weights.mem[241] = 8'b10;
		dut.weights.mem[242] = 8'b11111010;
		dut.weights.mem[243] = 8'b110;
		dut.weights.mem[244] = 8'b1001;
		dut.weights.mem[245] = 8'b10;
		dut.weights.mem[246] = 8'b11110100;
		dut.weights.mem[247] = 8'b0;
		dut.weights.mem[248] = 8'b0;
		dut.weights.mem[249] = 8'b11111110;
		dut.weights.mem[250] = 8'b11111000;
		dut.weights.mem[251] = 8'b11110101;
		dut.weights.mem[252] = 8'b111;
		dut.weights.mem[253] = 8'b11111111;
		dut.weights.mem[254] = 8'b10;
		dut.weights.mem[255] = 8'b11000;
		dut.weights.mem[256] = 8'b11110101;
		dut.weights.mem[257] = 8'b11111110;
		dut.weights.mem[258] = 8'b1000;
		dut.weights.mem[259] = 8'b11110011;
		dut.weights.mem[260] = 8'b11001111;
		dut.weights.mem[261] = 8'b11101100;
		dut.weights.mem[262] = 8'b11111100;
		dut.weights.mem[263] = 8'b10001;
		dut.weights.mem[264] = 8'b100;
		dut.weights.mem[265] = 8'b1001010;
		dut.weights.mem[266] = 8'b1111;
		dut.weights.mem[267] = 8'b1101;
		dut.weights.mem[268] = 8'b11111111;
		dut.weights.mem[269] = 8'b11110110;
		dut.weights.mem[270] = 8'b11111011;
		dut.weights.mem[271] = 8'b11111111;
		dut.weights.mem[272] = 8'b1111;
		dut.weights.mem[273] = 8'b11111011;
		dut.weights.mem[274] = 8'b11111001;
		dut.weights.mem[275] = 8'b101;
		dut.weights.mem[276] = 8'b11;
		dut.weights.mem[277] = 8'b10100;
		dut.weights.mem[278] = 8'b11101010;
		dut.weights.mem[279] = 8'b11100101;
		dut.weights.mem[280] = 8'b11110101;
		dut.weights.mem[281] = 8'b11111011;
		dut.weights.mem[282] = 8'b1;
		dut.weights.mem[283] = 8'b11;
		dut.weights.mem[284] = 8'b11111110;
		dut.weights.mem[285] = 8'b0;
		dut.weights.mem[286] = 8'b11111110;
		dut.weights.mem[287] = 8'b11011;
		dut.weights.mem[288] = 8'b1011;
		dut.weights.mem[289] = 8'b11101101;
		dut.weights.mem[290] = 8'b11111111;
		dut.weights.mem[291] = 8'b110;
		dut.weights.mem[292] = 8'b11111001;
		dut.weights.mem[293] = 8'b11100110;
		dut.weights.mem[294] = 8'b11110101;
		dut.weights.mem[295] = 8'b1011;
		dut.weights.mem[296] = 8'b101;
		dut.weights.mem[297] = 8'b100;
		dut.weights.mem[298] = 8'b100;
		dut.weights.mem[299] = 8'b110;
		dut.weights.mem[300] = 8'b10;
		dut.weights.mem[301] = 8'b11101101;
		dut.weights.mem[302] = 8'b11110101;
		dut.weights.mem[303] = 8'b11101110;
		dut.weights.mem[304] = 8'b1000;
		dut.weights.mem[305] = 8'b111;
		dut.weights.mem[306] = 8'b11111111;
		dut.weights.mem[307] = 8'b100;
		dut.weights.mem[308] = 8'b10100;
		dut.weights.mem[309] = 8'b111;
		dut.weights.mem[310] = 8'b1;
		dut.weights.mem[311] = 8'b10100;
		dut.weights.mem[312] = 8'b11110110;
		dut.weights.mem[313] = 8'b1000;
		dut.weights.mem[314] = 8'b11111100;
		dut.weights.mem[315] = 8'b100;
		dut.weights.mem[316] = 8'b110;
		dut.weights.mem[317] = 8'b1110;
		dut.weights.mem[318] = 8'b11111010;
		dut.weights.mem[319] = 8'b11110101;
		dut.weights.mem[320] = 8'b0;
		dut.weights.mem[321] = 8'b10;
		dut.weights.mem[322] = 8'b0;
		dut.weights.mem[323] = 8'b1111;
		dut.weights.mem[324] = 8'b1011;
		dut.weights.mem[325] = 8'b11110011;
		dut.weights.mem[326] = 8'b11110000;
		dut.weights.mem[327] = 8'b10011;
		dut.weights.mem[328] = 8'b11111001;
		dut.weights.mem[329] = 8'b110;
		dut.weights.mem[330] = 8'b1000;
		dut.weights.mem[331] = 8'b11110111;
		dut.weights.mem[332] = 8'b1000;
		dut.weights.mem[333] = 8'b11111111;
		dut.weights.mem[334] = 8'b10;
		dut.weights.mem[335] = 8'b11110001;
		dut.weights.mem[336] = 8'b11110011;
		dut.weights.mem[337] = 8'b101;
		dut.weights.mem[338] = 8'b1111;
		dut.weights.mem[339] = 8'b11;
		dut.weights.mem[340] = 8'b10001;
		dut.weights.mem[341] = 8'b101;
		dut.weights.mem[342] = 8'b11111001;
		dut.weights.mem[343] = 8'b11111110;
		dut.weights.mem[344] = 8'b11110100;
		dut.weights.mem[345] = 8'b11101011;
		dut.weights.mem[346] = 8'b11111100;
		dut.weights.mem[347] = 8'b0;
		dut.weights.mem[348] = 8'b1111;
		dut.weights.mem[349] = 8'b110;
		dut.weights.mem[350] = 8'b11110100;
		dut.weights.mem[351] = 8'b11100111;
		dut.weights.mem[352] = 8'b1001;
		dut.weights.mem[353] = 8'b1111;
		dut.weights.mem[354] = 8'b11111100;
		dut.weights.mem[355] = 8'b1000000;
		dut.weights.mem[356] = 8'b10;
		dut.weights.mem[357] = 8'b11111110;
		dut.weights.mem[358] = 8'b10001;
		dut.weights.mem[359] = 8'b1101;
		dut.weights.mem[360] = 8'b11100110;
		dut.weights.mem[361] = 8'b11;
		dut.weights.mem[362] = 8'b10111;
		dut.weights.mem[363] = 8'b101;
		dut.weights.mem[364] = 8'b11100100;
		dut.weights.mem[365] = 8'b11111000;
		dut.weights.mem[366] = 8'b11110111;
		dut.weights.mem[367] = 8'b101010;
		dut.weights.mem[368] = 8'b110;
		dut.weights.mem[369] = 8'b11111010;
		dut.weights.mem[370] = 8'b10;
		dut.weights.mem[371] = 8'b11111001;
		dut.weights.mem[372] = 8'b11110001;
		dut.weights.mem[373] = 8'b1;
		dut.weights.mem[374] = 8'b1111;
		dut.weights.mem[375] = 8'b101;
		dut.weights.mem[376] = 8'b100;
		dut.weights.mem[377] = 8'b11111010;
		dut.weights.mem[378] = 8'b11;
		dut.weights.mem[379] = 8'b101;
		dut.weights.mem[380] = 8'b1001;
		dut.weights.mem[381] = 8'b10010;
		dut.weights.mem[382] = 8'b11111000;
		dut.weights.mem[383] = 8'b110;
		dut.weights.mem[384] = 8'b10000;
		dut.weights.mem[385] = 8'b0;
		dut.weights.mem[386] = 8'b11111101;
		dut.weights.mem[387] = 8'b11110101;
		dut.weights.mem[388] = 8'b11101100;
		dut.weights.mem[389] = 8'b10001;
		dut.weights.mem[390] = 8'b111;
		dut.weights.mem[391] = 8'b11100001;
		dut.weights.mem[392] = 8'b11111010;
		dut.weights.mem[393] = 8'b100;
		dut.weights.mem[394] = 8'b1010;
		dut.weights.mem[395] = 8'b10010;
		dut.weights.mem[396] = 8'b1011;
		dut.weights.mem[397] = 8'b11110011;
		dut.weights.mem[398] = 8'b11111110;
		dut.weights.mem[399] = 8'b100;
		dut.weights.mem[400] = 8'b11110000;
		dut.weights.mem[401] = 8'b11011;
		dut.weights.mem[402] = 8'b11110110;
		dut.weights.mem[403] = 8'b1101;
		dut.weights.mem[404] = 8'b11111111;
		dut.weights.mem[405] = 8'b11111110;
		dut.weights.mem[406] = 8'b0;
		dut.weights.mem[407] = 8'b11100100;
		dut.weights.mem[408] = 8'b10110;
		dut.weights.mem[409] = 8'b111;
		dut.weights.mem[410] = 8'b11101100;
		dut.weights.mem[411] = 8'b11110011;
		dut.weights.mem[412] = 8'b11111111;
		dut.weights.mem[413] = 8'b110;
		dut.weights.mem[414] = 8'b1010;
		dut.weights.mem[415] = 8'b11111011;
		dut.weights.mem[416] = 8'b111;
		dut.weights.mem[417] = 8'b110;
		dut.weights.mem[418] = 8'b0;
		dut.weights.mem[419] = 8'b1100;
		dut.weights.mem[420] = 8'b11111100;
		dut.weights.mem[421] = 8'b101;
		dut.weights.mem[422] = 8'b11111001;
		dut.weights.mem[423] = 8'b10;
		dut.weights.mem[424] = 8'b11;
		dut.weights.mem[425] = 8'b11111000;
		dut.weights.mem[426] = 8'b111;
		dut.weights.mem[427] = 8'b11111110;
		dut.weights.mem[428] = 8'b11111101;
		dut.weights.mem[429] = 8'b1001;
		dut.weights.mem[430] = 8'b10101;
		dut.weights.mem[431] = 8'b11;
		dut.weights.mem[432] = 8'b11101001;
		dut.weights.mem[433] = 8'b11111000;
		dut.weights.mem[434] = 8'b1110;
		dut.weights.mem[435] = 8'b11100110;
		dut.weights.mem[436] = 8'b10101;
		dut.weights.mem[437] = 8'b1;
		dut.weights.mem[438] = 8'b11111101;
		dut.weights.mem[439] = 8'b1;
		dut.weights.mem[440] = 8'b101;
		dut.weights.mem[441] = 8'b11100111;
		dut.weights.mem[442] = 8'b11000;
		dut.weights.mem[443] = 8'b100010;
		dut.weights.mem[444] = 8'b0;
		dut.weights.mem[445] = 8'b0;
		dut.weights.mem[446] = 8'b1101;
		dut.weights.mem[447] = 8'b11111001;
		dut.weights.mem[448] = 8'b1000;
		dut.weights.mem[449] = 8'b11110110;
		dut.weights.mem[450] = 8'b11011100;
		dut.weights.mem[451] = 8'b11111101;
		dut.weights.mem[452] = 8'b1011;
		dut.weights.mem[453] = 8'b11101;
		dut.weights.mem[454] = 8'b101;
		dut.weights.mem[455] = 8'b100;
		dut.weights.mem[456] = 8'b11111110;
		dut.weights.mem[457] = 8'b101;
		dut.weights.mem[458] = 8'b10110;
		dut.weights.mem[459] = 8'b11110001;
		dut.weights.mem[460] = 8'b1101;
		dut.weights.mem[461] = 8'b11110111;
		dut.weights.mem[462] = 8'b1010;
		dut.weights.mem[463] = 8'b11;
		dut.weights.mem[464] = 8'b11;
		dut.weights.mem[465] = 8'b11111011;
		dut.weights.mem[466] = 8'b11111000;
		dut.weights.mem[467] = 8'b11111110;
		dut.weights.mem[468] = 8'b11111111;
		dut.weights.mem[469] = 8'b0;
		dut.weights.mem[470] = 8'b1110;
		dut.weights.mem[471] = 8'b100;
		dut.weights.mem[472] = 8'b1001;
		dut.weights.mem[473] = 8'b11111000;
		dut.weights.mem[474] = 8'b1111;
		dut.weights.mem[475] = 8'b11110100;
		dut.weights.mem[476] = 8'b11111010;
		dut.weights.mem[477] = 8'b11111111;
		dut.weights.mem[478] = 8'b11111001;
		dut.weights.mem[479] = 8'b11111000;
		dut.weights.mem[480] = 8'b11111101;
		dut.weights.mem[481] = 8'b11110011;
		dut.weights.mem[482] = 8'b1;
		dut.weights.mem[483] = 8'b11110011;
		dut.weights.mem[484] = 8'b0;
		dut.weights.mem[485] = 8'b11110011;
		dut.weights.mem[486] = 8'b10011;
		dut.weights.mem[487] = 8'b11110111;
		dut.weights.mem[488] = 8'b10001;
		dut.weights.mem[489] = 8'b11;
		dut.weights.mem[490] = 8'b11110010;
		dut.weights.mem[491] = 8'b10111;
		dut.weights.mem[492] = 8'b10100;
		dut.weights.mem[493] = 8'b11110011;
		dut.weights.mem[494] = 8'b10000;
		dut.weights.mem[495] = 8'b11111111;
		dut.weights.mem[496] = 8'b11111110;
		dut.weights.mem[497] = 8'b11111101;
		dut.weights.mem[498] = 8'b1100;
		dut.weights.mem[499] = 8'b11111111;
		dut.weights.mem[500] = 8'b11110111;
		dut.weights.mem[501] = 8'b11110000;
		dut.weights.mem[502] = 8'b1011;
		dut.weights.mem[503] = 8'b1001;
		dut.weights.mem[504] = 8'b10000;
		dut.weights.mem[505] = 8'b11111111;
		dut.weights.mem[506] = 8'b11111001;
		dut.weights.mem[507] = 8'b1000;
		dut.weights.mem[508] = 8'b11111100;
		dut.weights.mem[509] = 8'b110;
		dut.weights.mem[510] = 8'b110;
		dut.weights.mem[511] = 8'b101;
		dut.weights.mem[512] = 8'b11111101;
		dut.weights.mem[513] = 8'b1111;
		dut.weights.mem[514] = 8'b11111111;
		dut.weights.mem[515] = 8'b110;
		dut.weights.mem[516] = 8'b1010;
		dut.weights.mem[517] = 8'b11;
		dut.weights.mem[518] = 8'b11110100;
		dut.weights.mem[519] = 8'b11101111;
		dut.weights.mem[520] = 8'b11111110;
		dut.weights.mem[521] = 8'b10;
		dut.weights.mem[522] = 8'b11111101;
		dut.weights.mem[523] = 8'b111;
		dut.weights.mem[524] = 8'b11111111;
		dut.weights.mem[525] = 8'b101;
		dut.weights.mem[526] = 8'b110;
		dut.weights.mem[527] = 8'b0;
		dut.weights.mem[528] = 8'b11111101;
		dut.weights.mem[529] = 8'b11111110;
		dut.weights.mem[530] = 8'b11111101;
		dut.weights.mem[531] = 8'b11100111;
		dut.weights.mem[532] = 8'b101010;
		dut.weights.mem[533] = 8'b11111100;
		dut.weights.mem[534] = 8'b11110011;
		dut.weights.mem[535] = 8'b1001;
		dut.weights.mem[536] = 8'b11111011;
		dut.weights.mem[537] = 8'b11111000;
		dut.weights.mem[538] = 8'b111;
		dut.weights.mem[539] = 8'b11111101;
		dut.weights.mem[540] = 8'b11100010;
		dut.weights.mem[541] = 8'b11101011;
		dut.weights.mem[542] = 8'b11110010;
		dut.weights.mem[543] = 8'b100100;
		dut.weights.mem[544] = 8'b11110110;
		dut.weights.mem[545] = 8'b11110010;
		dut.weights.mem[546] = 8'b0;
		dut.weights.mem[547] = 8'b11110110;
		dut.weights.mem[548] = 8'b11111101;
		dut.weights.mem[549] = 8'b11111100;
		dut.weights.mem[550] = 8'b11111001;
		dut.weights.mem[551] = 8'b11111100;
		dut.weights.mem[552] = 8'b10110;
		dut.weights.mem[553] = 8'b1111;
		dut.weights.mem[554] = 8'b11110110;
		dut.weights.mem[555] = 8'b110;
		dut.weights.mem[556] = 8'b11111100;
		dut.weights.mem[557] = 8'b0;
		dut.weights.mem[558] = 8'b11111011;
		dut.weights.mem[559] = 8'b11111100;
		dut.weights.mem[560] = 8'b100;
		dut.weights.mem[561] = 8'b11110001;
		dut.weights.mem[562] = 8'b10000;
		dut.weights.mem[563] = 8'b11111100;
		dut.weights.mem[564] = 8'b11110010;
		dut.weights.mem[565] = 8'b10010;
		dut.weights.mem[566] = 8'b11111010;
		dut.weights.mem[567] = 8'b11111100;
		dut.weights.mem[568] = 8'b100;
		dut.weights.mem[569] = 8'b11111111;
		dut.weights.mem[570] = 8'b110;
		dut.weights.mem[571] = 8'b101;
		dut.weights.mem[572] = 8'b1001;
		dut.weights.mem[573] = 8'b11110011;
		dut.weights.mem[574] = 8'b11101011;
		dut.weights.mem[575] = 8'b0;
		dut.weights.mem[576] = 8'b1001;
		dut.weights.mem[577] = 8'b0;
		dut.weights.mem[578] = 8'b1010;
		dut.weights.mem[579] = 8'b11110000;
		dut.weights.mem[580] = 8'b1001;
		dut.weights.mem[581] = 8'b1100;
		dut.weights.mem[582] = 8'b11111111;
		dut.weights.mem[583] = 8'b11110101;
		dut.weights.mem[584] = 8'b0;
		dut.weights.mem[585] = 8'b10;
		dut.weights.mem[586] = 8'b10101;
		dut.weights.mem[587] = 8'b1010;
		dut.weights.mem[588] = 8'b11110101;
		dut.weights.mem[589] = 8'b11111101;
		dut.weights.mem[590] = 8'b1;
		dut.weights.mem[591] = 8'b11110100;
		dut.weights.mem[592] = 8'b111;
		dut.weights.mem[593] = 8'b0;
		dut.weights.mem[594] = 8'b11111100;
		dut.weights.mem[595] = 8'b10;
		dut.weights.mem[596] = 8'b1111;
		dut.weights.mem[597] = 8'b11110110;
		dut.weights.mem[598] = 8'b11111010;
		dut.weights.mem[599] = 8'b11111010;
		dut.weights.mem[600] = 8'b11111101;
		dut.weights.mem[601] = 8'b11111110;
		dut.weights.mem[602] = 8'b1010;
		dut.weights.mem[603] = 8'b11;
		dut.weights.mem[604] = 8'b11111101;
		dut.weights.mem[605] = 8'b11;
		dut.weights.mem[606] = 8'b1;
		dut.weights.mem[607] = 8'b11111010;
		dut.weights.mem[608] = 8'b11111010;
		dut.weights.mem[609] = 8'b1;
		dut.weights.mem[610] = 8'b11111000;
		dut.weights.mem[611] = 8'b11110100;
		dut.weights.mem[612] = 8'b100110;
		dut.weights.mem[613] = 8'b11101110;
		dut.weights.mem[614] = 8'b11111101;
		dut.weights.mem[615] = 8'b11;
		dut.weights.mem[616] = 8'b11101110;
		dut.weights.mem[617] = 8'b0;
		dut.weights.mem[618] = 8'b11111000;
		dut.weights.mem[619] = 8'b1;
		dut.weights.mem[620] = 8'b1110;
		dut.weights.mem[621] = 8'b101;
		dut.weights.mem[622] = 8'b111000;
		dut.weights.mem[623] = 8'b11100011;
		dut.weights.mem[624] = 8'b10;
		dut.weights.mem[625] = 8'b11111111;
		dut.weights.mem[626] = 8'b11011111;
		dut.weights.mem[627] = 8'b1000;
		dut.weights.mem[628] = 8'b11111111;
		dut.weights.mem[629] = 8'b0;
		dut.weights.mem[630] = 8'b11101011;
		dut.weights.mem[631] = 8'b11101011;
		dut.weights.mem[632] = 8'b10;
		dut.weights.mem[633] = 8'b100011;
		dut.weights.mem[634] = 8'b11111110;
		dut.weights.mem[635] = 8'b11111111;
		dut.weights.mem[636] = 8'b11100;
		dut.weights.mem[637] = 8'b11100010;
		dut.weights.mem[638] = 8'b11110100;
		dut.weights.mem[639] = 8'b11101100;
		dut.weights.mem[640] = 8'b11;
		dut.weights.mem[641] = 8'b1;
		dut.weights.mem[642] = 8'b101;
		dut.weights.mem[643] = 8'b11000;
		dut.weights.mem[644] = 8'b11111100;
		dut.weights.mem[645] = 8'b11110001;
		dut.weights.mem[646] = 8'b11111111;
		dut.weights.mem[647] = 8'b11111101;
		dut.weights.mem[648] = 8'b11110111;
		dut.weights.mem[649] = 8'b0;
		dut.weights.mem[650] = 8'b0;
		dut.weights.mem[651] = 8'b101;
		dut.weights.mem[652] = 8'b0;
		dut.weights.mem[653] = 8'b1000;
		dut.weights.mem[654] = 8'b11111011;
		dut.weights.mem[655] = 8'b11111101;
		dut.weights.mem[656] = 8'b11111100;
		dut.weights.mem[657] = 8'b11110101;
		dut.weights.mem[658] = 8'b100;
		dut.weights.mem[659] = 8'b11111010;
		dut.weights.mem[660] = 8'b1001;
		dut.weights.mem[661] = 8'b11111001;
		dut.weights.mem[662] = 8'b11110111;
		dut.weights.mem[663] = 8'b1011;
		dut.weights.mem[664] = 8'b100;
		dut.weights.mem[665] = 8'b1010;
		dut.weights.mem[666] = 8'b0;
		dut.weights.mem[667] = 8'b0;
		dut.weights.mem[668] = 8'b111;
		dut.weights.mem[669] = 8'b11111101;
		dut.weights.mem[670] = 8'b1;
		dut.weights.mem[671] = 8'b1001;
		dut.weights.mem[672] = 8'b11110111;
		dut.weights.mem[673] = 8'b110;
		dut.weights.mem[674] = 8'b10;
		dut.weights.mem[675] = 8'b11111101;
		dut.weights.mem[676] = 8'b11110011;
		dut.weights.mem[677] = 8'b11111010;
		dut.weights.mem[678] = 8'b10111;
		dut.weights.mem[679] = 8'b11110011;
		dut.weights.mem[680] = 8'b111;
		dut.weights.mem[681] = 8'b111;
		dut.weights.mem[682] = 8'b11111011;
		dut.weights.mem[683] = 8'b100;
		dut.weights.mem[684] = 8'b1001;
		dut.weights.mem[685] = 8'b111;
		dut.weights.mem[686] = 8'b11111001;
		dut.weights.mem[687] = 8'b0;
		dut.weights.mem[688] = 8'b1010;
		dut.weights.mem[689] = 8'b11111000;
		dut.weights.mem[690] = 8'b11110111;
		dut.weights.mem[691] = 8'b11111110;
		dut.weights.mem[692] = 8'b100;
		dut.weights.mem[693] = 8'b11110110;
		dut.weights.mem[694] = 8'b1000;
		dut.weights.mem[695] = 8'b110;
		dut.weights.mem[696] = 8'b11111110;
		dut.weights.mem[697] = 8'b11110010;
		dut.weights.mem[698] = 8'b1001;
		dut.weights.mem[699] = 8'b110;
		dut.weights.mem[700] = 8'b101;
		dut.weights.mem[701] = 8'b11111100;
		dut.weights.mem[702] = 8'b11011;
		dut.weights.mem[703] = 8'b11101011;
		dut.weights.mem[704] = 8'b11110100;
		dut.weights.mem[705] = 8'b1110;
		dut.weights.mem[706] = 8'b11111011;
		dut.weights.mem[707] = 8'b11110100;
		dut.weights.mem[708] = 8'b11110111;
		dut.weights.mem[709] = 8'b10;
		dut.weights.mem[710] = 8'b11111111;
		dut.weights.mem[711] = 8'b1010;
		dut.weights.mem[712] = 8'b11000;
		dut.weights.mem[713] = 8'b11111010;
		dut.weights.mem[714] = 8'b10001;
		dut.weights.mem[715] = 8'b101;
		dut.weights.mem[716] = 8'b11110001;
		dut.weights.mem[717] = 8'b11111000;
		dut.weights.mem[718] = 8'b11110001;
		dut.weights.mem[719] = 8'b110;
		dut.weights.mem[720] = 8'b11101100;
		dut.weights.mem[721] = 8'b11100110;
		dut.weights.mem[722] = 8'b11110010;
		dut.weights.mem[723] = 8'b1100;
		dut.weights.mem[724] = 8'b0;
		dut.weights.mem[725] = 8'b11111111;
		dut.weights.mem[726] = 8'b10101;
		dut.weights.mem[727] = 8'b1011;
		dut.weights.mem[728] = 8'b11111010;
		dut.weights.mem[729] = 8'b11110110;
		dut.weights.mem[730] = 8'b0;
		dut.weights.mem[731] = 8'b11101110;
		dut.weights.mem[732] = 8'b11101000;
		dut.weights.mem[733] = 8'b10100;
		dut.weights.mem[734] = 8'b1000;
		dut.weights.mem[735] = 8'b11111010;
		dut.weights.mem[736] = 8'b101011;
		dut.weights.mem[737] = 8'b101000;
		dut.weights.mem[738] = 8'b11101100;
		dut.weights.mem[739] = 8'b111;
		dut.weights.mem[740] = 8'b11100111;
		dut.weights.mem[741] = 8'b11110101;
		dut.weights.mem[742] = 8'b0;
		dut.weights.mem[743] = 8'b1110;
		dut.weights.mem[744] = 8'b11110111;
		dut.weights.mem[745] = 8'b101;
		dut.weights.mem[746] = 8'b11010;
		dut.weights.mem[747] = 8'b101110;
		dut.weights.mem[748] = 8'b11101111;
		dut.weights.mem[749] = 8'b11111100;
		dut.weights.mem[750] = 8'b11101011;
		dut.weights.mem[751] = 8'b11101011;
		dut.weights.mem[752] = 8'b10011;
		dut.weights.mem[753] = 8'b11111100;
		dut.weights.mem[754] = 8'b11100010;
		dut.weights.mem[755] = 8'b0;
		dut.weights.mem[756] = 8'b10010;
		dut.weights.mem[757] = 8'b101010;
		dut.weights.mem[758] = 8'b11110010;
		dut.weights.mem[759] = 8'b1010;
		dut.weights.mem[760] = 8'b11110110;
		dut.weights.mem[761] = 8'b11110101;
		dut.weights.mem[762] = 8'b10101;
		dut.weights.mem[763] = 8'b11111000;
		dut.weights.mem[764] = 8'b11100000;
		dut.weights.mem[765] = 8'b101;
		dut.weights.mem[766] = 8'b10101;
		dut.weights.mem[767] = 8'b100101;
		dut.weights.mem[768] = 8'b11111000;
		dut.weights.mem[769] = 8'b10011;
		dut.weights.mem[770] = 8'b11111010;
		dut.weights.mem[771] = 8'b100;
		dut.weights.mem[772] = 8'b1111;
		dut.weights.mem[773] = 8'b11110110;
		dut.weights.mem[774] = 8'b11100010;
		dut.weights.mem[775] = 8'b11110100;
		dut.weights.mem[776] = 8'b1110;
		dut.weights.mem[777] = 8'b11110;
		dut.weights.mem[778] = 8'b1011;
		dut.weights.mem[779] = 8'b101000;
		dut.weights.mem[780] = 8'b1011;
		dut.weights.mem[781] = 8'b111;
		dut.weights.mem[782] = 8'b0;
		dut.weights.mem[783] = 8'b11111101;
		dut.weights.mem[784] = 8'b11100001;
		dut.weights.mem[785] = 8'b11111101;
		dut.weights.mem[786] = 8'b11111101;
		dut.weights.mem[787] = 8'b1000;
		dut.weights.mem[788] = 8'b1100;
		dut.weights.mem[789] = 8'b101001;
		dut.weights.mem[790] = 8'b11111010;
		dut.weights.mem[791] = 8'b11111101;
		dut.weights.mem[792] = 8'b11110101;
		dut.weights.mem[793] = 8'b10001;
		dut.weights.mem[794] = 8'b11111101;
		dut.weights.mem[795] = 8'b100;
		dut.weights.mem[796] = 8'b11111110;
		dut.weights.mem[797] = 8'b0;
		dut.weights.mem[798] = 8'b11011;
		dut.weights.mem[799] = 8'b11101;
		dut.weights.mem[800] = 8'b110;
		dut.weights.mem[801] = 8'b101;
		dut.weights.mem[802] = 8'b11;
		dut.weights.mem[803] = 8'b100;
		dut.weights.mem[804] = 8'b101;
		dut.weights.mem[805] = 8'b11111100;
		dut.weights.mem[806] = 8'b11111001;
		dut.weights.mem[807] = 8'b11111100;
		dut.weights.mem[808] = 8'b11;
		dut.weights.mem[809] = 8'b11100;

		// reset values
		rst <= 1'b1;
		#PERIOD;
		rst <= 1'b0;
		#PERIOD;
		#PERIOD;
		#PERIOD;

		dut.inference_en = 1'b1;
		dut.beta = 8'b11100110;
		dut.vth = 8'b01110000;
		dut.total_timesteps = 100;

		for (i=0; i<100000; i=i+1)
		begin
			#PERIOD;
		end

		$display("0: %d", dut.output_spike_count_0);
		$display("1: %d", dut.output_spike_count_1);
		$display("2: %d", dut.output_spike_count_2);
		$display("3: %d", dut.output_spike_count_3);
		$display("4: %d", dut.output_spike_count_4);
		$display("5: %d", dut.output_spike_count_5);
		$display("6: %d", dut.output_spike_count_6);
		$display("7: %d", dut.output_spike_count_7);
		$display("8: %d", dut.output_spike_count_8);
		$display("9: %d", dut.output_spike_count_9);

		$finish;

	end

	snn dut (
		.wb_clk_i(clock),
		.wb_rst_i(rst)
	);


endmodule
`default_nettype wire
