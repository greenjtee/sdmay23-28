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

module snn_unit_test_tb;
	parameter PERIOD = 10;

	reg clock;
	reg rst;
	integer i;

	initial
	begin 
		clock = 0;
		forever
		begin
		#5 clock = ~clock;
		end
	end

	initial
	begin
		$dumpfile("snn_unit_test.vcd");
		$dumpvars(0, snn_unit_test_tb);

		// reset values
		rst <= 1'b1;
		#PERIOD;

		rst <= 1'b0;
		#PERIOD;

		// enable inference
		dut.inference_en = 1;

		// setup sram with values

		for (i = 0; i < 10000; i=i+1)
		begin
			#PERIOD;
		end

		$finish;
	end

	snn dut (
		.wb_clk_i(clock),
		.wb_rst_i(rst),
		.wbs_stb_i(),
		.wbs_cyc_i(),
		.wbs_we_i(),
		.wbs_sel_i(),
		.wbs_dat_i(),
		.wbs_adr_i(),
		.wbs_ack_o(),
		.wbs_dat_o(),

    	.la_data_in(),
    	.la_data_out(),
    	.la_oenb(),

        .io_in(),
        .io_out(),
        .io_oeb(),

	    .irq()
	);

endmodule
