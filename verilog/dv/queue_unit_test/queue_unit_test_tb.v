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
module queue_unit_test_tb;
	parameter PERIOD = 10;

	integer i;

	reg clock;
	reg rst;
	
	reg insert_i;
	reg read_i;
    reg [7:0] data_i;

    wire valid_o;
    wire [7:0] data_o;

	initial begin 
		clock = 0;
		forever begin
		#5 clock = ~clock;
	end end

	initial
	begin
		$dumpfile("queue_unit_test.vcd");
		$dumpvars(0, queue_unit_test_tb);

		// initialize data
		insert_i = 1'b0;
		read_i = 1'b0;
		data_i = 8'h00;

		// reset values
		rst <= 1'b1;
		#PERIOD;

		// try inserting data without setting insert bit
		rst <= 1'b0;
		data_i = 8'hEC;
		#PERIOD;

		// insert data
		insert_i = 1'b1;
		data_i = 8'h01;
		#PERIOD;
		insert_i = 1'b1;
		data_i = 8'h02;
		#PERIOD;
		insert_i = 1'b1;
		data_i = 8'h03;
		#PERIOD;

		// read data out
		insert_i = 1'b0;
		read_i = 1'b1;
		#PERIOD;
		read_i = 1'b1;
		#PERIOD;
		read_i = 1'b1;
		#PERIOD;

		// insert many things
		// more than size of sram, should overwrite
		for (i = 0; i < 1000 ; i=i+1) begin
			insert_i = 1'b1;
			data_i = i;
			#PERIOD;
		end
		
		insert_i = 1'b0;

		// read many things back
		for (i = 0; i < 1000 ; i=i+1) begin
			read_i = 1'b1;
			#PERIOD;
		end

		$finish;

	end

	queue dut (
		.clk(clock),
		.rst(rst),
		.insert(insert_i),
		.read(read_i),
		.data_i(data_i),
		.valid_o(valid_o),
		.data_o(data_o)
	);

endmodule
