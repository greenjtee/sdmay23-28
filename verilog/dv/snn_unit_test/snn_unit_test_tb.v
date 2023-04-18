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

	initial begin 
		clock = 0;
		forever begin
		#5 clock = ~clock;
	end end

	initial
	begin
		$dumpfile("snn_unit_test.vcd");
		$dumpvars(0, snn_unit_test_tb);

		// reset values
		rst <= 1'b1;
		#PERIOD;

		rst <= 1'b0;
		#PERIOD;

		$stop;

	end

	snn dut (
	);

endmodule
