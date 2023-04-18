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

module neuron_unit_test_tb;
	// always #12.5 clock <= (clock === 1'b0);

	// reg clock;
    reg [7:0] weight_i;
    reg [7:0] v_mem_in_i;
    reg [7:0] beta_i;
    reg function_sel_i;
    reg [7:0] v_th_i;
	
    wire spike_o;
    wire [7:0] v_mem_out_o;

	initial
	begin
		$dumpfile("neuron_unit_test.vcd");
		$dumpvars(0, neuron_unit_test_tb);

		weight_i = 8'h00;
		v_mem_in_i = 8'h00;
		beta_i = 8'h00;
		function_sel_i = 8'h00;
		v_th_i = 8'h00;

		#10;


		$finish;
	end

	neuron dut (
		.weight(weight_i),
		.v_mem_in(v_mem_in_i),
		.beta(beta_i),
		.function_sel(function_sel_i),
		.v_th(v_th_i),
		.spike(spike_o),
		.v_mem_out(v_mem_out_o)
	);

endmodule
