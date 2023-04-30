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
module rand_gen_unit_test_tb;
	parameter PERIOD = 10;

    reg [7:0] seed;
    reg set_seed;
    reg clk;
    reg rst;

    wire [7:0] data;
    // wire bit;

    integer i;

	initial begin 
		clk = 0;
		forever begin
		#5 clk = ~clk;
	end end

	initial
	begin
		$dumpfile("rand_gen_unit_test.vcd");
		$dumpvars(0, rand_gen_unit_test_tb);

        rst <= 1'b0;
        clk <= 1'b0;
        set_seed <= 1'b0;
        seed <= 8'h00;

        rst <= 1'b1;
        #PERIOD;

        // set_seed <= 1'b1;
        // seed <= 8'hA3;
        rst <= 1'b0;
        // #PERIOD;

        set_seed <= 1'b0;
        for (i = 0; i < 65536; i = i + 1)
        begin
            #PERIOD;
        end

        $finish;
    end

    rand_gen dut (
        .clk(clk),
        .rst(rst),
        .seed_i(seed),
        .set_seed_i(set_seed),
        .rand_o(data)
    );

endmodule