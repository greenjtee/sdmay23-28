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

module queue #(
    parameter MAX_SIZE = 1024
)(
    input clk,
    input rst,
    input insert,
    input read,
    input [7:0] data_i,

    output valid_o,
    output [7:0] data_o
);
    reg [9:0] head_address;
    reg [9:0] tail_address;
    reg [9:0] size;

    reg [7:0] queue_data_i;
    reg queue_insert;

    assign valid_o = |size; // if queue has any size, output is valid

    always @(posedge clk) begin
        if (rst) begin

            head_address <= 10'b00_0000_0000;
            tail_address <= 10'b00_0000_0000;
            size <= 10'b00_0000_0000;
        end else begin

            queue_data_i    <= data_i;
            queue_insert    <= ~insert;

            if (~queue_insert) begin
                size <= size + 1;
                tail_address <= tail_address + 1;
            end else if (read) begin
                size <= size - 1;
                head_address <= head_address + 1;
            end
        end
    end

    sky130_sram_1kbyte_1rw1r_8x1024_8 #(
        .VERBOSE(1)
    )
    queue_sram(
        // rw
        .clk0(clk),
        .csb0(queue_insert),
        .web0(queue_insert),
        .wmask0(1'b1),
        .addr0(tail_address),
        .din0(queue_data_i),
        .dout0(),
        // r
        .clk1(clk),
        .csb1(~read),
        .addr1(head_address),
        .dout1(data_o)
    );

endmodule

`default_nettype wire
