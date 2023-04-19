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

module rand_gen #(
)(
    input clk,
    input rst,
    input [7:0] seed_i,
    input set_seed_i,
    output [7:0] rand_o
);
    reg [15:0] data;
    wire [15:0] data_next;

    assign rand_o = data[7:0];

    assign data_next[0] = data[0] ^ data[5] ^ data[7] ^ data[8] ^ data[9] ^ data[11] ^ data[12];
    assign data_next[1] = data[1] ^ data[6] ^ data[8] ^ data[9] ^ data[10] ^ data[12] ^ data[13];
    assign data_next[2] = data[2] ^ data[7] ^ data[9] ^ data[10] ^ data[11] ^ data[13] ^ data[14];
    assign data_next[3] = data[3] ^ data[8] ^ data[10] ^ data[11] ^ data[12] ^ data[14] ^ data[15];
    assign data_next[4] = data[0] ^ data[9] ^ data[11] ^ data[12];
    assign data_next[5] = data[1] ^ data[10] ^ data[12] ^ data[13];
    assign data_next[6] = data[2] ^ data[11] ^ data[13] ^ data[14];
    assign data_next[7] = data[3] ^ data[12] ^ data[14] ^ data[15];
    assign data_next[8] = data[0];
    assign data_next[9] = data[1];
    assign data_next[10] = data[2];
    assign data_next[11] = data[3];
    assign data_next[12] = data[4];
    assign data_next[13] = data[5];
    assign data_next[14] = data[6];
    assign data_next[15] = data[7];

    always @(posedge clk)
    begin
        if (rst) begin
            data <= 8'hFF;
        end
        else begin
            if (set_seed_i) begin
                data <= seed_i;
            end
            else begin
                data <= data_next;
            end
        end
    end



endmodule

`default_nettype wire
