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

module neuron #(
    parameter SIZE = 8
)(
    input [SIZE-1:0] weight,
    input [SIZE-1:0] v_mem_in,
    input [SIZE-1:0] beta,
    input function_sel,
    input [SIZE-1:0] v_th,
    output spike,
    output [SIZE-1:0] v_mem_out
);

    wire [SIZE-1:0] v_mem_decayed;
    wire [SIZE:0] v_mem_added;

    wire overflow;
    wire underflow;

    wire [SIZE-1:0] intermediate;

    wire [2*SIZE-1:0] v_mem_mult;
    
    //basic functions for decay and addition of weight
    assign v_mem_mult = (v_mem_in * beta);
    assign v_mem_decayed = v_mem_mult >> 8;
    assign v_mem_added = weight[7] ? (v_mem_in - weight[6:0]) : (v_mem_in + weight[6:0]);

    assign overflow = weight[7] ? (v_mem_in < weight[6:0]) : 0; // overflow if carry into last bit
    assign underflow = weight[7] ? (v_mem_in < weight[6:0]) : 0; // underflow if v_mem_in < weight[6:0] and we did subtraction

    //assign a spike if we pass our threshold voltage
    assign spike = v_mem_decayed > v_th ? 1 : 0;

    assign intermediate = overflow ? 8'h7F : ((function_sel ? (spike ? 0 : v_mem_decayed) : 
                                      (v_mem_added)));

    assign v_mem_out = underflow ? 8'h00 : intermediate;

endmodule

`default_nettype wire
