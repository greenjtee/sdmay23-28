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

module neuron (
    input signed [7:0] weight,
    input signed [8:0] v_mem_in,
    input [7:0] beta,
    input function_sel,
    input [7:0] v_th,
    output spike,
    output signed [8:0] v_mem_out
);

    wire signed [8:0] v_mem_decayed;
    wire signed [8:0] v_mem_added;
    wire signed [8:0] extended_weight;

    wire signed [2*8-1:0] v_mem_mult;

    assign extended_weight = {weight[7], weight};
    
    //basic functions for decay and addition of weight
    assign v_mem_mult = (v_mem_in * beta);
    assign v_mem_decayed = v_mem_in; // v_mem_mult >>> 8;
    assign v_mem_added = v_mem_in + extended_weight;

    //assign a spike if we pass our threshold voltage
    assign spike = v_mem_decayed > v_th ? 1 : 0;

    assign v_mem_out = function_sel ? (spike ? 0 : v_mem_decayed) : 
                                      (v_mem_added);

    // assign v_mem_out = underflow ? 8'h00 : intermediate;

endmodule

`default_nettype wire
