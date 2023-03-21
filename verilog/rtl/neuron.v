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
    parameter WEIGHT_SIZE = 8,
    parameter V_MEM_SIZE = 8,
    parameter B_SIZE = 8
)(
    input [WEIGHT_SIZE-1:0] weight,
    input [V_MEM_SIZE-1:0] v_mem_in,
    input [B_SIZE-1:0] beta,
    input function_sel,
    input [V_MEM_SIZE-1:0] v_th,
    output spike,
    output [V_MEM_SIZE-1:0] v_mem_out
);

    wire [V_MEM_SIZE-1:0] v_mem_decayed;
    wire [V_MEM_SIZE-1:0] v_mem_added;
    
    //basic functions for decay and addition of weight
    assign v_mem_decayed = v_mem_in * beta;
    assign v_mem_added = v_mem_in + weight;

    //assign a spike if we pass our threshold voltage
    assign spike = v_mem_decayed > v_th ? 1 : 0;

    assign v_mem_out = function_sel ? (spike ? 0 : v_mem_decayed) : 
                                      (v_mem_added);
endmodule

`default_nettype wire
