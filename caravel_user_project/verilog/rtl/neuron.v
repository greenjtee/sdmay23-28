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
