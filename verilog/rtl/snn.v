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

module snn #(
    parameter BITS = 32
)(
`ifdef USE_POWER_PINS
    inout vccd1,	// User area 1 1.8V supply
    inout vssd1,	// User area 1 digital ground
`endif

    // Wishbone Slave ports (WB MI A)
    input wb_clk_i,
    input wb_rst_i,
    input wbs_stb_i,
    input wbs_cyc_i,
    input wbs_we_i,
    input [3:0] wbs_sel_i,
    input [31:0] wbs_dat_i,
    input [31:0] wbs_adr_i,
    output wbs_ack_o,
    output [31:0] wbs_dat_o,

    // Logic Analyzer Signals
    input  [127:0] la_data_in,
    output [127:0] la_data_out,
    input  [127:0] la_oenb,

    // IOs
    input  [15:0] io_in,
    output [15:0] io_out,
    output [15:0] io_oeb,

    // IRQ
    output [2:0] irq
);
    wire clk;
    wire rst;

    wire [15:0] io_in;
    wire [15:0] io_out;
    wire [15:0] io_oeb;

    wire [31:0] rdata; 
    wire [31:0] wdata;
    wire [BITS-1:0] count;

    wire valid;
    wire [3:0] wstrb;
    wire [31:0] la_write;

    // WB MI A
    assign valid = wbs_cyc_i && wbs_stb_i; 
    assign wstrb = wbs_sel_i & {4{wbs_we_i}};
    assign wbs_dat_o = rdata;
    assign wdata = wbs_dat_i;

    // IO
    assign io_out = count;
    assign io_oeb = {(15){rst}};

    // IRQ
    assign irq = 3'b000;	// Unused

    assign la_write = ~la_oenb[63:32] & ~{BITS{valid}};
    assign clk = (~la_oenb[64]) ? la_data_in[64]: wb_clk_i;
    assign rst = (~la_oenb[65]) ? la_data_in[65]: wb_rst_i;

    //number of timesteps
    localparam NUM_TIMESTEPS = 100;

    //state machine parameters
    localparam LOAD_IMG     = 3'b000;
    localparam GEN_SPIKES   = 3'b001;
    localparam LOAD_SPIKE   = 3'b010;
    localparam LOAD_WEIGHT  = 3'b011;
    localparam OPSTORE      = 3'b100;
    localparam LEAK_VMEM    = 3'b101;
    localparam LEAK_FIRE    = 3'b110;
    localparam CHECK_END    = 3'b111;

    reg[2:0] ps; //present state
    wire[2:0] ns; //next state 

    //done signal
    wire inf_done;

    //intermediate registers for keeping track of neuron processing
    reg[9:0] vmem_processed_reg;            //how many output neurons have been calculated
    wire[9:0] vmem_processed;
    reg[7:0] spikes_processed_reg;          //spike number currently being processed
    wire[7:0] spikes_processed;
    reg[7:0] curr_timestep;
    wire load_timestep;

    //internal reset
    wire rst_internal;

    //load spike intermediate signlas
    wire queue_valid;                        //whether or not the data from queue is valid - controls state transition

    //output neuron specific registers
    reg[31:0] vmem [9:0];                   //registers of current vmem values
    reg[31:0] spike_num [9:0]               //how many times neuron spiked
    
    //registers for operation results
    reg[31:0] curr_vmem_reg;                //value of current membrane voltage (neuron)
    reg[31:0] curr_weight_reg;              //value of current weight being used
    reg nueron_op_reg;                      //function for neuron to carry out

    //signals used for neuron operation
    wire[31:0] curr_vmem;                   //value of current membrane voltage (neuron)
    wire[31:0] curr_weight;                 //value of current weight being used
    wire[31:0] result;                      //neuron output for integrate function
    wire nueron_op;                         //function for neuron to carry out
    wire vmem_load;                         //dictates loading of a vmem register
    wire spike_load;                        //dictates incrementing a spike count register
    wire spike_out;                         //whether or not there is a spike

    //clocked process
    always @ (posedge(clk)) begin
        if (rst or rst_internal) begin //reset
            ps                      <= LOAD_IMG;
            vmem_processed_reg      <= 0;
            spikes_processed_reg    <= 0;
            curr_timestep           <= 0;
            curr_vmem_reg           <= 0;
            curr_weight_reg         <= 0;
            nueron_op_reg           <= 0;
            int i;
            for (i = 0; i < 10; i = i + 1) begin
                vmem[i]             <= 0;
                spike_num[i]        <= 0;
            end

        end else begin //update registers
            curr_vmem_reg           <= curr_vmem;
            curr_weight_reg         <= curr_weight;
            nueron_op_reg           <= nueron_op;
            vmem_processed_reg      <= vmem_processed;
            spikes_processed_reg    <= spikes_processed;
            curr_timestep           <= curr_timestep;
            ps                      <= ns;
            if (vmem_load = 1) begin
                vmem[vmem_processed]    <=  result;
            end
            if (spike_load = 1) begin
                spike_num[vmem_processed]   <= spike_num[vmem_processed] + 1;
            end
            if (load_timestep = 1) begin
                curr_timestep   <=  curr_timestep + 1;
            end
        end
    end

    //non clocked process
    always @ (ps) begin
        //add default statements
        curr_vmem                   = curr_vmem_reg;
        curr_weight                 = curr_weight_reg;
        nueron_op                   = nueron_op_reg;
        spikes_processed            = spikes_processed_reg;
        vmem_processed              = vmem_processed_reg;
        vmem_load                   = 0;
        spike_load                  = 0;
        load_timestep               = 0;
        inf_done                    = 0;
        rst_internal                = 0;
        ns                          = ps;

        //begin case
        case(ps)
            //A transaction involves receiving a valid signal with non-zero data and replying with a done signal
            //Separate addresses for different blocks (each one has a different base addr for our unit to differentiate between)
            LOAD_IMG:
                //if valid signal high, send data to mem
                //if start signal is high, last data received and move forward to spike generation

            GEN_SPIKES:
                //for each pixel in 14x14 image
                    //load value from mem (top 8 bits id and lower 8 bits current comparison seed????)
                    //sample algorithm for generating random value
                        // data_next[4] = data[4]^data[1];
                        // data_next[3] = data[3]^data[0];
                        // data_next[2] = data[2]^data_next[4];
                        // data_next[1] = data[1]^data_next[3];
                        // data_next[0] = data[0]^data_next[2];
                    //if seed <= pixel intensity,
                        //send '1' to queue
                    //else (seed > intensity)
                        //send '0' to queue
                //move to load spike stage

            LOAD_SPIKE:
                //load next spike from queue, 
                if () begin //valid signal from queue is high and spike is high
                    ns                  = LOAD_WEIGHT;
                end else if () begin //valid signal is high and spike is low
                    spikes_processed    = spikes_processed_reg + 1;
                    ns                  = LOAD_SPIKE;
                end else begin //valid signal low
                    vmem_processed      = 0;
                    spikes_processed    = 0;
                    ns                  = LEAK_VMEM;   //all spikes processed
                end

            LOAD_WEIGHT:
                //carry out data transaction for loading weight
                if () begin //weight is loaded
                    curr_vmem   = vmem[vmem_processed_reg];   //since vmem is a register we can just load it in as we move on to the next stage
                    nueron_op   = 0;
                    ns          = OPSTORE;
                end else begin
                    ns          = LOAD_WEIGHT;
                end

            OPSTORE:
                vmem_load       = 1;
                if (vmem_processed_reg = 9) begin
                    ns          = LOAD_SPIKE;
                end else begin
                    vmem_processed  = vmem_processed_reg + 1;
                    ns              = LOAD_WEIGHT;
                end

            LEAK_VMEM:
                curr_vmem       = vmem[vmem_processed_reg];
                nueron_op       = 1;
                ns              = LEAK_FIRE;

            LEAK_FIRE:
                if (spike_out = 1) begin
                    spike_load  = 1;
                end
                if (vmem_processed_reg = 9) begin
                    ns          = CHECK_END;
                end else begin
                    vmem_processed  = vmem_processed_reg + 1;
                    ns              = LEAK_VMEM;
                end

            CHECK_END:
                if (curr_timestep = NUM_TIMESTEPS - 1) begin
                    rst_internal    <= 1;
                    inf_done        <= 1;
                end else begin
                    ps              <= GEN_SPIKES;     
                end

        endcase
    end

    //neuron module
    neuron #(
        
    ) neuron(
        .weight(),
        .v_mem_in(),
        .beta(),
        .function_sel(),
        .v_th(),
        .spike(),
        .v_mem_out()
    );

    //queue module
    queue #(

    ) queue(
        .clk(),
        .rst(),
        .insert(),
        .read(),
        .data_i(),
        .valid_o(),
        .data_o()
    );

endmodule

`default_nettype wire
