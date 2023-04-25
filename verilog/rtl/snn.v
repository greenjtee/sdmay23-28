`default_nettype none

module snn #(
    parameter WEIGHTS_BASE = 32'h3000_0000,
    parameter DIM_X = 14,
    parameter DIM_Y = DIM_X,
    parameter NUM_PIXELS = DIM_X * DIM_Y,
    parameter OUTPUTS = 10,
    parameter WEIGHTS = NUM_PIXELS*OUTPUTS,
    parameter HALF_WEIGHTS = WEIGHTS >> 1,
    parameter HALF_PIXELS = NUM_PIXELS >> 1,
    parameter DEFAULT_TIMESTEPS = 100
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
    assign clk = wb_clk_i;

    wire rst;
    assign rst = wb_rst_i;

    // state machine parameters
    parameter WAIT          = 3'b000;
    parameter GEN_SPIKES    = 3'b001;
    parameter LOAD_SPIKE    = 3'b010;
    parameter LOAD_WEIGHT   = 3'b011;
    parameter OPSTORE       = 3'b100;
    parameter LEAK_VMEM     = 3'b101;
    parameter LEAK_FIRE     = 3'b110;
    parameter CHECK_END     = 3'b111;

    // neuron operations
    parameter NEURON_INTEGRATE  = 1'b0;
    parameter NEURON_LEAK_FIRE  = 1'b1;

    // state machine signals
    reg [2:0] ps;                                // present state
    reg [2:0] ns;                               // next state 

    reg [9:0] curr_timestep;
    reg [7:0] output_index;

    reg vmem_load;
    reg spike_load;
    reg timestep_load;

    reg increment_timestep;
    reg increment_output_index;
    reg increment_pixel_index;
    reg reset_pixel_index;
    reg reset_output_index;

    // image sram signals
    reg image_en;
    reg [9:0] image_addr_i;
    reg [7:0] image_data_i;

    reg [9:0] image_addr_o;
    wire [7:0] image_data_o;

    // weights 0 sram signals
    reg weights0_en;
    reg [9:0] weights0_addr_i;
    reg [7:0] weights0_data_i;

    reg [9:0] weights0_addr_o;
    wire [7:0] weights0_data_o;

    // weights 1 sram signals
    reg weights1_en;
    reg [9:0] weights1_addr_i;
    reg [7:0] weights1_data_i;

    reg [9:0] weights1_addr_o;
    wire [7:0] weights1_data_o;

    // rng signals
    reg [7:0] seed;
    reg set_seed;

    wire [7:0] rand_val;

    // queue signals
    reg queue_insert;
    reg queue_read;
    reg [7:0] queue_data_i;

    wire queue_valid;
    wire [7:0] queue_data_o;

    // current value signals
    reg [7:0] curr_weight;
    reg [7:0] curr_vmem;

    // neuron module signals
    wire neuron_spike_o;
    wire [7:0] neuron_vmem_o;

    reg neuron_op;

    // control register signals
    reg inference_en;
    reg [7:0] beta;
    reg [7:0] vth;
    reg [9:0] total_timesteps;

    reg inference_done;

    // output neuron specific registers
    reg [7:0] output_vmem [OUTPUTS-1:0];
    reg [7:0] output_spike [OUTPUTS-1:0];
    
    integer i;

    // wishbone ----------------------------------------------------------------------------------------------------

    wire [3:0] wstrb;
    reg wbs_ack_o;
    reg wbs_dat_o;

    wire valid;
    assign valid = wbs_cyc_i && wbs_stb_i;

    // writes
    always @(posedge clk) begin
        if (rst) begin
            wbs_ack_o <= 0;
        end else begin
            wbs_ack_o <= 1'b0;
            image_en <= 1'b0;

            weights0_en <= 1'b0;
            weights1_en <= 1'b0;

            // image data - reads from ps
            if(wbs_we_i && valid && !wbs_ack_o && (wbs_adr_i >= 32'h3000_0000 && wbs_adr_i < 32'h3000_1000)) begin
                wbs_ack_o <= 1'b1;

                image_addr_i <= (wbs_adr_i - 32'h3000_0000) >> 2;
                image_data_i <= wbs_dat_i[7:0];
                image_en <= 1'b1;
            end

            // weight data - reads from ps
            if(wbs_we_i && valid && !wbs_ack_o && (wbs_adr_i >= 32'h3000_1000 && wbs_adr_i < 32'h3000_2000)) begin
                wbs_ack_o <= 1'b1;

                weights0_addr_i <= (wbs_adr_i - 32'h3000_1000) >> 2;
                weights0_data_i <= wbs_dat_i[7:0];
                weights0_en <= 1'b1;
            end
            if(wbs_we_i && valid && !wbs_ack_o && (wbs_adr_i >= 32'h3000_2000 && wbs_adr_i < 32'h3000_3000)) begin
                wbs_ack_o <= 1'b1;

                weights1_addr_i <= (wbs_adr_i - 32'h3000_2000) >> 2;
                weights1_data_i <= wbs_dat_i[7:0];
                weights1_en <= 1'b1;
            end

            // output data - writes to ps
            if(!wbs_we_i && valid && !wbs_ack_o && (wbs_adr_i >= 32'h3000_3000 && wbs_adr_i < 32'h3000_4000)) begin
                wbs_ack_o <= 1'b1;

                wbs_dat_o <= { 24'b0, output_spike[wbs_adr_i - 32'h3000_3000]};
            end

            // control signals - reads from ps
            if(wbs_we_i && valid && !wbs_ack_o && wbs_adr_i == 32'h3000_4000) begin
                wbs_ack_o <= 1'b1;

                inference_en <= wbs_dat_i[29];
                total_timesteps <= wbs_dat_i[25:16];
                vth <= wbs_dat_i[15:8];
                beta <= wbs_dat_i[7:0];
            end
            // control signals - writes to ps
            if(!wbs_we_i && valid && !wbs_ack_o && wbs_adr_i == 32'h3000_4000) begin
                wbs_ack_o <= 1'b1;

                wbs_dat_o <= {3'b0, inference_en, 2'b0, total_timesteps, vth, beta};
            end
        end
    end

    // wishbone end ----------------------------------------------------------------------------------------------------

    // clocked process
    always @ (posedge(clk))
    begin
        if (rst)
        begin
            // state machine signals
            ps = WAIT;
            ns = WAIT;

            curr_timestep = 0;
            output_index = 0;

            vmem_load = 0;
            spike_load = 0;
            timestep_load = 0;

            increment_timestep = 0;
            increment_output_index = 0;
            increment_pixel_index = 0;
            reset_pixel_index = 0;
            reset_output_index = 0;

            // image sram signals
            image_en = 0;
            image_addr_i = 0;
            image_data_i = 0;
            image_addr_o = 0;

            // weights 0 sram signals
            weights0_en = 0;
            weights0_addr_i = 0;
            weights0_data_i = 0;
            weights0_addr_o = 0;

            // weights 1 sram signals
            weights1_en = 0;
            weights1_addr_i = 0;
            weights1_data_i = 0;
            weights1_addr_o = 0;

            // rng signals
            seed = 0;
            set_seed = 0;

            // queue signals
            queue_insert = 0;
            queue_read = 0;
            queue_data_i = 0;

            // current value signals
            curr_weight = 0;
            curr_vmem = 0;

            // neuron module signals
            neuron_op = 0;

            // control register signals
            inference_en = 0;
            beta = 0;
            vth = 0;
            total_timesteps = DEFAULT_TIMESTEPS;

            inference_done = 0;

            // output neuron specific registers
            for (i = 0; i < OUTPUTS; i = i + 1)
            begin
                output_vmem[i]  <= 0;
                output_spike[i] <= 0;
            end

        end
        else
        begin // update registers

            // rand register updates
            set_seed            <= set_seed;
            seed                <= seed;

            // queue register updates
            ps                      <= ns;
            
            // enable signal for loading vmem register
            if (vmem_load)
            begin
                output_vmem[output_index] <= neuron_vmem_o;
            end
            
            // enable signal for loading spike register
            if (spike_load)
            begin
                output_spike[output_index] <= output_spike[output_index] + 1;
            end
            
            // enable signal for incrementing timestep
            if (timestep_load)
            begin
                curr_timestep <= curr_timestep + 1;
            end
            
            if (increment_output_index)
            begin
                output_index = output_index + 1;
            end

            if (increment_pixel_index)
            begin
                image_addr_o = image_addr_o + 1;
            end

            if (reset_pixel_index)
            begin
                image_addr_o = 0;
            end

            if (reset_output_index)
            begin
                output_index = 0;
            end
        end
    end

    //non clocked process
    always @ (*)
    begin
        reset_pixel_index = 0;
        reset_output_index = 0;
        increment_output_index = 0;
        increment_output_index = 0;

        // rand registers
        set_seed        = 0;
        seed            = 0;

        // queue registers
        queue_insert    = 0;
        queue_read      = 0;
        queue_data_i    = 0;

        // weights
        weights0_en = 0;
        weights1_en = 0;

        // state machine
        ns              = ps;

        case(ps)
            // wait for inference start bit to be set, this allows the user to load image and weight data, control registers, etc
            WAIT:
            begin
                ns = WAIT;
                if (inference_en)
                begin
                    ns = GEN_SPIKES;
                    increment_pixel_index = 1;
                end
            end

            // if next random val is greater than pixel val, generate a spike
            // if we are at the last pixel, move to load spike state
            // otherwise, go to next pixel
            GEN_SPIKES:
            begin
                // spike generated, insert in queue
                if (rand_val < image_data_o)
                begin
                    queue_insert = 1;
                    queue_data_i = image_addr_o;
                end

                increment_pixel_index = 1;
                
                // done generating spikes
                if (image_addr_o == NUM_PIXELS - 1)
                begin
                    reset_pixel_index = 1;
                    ns              = LOAD_SPIKE;
                end
            end

            // get neuron that spiked out of queue
            LOAD_SPIKE:
            begin
                if (queue_valid)                                            // queue has more spikes, read next one
                begin 
                    queue_read  = 1;
                    ns          = LOAD_WEIGHT;
                end
                else                                                            // queue has no more spikes, go to next stage
                begin
                    ns          = LEAK_VMEM;                                    //all spikes processed
                end
            end

            // load next weight (0 - 9) for the spiking neuron
            LOAD_WEIGHT:
            begin
                if (queue_data_o < HALF_PIXELS) // desired weight value = input spike * (output spikes) + output_spike_index, distributed over two sram
                begin
                    weights0_en = 1;
                    weights0_addr_o = (queue_data_o * 10) + output_index;
                end
                else
                begin
                    weights1_en = 1;
                    weights1_addr_o = ((queue_data_o - HALF_PIXELS) * 10) + output_index;   
                end

                curr_vmem           = output_vmem[output_index];                            // since vmem is a register we can just load it in as we move on to the next stage
                neuron_op           = NEURON_INTEGRATE;    
                ns                  = OPSTORE;
            end

            // add next weight to output neuron vmem
            OPSTORE:
            begin
                if (output_index == OUTPUTS - 1)                                            // if we have processed all output neurons, go to next stage
                begin
                    reset_output_index = 1;
                    ns                 = LOAD_SPIKE;
                end
                else                                                                        // otherwise, move to the next vmem
                begin
                    increment_output_index  = 1;
                    ns                      = LOAD_WEIGHT;
                end
            end

            // get next output vmem from registers
            LEAK_VMEM:
            begin
                curr_vmem   = output_vmem[output_index];
                neuron_op   = NEURON_LEAK_FIRE;
                ns          = LEAK_FIRE;
            end

            // leak the vmem and fire if necessary
            LEAK_FIRE:
            begin
                if (neuron_spike_o)
                begin
                    spike_load = 1;
                end

                if (output_index == OUTPUTS - 1)
                begin
                    ns = CHECK_END;
                end
                else
                begin
                    increment_output_index = 1;
                    ns              = LEAK_VMEM;
                end
            end

            // if we have done all timesteps, we are done
            CHECK_END:
            begin
                if (curr_timestep == total_timesteps) begin
                    inference_done        = 1;
                end else begin
                    ps              = GEN_SPIKES;     
                end
            end
        endcase
    end

    //neuron module
    neuron #() neuron(
        .weight(curr_weight),
        .v_mem_in(curr_vmem),
        .beta(beta),
        .function_sel(neuron_op),
        .v_th(vth),
        .spike(neuron_spike_o),
        .v_mem_out(neuron_vmem_o)
    );

    //queue module
    queue #() queue(
        .clk(clk),
        .rst(rst),
        .insert(queue_insert),
        .read(queue_read),
        .data_i(queue_data_i),
        .valid_o(queue_valid),
        .data_o(queue_data_o)
    );

    //image storage
    sky130_sram_1kbyte_1rw1r_8x1024_8 #(
        .VERBOSE(1)
    ) image(
        // rw
        .clk0(clk),
        .csb0(image_en),
        .web0(image_en),
        .wmask0(1'b1),
        .addr0(image_addr_i),
        .din0(image_data_i),
        .dout0(),
        // r
        .clk1(clk),
        .csb1(1'b1),
        .addr1(image_addr_o),
        .dout1(image_data_o)
    );

    //weight storage
    sky130_sram_1kbyte_1rw1r_8x1024_8 #(
        .VERBOSE(1)
    ) weights_0(
        // rw
        .clk0(clk),
        .csb0(weights0_en),
        .web0(weights0_en),
        .wmask0(1'b1),
        .addr0(weights0_addr_i),
        .din0(weights0_data_i),
        .dout0(),
        // r
        .clk1(clk),
        .csb1(1'b1),
        .addr1(weights0_addr_o),
        .dout1(weights0_data_o)
    );

    sky130_sram_1kbyte_1rw1r_8x1024_8 #(
        .VERBOSE(1)
    ) weights_1(
        // rw
        .clk0(clk),
        .csb0(weights1_en),
        .web0(weights1_en),
        .wmask0(1'b1),
        .addr0(weights1_addr_i),
        .din0(weights1_data_i),
        .dout0(),
        // r
        .clk1(clk),
        .csb1(1'b1),
        .addr1(weights1_addr_o),
        .dout1(weights1_data_o)
    );

    // random spike rate encoding generator
    rand_gen #(

    ) spike_gen_rand(
        .clk(clk),
        .rst(rst),
        .seed_i(seed),
        .set_seed_i(set_seed),
        .rand_o(rand_val)
    );

endmodule

`default_nettype wire
