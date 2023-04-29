`default_nettype none

module snn #(
    parameter WEIGHTS_BASE = 32'h3000_0000,
    parameter DIM_X = 9,
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

    reg irq;

    // state machine parameters
    parameter WAIT          = 4'h0;
    parameter LOAD_PIXEL    = 4'h1;
    parameter LOAD_PIXEL_2  = 4'h2;
    parameter LOAD_PIXEL_3  = 4'h3;
    parameter LOAD_PIXEL_4  = 4'h4;
    parameter GEN_SPIKE     = 4'h5;
    parameter LOAD_SPIKE    = 4'h6;
    parameter SAVE_NEURON_INDEX = 4'h7;
    parameter LOAD_WEIGHT   = 4'h8;
    parameter LOAD_WEIGHT_2 = 4'h9;
    parameter LOAD_VMEM     = 4'hA;
    parameter OPSTORE       = 4'hB;
    parameter LEAK_VMEM     = 4'hC;
    parameter LEAK_FIRE     = 4'hD;
    parameter CHECK_END     = 4'hE;

    // neuron operations
    parameter NEURON_INTEGRATE  = 1'b0;
    parameter NEURON_LEAK_FIRE  = 1'b1;

    // state machine signals
    reg [3:0] ps;                                // present state
    reg [3:0] ns;                               // next state 

    reg [9:0] curr_timestep;
    reg [7:0] output_index;


    reg vmem_store;
    reg spike_store;

    reg increment_timestep;
    reg increment_output_index;
    reg increment_pixel_index;
    reg reset_pixel_index;
    reg reset_output_index;
    reg trigger_interrupt;
    reg insert_spike;

    // image sram signals
    reg image_en;
    reg [9:0] image_addr_i;
    reg [9:0] image_addr_o;
    reg [7:0] image_data_i;
    wire [7:0] image_data_o;

    reg [7:0] pixel_index;
    reg image_read_o;

    wire [7:0] pixel;
    reg [9:0] last_pixel;

    // weights sram signals
    reg weights_en;
    reg [9:0] weights_addr_i;
    reg [7:0] weights_data_i;

    reg [9:0] weights_addr_o;
    reg weights_read_o;
    wire [7:0] weights_data_o;
    reg [7:0] weights_data_reg;

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
    reg [7:0] neuron_vmem_reg;
    wire [7:0] neuron_vmem_o;

    reg neuron_op;

    // control register signals
    reg inference_en;
    reg [7:0] beta;
    reg [7:0] vth;
    reg [9:0] total_timesteps;

    reg inference_done;

    // output neuron specific registers
    reg [8:0] output_vmem [OUTPUTS-1:0];
    reg [7:0] output_spike [OUTPUTS-1:0];

    reg [3:0] output_vmem_index;
    reg [3:0] output_spike_index;
    reg [9:0] spiking_neuron_index;

    reg output_weight_store;

    // wires for easy viewing
    wire [7:0] output_spike_count_0 = output_spike[0];
    wire [7:0] output_spike_count_1 = output_spike[1];
    wire [7:0] output_spike_count_2 = output_spike[2];
    wire [7:0] output_spike_count_3 = output_spike[3];
    wire [7:0] output_spike_count_4 = output_spike[4];
    wire [7:0] output_spike_count_5 = output_spike[5];
    wire [7:0] output_spike_count_6 = output_spike[6];
    wire [7:0] output_spike_count_7 = output_spike[7];
    wire [7:0] output_spike_count_8 = output_spike[8];
    wire [7:0] output_spike_count_9 = output_spike[9];

    // wires for easy viewing
    wire [8:0] output_vmem_0 = output_vmem[0];
    wire [8:0] output_vmem_1 = output_vmem[1];
    wire [8:0] output_vmem_2 = output_vmem[2];
    wire [8:0] output_vmem_3 = output_vmem[3];
    wire [8:0] output_vmem_4 = output_vmem[4];
    wire [8:0] output_vmem_5 = output_vmem[5];
    wire [8:0] output_vmem_6 = output_vmem[6];
    wire [8:0] output_vmem_7 = output_vmem[7];
    wire [8:0] output_vmem_8 = output_vmem[8];
    wire [8:0] output_vmem_9 = output_vmem[9];
    
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
            inference_en <= 0;
            beta <= 0;
            vth <= 0;
            total_timesteps = DEFAULT_TIMESTEPS;

            wbs_dat_o <= 0;

        end else begin
            wbs_ack_o <= 1'b0;
            image_en <= 1'b0;
            weights_en <= 1'b0;

            wbs_dat_o <= 0;

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

                weights_addr_i <= (wbs_adr_i - 32'h3000_1000) >> 2;
                weights_data_i <= wbs_dat_i[7:0];
                weights_en <= 1;
            end

            // output data - writes to ps
            if(!wbs_we_i && valid && !wbs_ack_o && (wbs_adr_i >= 32'h3000_3000 && wbs_adr_i < 32'h3000_4000)) begin
                wbs_ack_o <= 1'b1;

                wbs_dat_o <= { 24'b0, output_spike[wbs_adr_i - 32'h3000_3000]};
            end

            // control signals - reads from ps
            if(wbs_we_i && valid && !wbs_ack_o && wbs_adr_i == 32'h3000_4000) begin
                wbs_ack_o <= 1'b1;

                inference_en <= wbs_dat_i[28];
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

            pixel_index = 0;
            last_pixel = 0;

            vmem_store = 0;
            spike_store = 0;

            increment_timestep = 0;
            increment_output_index = 0;
            increment_pixel_index = 0;

            reset_output_index = 0;
            reset_pixel_index = 0;

            // image sram signals
            image_addr_o = 0;

            // weights sram signals
            weights_addr_o = 0;

            // rng signals
            seed = 0;
            set_seed = 0;

            // queue signals
            queue_insert = 0;
            queue_read = 0;

            // current value signals
            curr_weight = 0;
            curr_vmem = 0;

            // neuron module signals
            neuron_op = 0;

            // control register signals
            inference_done = 0;

            // neuron vmem
            neuron_vmem_reg = 0;

            output_weight_store = 0;

            // output neuron specific registers
            for (i = 0; i < OUTPUTS; i = i + 1)
            begin
                output_vmem[i]  <= 0;
                output_spike[i] <= 0;
            end

            //irq
            irq = 3'b000;

        end
        else
        begin // update registers
            // rand register updates
            set_seed            <= set_seed;
            seed                <= seed;

            last_pixel <= pixel;
            neuron_vmem_reg <= neuron_vmem_o;
            
            // queue register updates
            ps                      <= ns;

            // enable signal for loading vmem register
            if (vmem_store)
            begin
                output_vmem[output_vmem_index] <= neuron_vmem_o;
            end
            
            // enable signal for loading spike register
            if (spike_store)
            begin
                output_spike[output_spike_index] <= output_spike[output_spike_index] + 1;
            end
            
            // enable signal for incrementing timestep
            if (increment_timestep)
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

            if (trigger_interrupt)
            begin
                irq = 3'b001;
            end

            if (output_weight_store)
            begin
                weights_data_reg = weights_data_o;
            end

            // if (insert_spike)
            // begin
            //     queue_insert = 1;
            //     queue_data_i = pixel_index;
            // end
        end
    end

    //non clocked process
    always @ (*)
    begin
        // reset_pixel_index = 0;

        // reset_output_index = 0;

        neuron_op = 0;

        spike_store = 0;

        increment_output_index = 0;

        increment_output_index = 0;

        increment_pixel_index = 0;

        increment_timestep = 0;

        trigger_interrupt = 0;

        insert_spike = 0;

        queue_read = 0;

        image_read_o = 0;

        vmem_store = 0;
        output_weight_store = 0;

        weights_read_o = 0;

        // state machine
        ns              = ps;

        case(ps)
            // wait for inference start bit to be set, this allows the user to load image and weight data, control registers, etc
            WAIT: // 0
            begin
                ns = WAIT;
                if (inference_en)
                begin
                    ns = LOAD_PIXEL;
                end
            end

            LOAD_PIXEL: // 1
            begin
                queue_insert = 0;
                pixel_index = image_addr_o;
                image_read_o = 1;

                ns = LOAD_PIXEL_2;
            end

            LOAD_PIXEL_2:
            begin
                // image_read_o = 1;
                // increment_pixel_index = 1;
                increment_pixel_index = 1;
                ns = GEN_SPIKE;
            end

            // LOAD_PIXEL_3:
            // begin
            //     image_read_o = 1;
            //     ns = GEN_SPIKE;
            // end
            
            // LOAD_PIXEL_4:
            // begin
            //     image_read_o = 1;
            //     ns = GEN_SPIKE;
            // end

            // if next random val is greater than pixel val, generate a spike
            // if we are at the last pixel, move to load spike state
            // otherwise, go to next pixel
            GEN_SPIKE: // 5
            begin
                ns = LOAD_PIXEL;

                if (rand_val < image_data_o) // spike generated, insert in queue
                begin
                    queue_insert = 1;
                    queue_data_i = pixel_index;
                end

                // done generating spikes
                if (pixel_index == NUM_PIXELS)
                begin
                    // reset_pixel_index = 1;
                    // pixel_index = 0;
                    ns              = LOAD_SPIKE;
                end
            end

            // get neuron that spiked out of queue
            LOAD_SPIKE: // 6
            begin
                if (queue_valid)                                            // queue has more spikes, read next one
                begin 
                    queue_read  = 1;
                    // weights_read_o = 1;

                    ns          = SAVE_NEURON_INDEX;
                end
                else                                                            // queue has no more spikes, go to next stage
                begin
                    ns          = LEAK_VMEM;                                    //all spikes processed
                end
            end

            SAVE_NEURON_INDEX: // 7
            begin
                spiking_neuron_index = queue_data_o;

                ns = LOAD_WEIGHT;
            end

            // load next weight (0 - 9) for the spiking neuron
            LOAD_WEIGHT: // 8
            begin

                weights_read_o = 1;
                weights_addr_o = (spiking_neuron_index * 10) + output_index;
                curr_vmem = output_vmem[output_index];
                output_vmem_index = output_index;

                ns = LOAD_WEIGHT_2;
            end

            LOAD_WEIGHT_2: // 9
            begin
                output_weight_store = 1;
                increment_output_index = 1;
                neuron_op = NEURON_INTEGRATE;

                ns = OPSTORE;
            end

            // add next weight to output neuron vmem
            OPSTORE: // B
            begin
                vmem_store = 1;

                ns = LOAD_WEIGHT;

                if (output_index == OUTPUTS)                                            // if we have processed all output neurons, go to next stage
                begin
                    output_index = 0;
                    ns                 = LOAD_SPIKE;
                end
            end

            // get next output vmem from registers
            LEAK_VMEM: // C
            begin
                curr_vmem   = output_vmem[output_index];
                output_spike_index = output_index;
                neuron_op   = NEURON_LEAK_FIRE;
                ns          = LEAK_FIRE;
            end

            // leak the vmem and fire if necessary
            LEAK_FIRE: // D
            begin
                if (neuron_spike_o)
                begin
                    spike_store = 1;
                end

                if (output_index == OUTPUTS)
                begin
                    increment_timestep = 1;
                    ns = CHECK_END;
                end
                else
                begin
                    increment_output_index = 1;
                    ns              = LEAK_VMEM;
                end
            end

            // if we have done all timesteps, we are done
            CHECK_END: // E
            begin
                if (curr_timestep == total_timesteps)
                begin
                    inference_en = 0;
                    trigger_interrupt = 1;
                    ns = WAIT;
                end
                else
                begin
                    image_addr_o = 0;
                    output_index = 0;
                    ns = LOAD_PIXEL;
                end
            end
        endcase
    end

    //neuron module
    neuron #() neuron(
        .weight(weights_data_reg),
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
        .data_i(queue_data_i),
        .read(queue_read),
        .valid_o(queue_valid),
        .data_o(queue_data_o)
    );

    //image storage
    sky130_sram_1kbyte_1rw1r_8x1024_8 #(
        .VERBOSE(1)
    ) image(
        // rw
        .clk0(clk),
        .csb0(~image_en),
        .web0(~image_en),
        .wmask0(1'b1),
        .addr0(image_addr_i),
        .din0(image_data_i),
        .dout0(),
        // r
        .clk1(clk),
        .csb1(~image_read_o),
        .addr1(image_addr_o),
        .dout1(image_data_o)
    );

    //weight storage
    sky130_sram_1kbyte_1rw1r_8x1024_8 #(
        .VERBOSE(1)
    ) weights(
        // rw
        .clk0(clk),
        .csb0(~weights_en),
        .web0(~weights_en),
        .wmask0(1'b1),
        .addr0(weights_addr_i),
        .din0(weights_data_i),
        .dout0(),
        // r
        .clk1(clk),
        .csb1(~weights_read_o),
        .addr1(weights_addr_o),
        .dout1(weights_data_o)
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
