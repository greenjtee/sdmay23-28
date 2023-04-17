`default_nettype none

module snn #(
    parameter BITS = 32,
    parameter WEIGHTS_BASE = 32'h3000_0000, // address of sram for weights
    parameter WEIGHTS = 14*14 // number of weights
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
    wire [15:0] io_oeb;

    // reg [31:0] rdata; 
    // reg [31:0] wdata;

    // wire [31:0] la_write;

    reg [7:0] sram_weight_in;
    reg [9:0] sram_weight_neuron_id_in;

    wire sram_weight_csb1_in;
    wire [7:0] vmem_in;
    reg sram_we;

    reg [7:0] beta_reg;
    reg function_sel_reg;
    reg [7:0] v_th_reg;

    wire neuron_spike_out;

    wire [7:0] neuron_v_mem_out;
    reg [7:0] neuron_weight_in;
    wire [7:0] sram_weight_out;
    wire [7:0] neuron_weight_out;

    wire [9:0] neuron_id_current;

    // IO
    assign io_oeb = {15{rst}};

    // assign la_write = ~la_oenb[63:32] & ~{BITS{valid}};
    assign clk = wb_clk_i;
    assign rst = wb_rst_i;

    // assign sram_weight_csb1_in = la_data_in[];
    assign vmem_in = la_data_in[31:26];
    assign neuron_id_current = la_data_in[25:16];

    assign neuron_v_mem_out = la_data_out[47:40];
    assign neuron_spike_out = la_data_out[48];
    assign neuron_weight_out = la_data_out[71:64];

    wire [31:0] rdata; 
    wire [31:0] wdata;

    wire valid;
    wire [3:0] wstrb;
    reg wbs_ack_o;

    assign valid = wbs_cyc_i && wbs_stb_i; 
    assign wstrb = wbs_sel_i & {4{wbs_we_i}};
    assign wbs_dat_o = rdata;
    assign wdata = wbs_dat_i;
    // assign wbs_ack_o = ready;

    always @(posedge clk)
    begin
        beta_reg <= la_data_in[7:0];
        v_th_reg <= la_data_in[15:8];
        function_sel_reg <= la_data_in[32];
        sram_we <= la_data_in[33];
        neuron_weight_in <= sram_weight_out;
    end

    // wishbone ----------------------------------------------------------------------------------------------------

    // writes
    always @(posedge clk) begin
        if (rst) begin
            wbs_ack_o <= 0;
        end else begin
            wbs_ack_o = 1'b0;
            if(valid && !wbs_ack_o && wbs_adr_i == 32'h3000_000) begin
                wbs_ack_o = 1'b1;
                if (wstrb[0]) sram_weight_neuron_id_in <= wbs_dat_i[7:0];
                if (wstrb[1]) sram_weight_in <= wbs_dat_i[15:8];
            end
        end
    end

    // wishbone end ----------------------------------------------------------------------------------------------------

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

    reg[2:0] ps;                                //present state
    wire[2:0] ns;                               //next state 

    //cpu signals
    wire cpu_valid;                             //cpu signal for valid data
    wire inf_start;                             //cpu signal to start an inference
    wire [11:0] cpu_write_addr;                 //cpu addr signal used to write data
    wire [11:0] read_addr;                      //user signal to read from memory          
    wire [7:0] cpu_data;                        //data signal from cpu

    //data write signal
    wire [7:0] user_data;                       //data signal from user area
    wire [7:0] write_data;                      //see next ling
    assign write_data = user_data or cpu_data;  //single write data signal shared between cpu and user area depending on state

    //done signal
    wire inf_done;                              //inference done signal
    reg inf_done_reg;                           //inference done signal

    //memory enable signals
    wire queue_en;
    wire image0_en;
    wire image1_en;
    wire weights0_en;
    wire weights1_en;
    
    //memory data signals
    wire [7:0] queue_data;
    wire [7:0] image0_data;
    wire [7:0] image1_data;
    wire [7:0] weights0_data;
    wire [7:0] weights1_data;

    //signals for spike generation
    reg [7:0] curr_pixel_reg;                   //current pixel being processed (row wise)
    wire [7:0] curr_pixel;                      //see above line
    wire pixel_ld0;                             //load signal for memory bank 0
    reg pixel_ld0_reg;                          //load signal for memory bank 0
    wire pixel_ld1;                             //load signal for memory bank 0
    reg pixel_ld1_reg;                          //load signal for memory bank 0
    wire ld_queue;                              //enable signal for loading queue

    //intermediate registers for keeping track of neuron processing
    reg [3:0] vmem_processed_reg;               //how many output neurons have been calculated
    wire [3:0] vmem_processed;                  //how many output neurons have been caluclated
    reg [7:0] curr_timestep;                    //counter that tracks number of passed timesteps
    wire load_timestep;                         //enable signal for starting the next timestep
    reg [7:0] target_timestep_reg;              //number of timesteps to carry out
    wire [7:0] target_timestep;

    //internal reset
    wire rst_internal;                          //internal reset
    wire sys_rst;                               //system reset
    assign sys_rst = rst_internal or rst;       //internal reset depends on the actual internal reset signal or system reset

    //load spike intermediate signlas
    wire queue_valid;                           //whether or not the data from queue is valid - controls state transition
    reg [7:0] queue_reg;                        //data register from queue

    //output neuron specific registers
    reg [7:0] vmem [9:0];                       //registers of current vmem values
    reg [7:0] spike_num [9:0];                  //how many times neuron spiked
    
    //registers for operation results
    reg [7:0] curr_vmem_reg;                    //value of current membrane voltage (neuron)
    reg [7:0] curr_weight_reg;                  //value of current weight being used
    reg nueron_op_reg;                          //function for neuron to carry out

    //signals used for neuron operation
    wire [7:0] curr_vmem;                       //value of current membrane voltage (neuron)
    wire [7:0] curr_weight;                     //value of current weight being used
    wire [7:0] result;                          //neuron output for integrate function
    wire nueron_op;                             //function for neuron to carry out
    wire vmem_load;                             //dictates loading of a vmem register
    wire spike_load;                            //dictates incrementing a spike count register
    wire spike_out;                             //whether or not there is a spike
    reg [7:0] beta_reg;                         //beta value
    wire [7:0] beta;                            //beta value
    reg [7:0] vthresh_reg;                      //voltage threshold 
    wire [7:0] vthresh;
    

    //clocked process
    always @ (posedge(clk)) begin
        if (sys_rst) begin  //reset
            ps                      <= LOAD_IMG;
            vmem_processed_reg      <= 0;
            curr_timestep           <= 0;
            target_timestep_reg     <= NUM_TIMESTEPS;
            curr_vmem_reg           <= 0;
            curr_weight_reg         <= 0;
            nueron_op_reg           <= 0;
            beta_reg                <= 0;
            curr_pixel_reg          <= 0; 
            pixel_ld0_reg           <= 0;
            pixel_ld1_reg           <= 0;
            vthresh_reg             <= 0;
            queue_reg               <= 0;
            int i;  //iterate through all arrays of registers
            for (i = 0; i < 10; i = i + 1) begin
                vmem[i]             <= 0;
                spike_num[i]        <= 0;
            end

        end else begin //update registers
            curr_vmem_reg           <= curr_vmem;
            curr_weight_reg         <= curr_weight;
            nueron_op_reg           <= nueron_op;
            vmem_processed_reg      <= vmem_processed;
            curr_timestep           <= curr_timestep;
            target_timestep_reg     <= target_timestep;
            beta_reg                <= beta;
            curr_pixel_reg          <= curr_pixel;
            pixel_ld0_reg           <= pixel_ld0;
            pixel_ld1_reg           <= pixel_ld1;
            vthresh_reg             <= vthresh;
            target_timestep_reg     <= target_timestep;
            queue_reg               <= queue_reg; 
            ps                      <= ns;
            if (vmem_load == 1) begin   //enable signal for loading vmem register
                vmem[vmem_processed]        <= result;
            end
            if (spike_load == 1) begin  //enable signal for loading spike register
                spike_num[vmem_processed]   <= spike_num[vmem_processed] + 1;
            end
            if (load_timestep == 1) begin   //enable signal for incrementing timestep
                curr_timestep               <= curr_timestep + 1;
            end
            if (queue_valid == 1) begin
                queue_reg   <= queue_data;
            end
        end
    end

    //non clocked process
    always @ (ps) begin
        //add default statements for all conditional signals
        //the following have corresponding registers
        curr_vmem           = curr_vmem_reg;
        curr_weight         = curr_weight_reg;
        nueron_op           = nueron_op_reg;
        vmem_processed      = vmem_processed_reg;
        beta                = beta_reg;
        curr_pixel          = curr_pixel_reg;
        pixel_ld0           = pixel_ld0_reg;
        pixel_ld1           = pixel_ld1_reg;
        vthresh             = vthresh_reg;
        target_timestep_reg = target_timestep;
        //the following do not have corresponding registers
        read_addr           = 0;
        pixel_ld1           = 0;      
        pixel_ld0           = 0;
        vmem_load           = 0;
        spike_load          = 0;
        load_timestep       = 0;
        inf_done            = 0;
        rst_internal        = 0;
        queue_en            = 0;
        image0_en           = 0;
        image1_en           = 0;
        weights0_en         = 0;
        weights1_en         = 0;
        ld_queue            = 0;
        user_data           = 0;
        ns                  = ps;

        //begin case
        case(ps)
            LOAD_IMG:  
                if (cpu_valid == 1) begin                                                           //if cpu is sending valid data
                    if (cpu_write_addr[11] == 0) begin                                              //check the 'image data' bit
                        if (cpu_write_addr >= 98) begin    //if these bits are high we use the 2nd bank of memory
                            image1_en   = 1;                                                        //enable writing for 2nd memory bank
                        end else begin                                                              //use 1st bank
                            image0_en   = 1;                                                        //enable writing for 1st memory bank
                        end
                    end else if (cpu_write_addr == "111111111111") begin    //if cpu is sending a beta value
                        beta    = cpu_data;                                 //set beta value
                    end else if (cpu_write_addr == "111111111110") begin
                        vthresh = cpu_data;
                    end else if (cpu_write_addr == "111111111101") begin
                        target_timestep = cpu_data;

                end else begin                                              //weight data
                                                                            //if these bits are high use the 2nd bank
                        if (cpu_write_addr >= 980) begin 
                            weights1_en     = 1;                            //enable write signal
                        end else begin                                      //use 1st bank
                            weights0_en     = 1;                            //enable write signal
                        end
                    end
                    ns  = LOAD_IMG;
                end else if (inf_start == 1) begin      //if cpu sends 'start' signal for the inference (all data is loaded)
                    ns  = GEN_SPIKES;                   //move forward to spike generation stage
                end else begin                          //otherwise
                    ns  = LOAD_IMG;                     //stay in current state
                end

            //state for generating the spikes to be used by the neuron
            GEN_SPIKES:
                /*
                The following if statement checks for if the current pixel
                is 0 since it will 'lag' behind the fetched data by 1 timestep.
                This avoids the fatal case of trying to access the "-1st" pixel.

                This block contains the 'randomizer' algorithm for the neuron that
                involves a series of bit xor's. We carry this operation with data from
                the correct memory block (hence why there are two if blocks). 
                A spike is generated if the resulting signal is less than the pixel's
                value and this is loaded into the queue.
                */
                if (curr_pixel_reg != 0) begin                              
                    if (pixel_ld1_reg == 1) begin                           
                        user_data[7] = image1_data[7] xor image1_data[3];
                        user_data[6] = image1_data[6] xor image1_data[2];
                        user_data[5] = image1_data[5] xor image1_data[1];
                        user_data[4] = image1_data[4] xor image1_data[0];
                        user_data[3] = image1_data[3] xor user_data[7];
                        user_data[2] = image1_data[2] xor user_data[6];
                        user_data[1] = image1_data[1] xor user_data[5];
                        user_data[0] = image1_data[0] xor user_data[4];
                        if (user_data <= curr_pixel_reg) begin
                            ld_queue    = 1;
                            write_data  = curr_pixel_reg - 1;
                        end
                    end else if (pixel_ld0_reg == 1) begin
                        user_data[7] = image0_data[7] xor image0_data[3];
                        user_data[6] = image0_data[6] xor image0_data[2];
                        user_data[5] = image0_data[5] xor image0_data[1];
                        user_data[4] = image0_data[4] xor image0_data[0];
                        user_data[3] = image0_data[3] xor user_data[7];
                        user_data[2] = image0_data[2] xor user_data[6];
                        user_data[1] = image0_data[1] xor user_data[5];
                        user_data[0] = image0_data[0] xor user_data[4];
                        if (user_data <= curr_pixel_reg) begin
                            ld_queue    = 1;
                            write_data  = curr_pixel_reg - 1;
                        end
                    end
                end
                if (curr_pixel_reg >= 98) begin                 //if we reach 1/2 through the image, switch to mem block 1
                    pixel_ld1           = 1;
                    read_addr           = curr_pixel_reg;
                    curr_pixel          = curr_pixel_reg + 1;
                end else begin                                  //load next pixel and increment pixel count.
                    pixel_ld0           = 1;
                    read_addr           = curr_pixel_reg;
                    curr_pixel          = curr_pixel_reg + 1;
                end
                if (curr_pixel_reg == "11000100") begin         //done generating spikes
                    ns  = LOAD_SPIKE;
                    ld_queue    = 1; //
                end else begin
                    ns  = GEN_SPIKES;
                end

            LOAD_SPIKE:
                if (valid_o == 1) begin //valid signal
                    read_addr   = (queue_reg * 10) + vmem_processed_reg;  //desired weight value
                    weights0_en = 1;
                    weights1_en = 1;
                    ns          = LOAD_WEIGHT;
                end else begin //valid signal low
                    vmem_processed      = 0;
                    spikes_processed    = 0;
                    ns                  = LEAK_VMEM;   //all spikes processed
                end

            LOAD_WEIGHT:
                if (queue_data >= 980) begin //use 2nd bank
                    curr_weight     = weights1_data;
                end else begin //use 1st bank
                    curr_weight     = weights0_data;
                end
                curr_vmem           = vmem[vmem_processed_reg];   //since vmem is a register we can just load it in as we move on to the next stage
                nueron_op           = 0;
                ns                  = OPSTORE;

            OPSTORE:
                vmem_load       = 1;
                if (vmem_processed_reg == 9) begin
                    ns                  = LOAD_SPIKE;
                end else begin
                    vmem_processed      = vmem_processed_reg + 1;
                    ns                  = LOAD_WEIGHT;
                end

            LEAK_VMEM:
                curr_vmem   = vmem[vmem_processed_reg];
                nueron_op   = 1;
                ns          = LEAK_FIRE;

            LEAK_FIRE:
                if (spike_out == 1) begin
                    spike_load      = 1;
                end
                if (vmem_processed_reg = 9) begin
                    ns              = CHECK_END;
                end else begin
                    vmem_processed  = vmem_processed_reg + 1;
                    ns              = LEAK_VMEM;
                end

            CHECK_END:
                if (curr_timestep == target_timestep_reg - 1) begin
                    rst_internal    = 1;
                    inf_done        = 1;
                end else begin
                    ps              = GEN_SPIKES;     
                end

        end case
    end

    //neuron module
    neuron #() neuron(
        .weight(curr_weight_reg),
        .v_mem_in(curr_vmem_reg),
        .beta(beta_reg),
        .function_sel(nueron_op_reg),
        .v_th(vthresh_reg),
        .spike(spike_out),
        .v_mem_out(result)
    );

    //queue module
    queue #() queue(
        .clk(clk),
        .rst(sys_rst),
        .insert(ld_queue),
        .read(queue_en),
        .data_i(write_data),
        .valid_o(queue_valid),
        .data_o(queue_data)
    );

    //image storage
    sky130_sram_1kbyte_1rw1r_8x1024_8 #() image_0(
        // rw
        .clk0(clk),
        .csb0(image0_en),
        .web0(image0_en),
        .wmask0(1'b1),
        .addr0(cpu_write_addr),
        .din0(write_data),
        .dout0(),
        // r
        .clk1(clk),
        .csb1(1'b0),
        .addr1(read_addr),
        .dout1(image0_data)
    );

    sky130_sram_1kbyte_1rw1r_8x1024_8 #() image_1(
        // rw
        .clk0(clk),
        .csb0(image1_en),
        .web0(image1_en),
        .wmask0(1'b1),
        .addr0(cpu_write_addr),
        .din0(write_data),
        .dout0(),
        // r
        .clk1(clk),
        .csb1(1'b0),
        .addr1(read_addr),
        .dout1(image1_data)
    );

    //weight storage
    sky130_sram_1kbyte_1rw1r_8x1024_8 #() weights_0(
        // rw
        .clk0(clk),
        .csb0(weights0_en),
        .web0(weights0_en),
        .wmask0(1'b1),
        .addr0(cpu_write_addr),
        .din0(write_data),
        .dout0(),
        // r
        .clk1(clk),
        .csb1(1'b0),
        .addr1(read_addr),
        .dout1(weights0_data)
    );

    sky130_sram_1kbyte_1rw1r_8x1024_8 #() weights_1(
        // rw
        .clk0(clk),
        .csb0(weights1_en),
        .web0(weights1_en),
        .wmask0(1'b1),
        .addr0(cpu_write_addr),
        .din0(write_data),
        .dout0(),
        // r
        .clk1(clk),
        .csb1(1'b0),
        .addr1(read_addr),
        .dout1(weights1_data)
    );

endmodule

`default_nettype wire
