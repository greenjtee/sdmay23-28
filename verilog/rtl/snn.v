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

    queue #()
    spike_queue (
        .clk(clk),
        .insert(1'b1),
        .read(1'b1),
        .data_i(la_data_in[103:96]),
        .valid_o(la_data_in[112]),
        .data_o(la_data_out[111:104])
    );

    neuron #(

    ) neuron(
        .weight(neuron_weight_in),
        .v_mem_in(vmem_in),
        .beta(beta_reg),
        .function_sel(function_sel_reg),
        .v_th(v_th_reg),
        .spike(neuron_spike_out),
        .v_mem_out(neuron_v_mem_out)
    );

    sky130_sram_1kbyte_1rw1r_8x1024_8 #(
        .VERBOSE(0)
    )
    sram_weights(
        // rw
        .clk0(clk),
        .csb0(1'b0),
        .web0(sram_we),
        .wmask0(1'b1),
        .addr0(sram_weight_neuron_id_in),
        .din0(sram_weight_in),
        .dout0(),
        // r
        .clk1(clk),
        .csb1(1'b0),
        .addr1(neuron_id_current),
        .dout1(sram_weight_out)
    );

endmodule

`default_nettype wire
