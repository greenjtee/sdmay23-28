`default_nettype none

module queue #(
    parameter MAX_SIZE = 1024
)(
    input clk,
    input rst,
    input insert,
    input read,
    input [7:0] data_i,

    output valid_o,
    output [7:0] data_o
);
    reg [9:0] head_address;
    reg [9:0] tail_address;
    reg [9:0] size;

    reg [7:0] queue_data_i;
    reg queue_insert;
    wire [7:0] data_o;

    assign valid_o = |size; // if queue has any size, output is valid

    always @(posedge clk) begin
        if (rst) begin

            head_address <= 10'b00_0000_0000;
            tail_address <= 10'b00_0000_0000;
            size <= 10'b00_0000_0000;
        end else begin

            queue_data_i    <= data_i;
            queue_insert    <= ~insert;

            if (insert) begin
                size <= size + 1;
                tail_address <= tail_address + 1;
            end else if (read) begin
                size <= size - 1;
                head_address <= head_address + 1;
            end
        end
    end

    sky130_sram_1kbyte_1rw1r_8x1024_8 #(
        .VERBOSE(1)
    )
    queue_sram(
        // rw
        .clk0(clk),
        .csb0(1'b0),
        .web0(queue_insert),
        .wmask0(1'b1),
        .addr0(tail_address),
        .din0(queue_data_i),
        .dout0(),
        // r
        .clk1(clk),
        .csb1(1'b0),
        .addr1(head_address),
        .dout1(data_o)
    );

endmodule

`default_nettype wire
