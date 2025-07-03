module hi_lo_registers (
    input wire clk,
    input wire reset,
    input wire [31:0] hi_in,
    input wire [31:0] lo_in,
    input wire hi_write,
    input wire lo_write,
    output reg [31:0] hi_out,
    output reg [31:0] lo_out
);

    always @(posedge clk or posedge reset) begin
    if (reset) begin
        hi_out <= 32'h00000000;
        lo_out <= 32'h00000000;
    end else begin
        if (hi_write) hi_out <= hi_in;
        if (lo_write) lo_out <= lo_in;
    end
end

endmodule