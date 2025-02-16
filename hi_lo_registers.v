module hi_lo_registers (
    input clk, reset,
    input [31:0] hi_in, lo_in, // Entradas
    input hi_write, lo_write,  // Sinais de escrita
    output reg [31:0] hi_out, lo_out // Saídas
);

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            hi_out <= 32'b0;
            lo_out <= 32'b0;
        end else begin
            if (hi_write) hi_out <= hi_in;
            if (lo_write) lo_out <= lo_in;
        end
    end

endmodule