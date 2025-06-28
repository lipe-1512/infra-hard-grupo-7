// register.v
// Registrador genérico de N bits com load enable e reset assíncrono.
module Registrador #(
    parameter WIDTH = 32
)(
    input wire clk,
    input wire reset,
    input wire Load,
    input wire [WIDTH-1:0] Entrada,
    output reg [WIDTH-1:0] Saida
);

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            Saida <= {WIDTH{1'b0}};
        end else if (Load) begin
            Saida <= Entrada;
        end
    end

endmodule