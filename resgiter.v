// register.v
// Registrador genérico de N bits com load enable e reset assíncrono.
// Substitui o arquivo Registrador.vhd corrompido.
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
        if (reset)
            Saida <= {WIDTH{1'b0}};
        else if (Load)
            Saida <= Entrada;
    end

endmodule