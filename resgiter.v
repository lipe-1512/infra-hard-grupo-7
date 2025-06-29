// registrador.v
// Registrador genérico com reset assíncrono global e clear síncrono.
module registrador #(
    parameter WIDTH = 32
)(
    input wire clk,
    input wire reset,        // Reset assíncrono para garantir um estado inicial conhecido na simulação
    input wire Load,
    input wire Clear,        // Novo: Sinal de clear síncrono controlado pela FSM
    input wire [WIDTH-1:0] Entrada,
    output reg [WIDTH-1:0] Saida
);

    always @(posedge clk or posedge reset) begin
        if (reset)
            Saida <= {WIDTH{1'b0}}; // Reset assíncrono para estado inicial
        else if (Clear) // Clear síncrono tem prioridade sobre o Load
            Saida <= {WIDTH{1'b0}};
        else if (Load)
            Saida <= Entrada;
    end

endmodule