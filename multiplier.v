// multiplier.v
// Módulo que implementa multiplicação 32x32 com sinal usando o Algoritmo de Booth.
// Resulta em um produto de 64 bits. É iterativo e leva 32 ciclos.
module multiplier (
    input wire signed [31:0] a,         // Multiplicando
    input wire signed [31:0] b,         // Multiplicador
    input wire               start,     // Sinal de início
    input wire               clk,
    input wire               reset,
    output reg signed [63:0] result,    // Resultado (64 bits)
    output reg               done       // Sinal de conclusão
);

    reg signed [64:0] accumulator; // Acumulador estendido com bit extra para Booth
    reg [5:0]         count;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            accumulator <= 65'd0;
            count   <= 6'd0;
            done    <= 1'b0;
            result  <= 64'd0;
        end else if (start) begin
            accumulator <= {32'd0, b, 1'b0}; // {A (zeros), Q (multiplicador), Q-1 (zero)}
            count   <= 6'd0;
            done    <= 1'b0;
            result  <= 64'd0;
        end else if (!done) begin
            if (count < 32) begin
                // Checa os 2 bits menos significativos (Q0 e Q-1)
                case (accumulator[1:0])
                    2'b01: accumulator[64:33] <= accumulator[64:33] + a; // 01: A = A + M
                    2'b10: accumulator[64:33] <= accumulator[64:33] - a; // 10: A = A - M
                    // 2'b00 e 2'b11: não faz nada
                endcase

                // Shift aritmético para a direita em todo o acumulador de 65 bits
                accumulator <= $signed(accumulator) >>> 1;
                count <= count + 1;
            end else begin
                done <= 1'b1;
                result <= accumulator[64:1]; // O resultado está nos 64 bits superiores
            end
        end
    end
endmodule