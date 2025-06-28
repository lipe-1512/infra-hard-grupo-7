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

    reg signed [63:0] product;
    reg [5:0]         count;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            product <= 64'd0;
            count   <= 6'd0;
            done    <= 1'b0;
            result  <= 64'd0;
        end else if (start) begin
            product <= {32'd0, b}; // Coloca multiplicador nos 32 bits inferiores
            count   <= 6'd0;
            done    <= 1'b0;
            result  <= 64'd0;
        end else if (!done) begin
            if (count < 32) begin
                // Checa os 2 bits menos significativos do produto (que contém o multiplicador)
                case (product[1:0])
                    2'b01: product[63:32] <= product[63:32] + a; // Add multiplicando
                    2'b10: product[63:32] <= product[63:32] - a; // Sub multiplicando
                    // 2'b00 e 2'b11: não faz nada
                endcase

                // Shift aritmético para a direita
                product <= product >>> 1;
                count <= count + 1;
            end else begin
                done <= 1'b1;
                result <= product;
            end
        end
    end
endmodule