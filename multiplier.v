module multiplier (
    input [31:0] a, b,        // Operandos
    input start,              // Sinal de início
    input clk,                // Sinal de clock (adicionado)
    output reg [63:0] result, // Resultado (64 bits)
    output reg done           // Sinal de conclusão
);
    reg [31:0] multiplicand;
    reg [31:0] multiplier;
    reg [63:0] product;
    reg [5:0] count;

    always @(posedge start) begin
        if (start) begin
            multiplicand = a;
            multiplier = b;
            product = 64'b0;
            count = 0;
            done = 0;
            result = 64'b0; // Inicializa o resultado aqui
        end
    end

    always @(posedge clk) begin
        if (count < 32) begin
            if (multiplier[0] == 1'b1) begin
                product = product + multiplicand;
            end
            multiplicand = multiplicand << 1;
            multiplier = multiplier >> 1;
            count = count + 1;
        end else begin
            done = 1;
            result = product; // Atualiza o resultado ao final do cálculo
        end
    end
endmodule