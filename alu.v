module alu (
    input [31:0] a, b,        // Operandos
    input [2:0] alu_control,  // Controle da ALU
    output reg [31:0] result,  // Resultado
    output zero                // Sinal de zero
);

    always @(*) begin
        case (alu_control)
            3'b000: result = a + b;       // Soma
            3'b001: result = a - b;       // Subtração
            3'b010: result = a & b;       // AND
            3'b011: result = a | b;       // OR
            3'b100: result = a << b;      // Shift left
            3'b101: result = a >> b;      // Shift right lógico
            3'b110: result = a >>> b;     // Shift right aritmético
            default: result = 32'b0;      // Operação inválida
        endcase
    end

    assign zero = (result == 32'b0); // Sinal de zero

endmodule