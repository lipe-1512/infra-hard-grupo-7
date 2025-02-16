module exception (
    input overflow, div_by_zero, invalid_opcode, // Sinais de exceção
    input [31:0] pc,                            // Endereço da instrução
    output reg [31:0] epc,                       // Endereço salvo
    output reg exception,                       // Sinal de exceção
    output reg [2:0] exception_code,            // Código da exceção
    output reg [31:0] new_pc                    // Novo valor do PC
);

    always @(*) begin
        if (overflow || div_by_zero || invalid_opcode) begin
            epc = pc; // Salva o endereço da instrução
            exception = 1;
            if (overflow) begin
                new_pc = 32'h000000FD; // Endereço da rotina de overflow
                exception_code = 3'b001; // Código para overflow
            end else if (div_by_zero) begin
                new_pc = 32'h000000FE; // Endereço da rotina de divisão por zero
                exception_code = 3'b010; // Código para divisão por zero
            end else if (invalid_opcode) begin
                new_pc = 32'h000000FF; // Endereço da rotina de opcode inválido
                exception_code = 3'b100; // Código para opcode inválido
            end
        end else begin
            exception = 0;
            new_pc = pc + 4; // Continua a execução normal
            exception_code = 3'b000; // Nenhuma exceção
        end
    end

endmodule