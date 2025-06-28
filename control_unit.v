module control_unit (
    input clk,
    input reset,
    input [5:0] opcode,
    input [5:0] funct,
    input Overflow,                
    output reg RegWrite,
    output reg MemWrite,
    output reg MemRead,
    output reg MemtoReg,
    output reg ALUSrc,
    output reg RegDst,
    output reg Branch,
    output reg Jump,
    output reg [2:0] ALUOp,
    output reg [2:0] state     
);

    // Definição dos estados
    localparam [2:0]
        FETCH = 3'b000,            // Busca da instrução
        DECODE = 3'b001,           // Decodificação
        EXECUTE = 3'b010,          // Execução
        MEMORY = 3'b011,           // Acesso à memória
        WRITEBACK = 3'b100,        // Escrita no banco de registradores
        OVERFLOW = 3'b101;         // Estado de tratamento de overflow

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            // Reinicializa todos os sinais de controle e volta ao estado FETCH
            state <= FETCH;
            RegWrite <= 0;
            MemWrite <= 0;
            MemRead <= 0;
            MemtoReg <= 0;
            ALUSrc <= 0;
            RegDst <= 0;
            Branch <= 0;
            Jump <= 0;
            ALUOp <= 3'b000;
        end else begin
            case (state)
                FETCH: begin
                    // Próximo estado: DECODE
                    state <= DECODE;
                end

                DECODE: begin
                    // Configura sinais de controle com base no opcode
                    case (opcode)
                        6'b000000: begin // Instruções tipo R
                            case (funct)
                                6'b100000: begin // add
                                    RegWrite <= 1;
                                    MemWrite <= 0;
                                    MemRead <= 0;
                                    MemtoReg <= 0;
                                    ALUSrc <= 0;
                                    RegDst <= 1;
                                    Branch <= 0;
                                    Jump <= 0;
                                    ALUOp <= 3'b010; // Soma
                                end
                                6'b100100: begin // and
                                    RegWrite <= 1;
                                    MemWrite <= 0;
                                    MemRead <= 0;
                                    MemtoReg <= 0;
                                    ALUSrc <= 0;
                                    RegDst <= 1;
                                    Branch <= 0;
                                    Jump <= 0;
                                    ALUOp <= 3'b011; // AND
                                end
                                6'b011010: begin // div
                                    RegWrite <= 0;
                                    MemWrite <= 0;
                                    MemRead <= 0;
                                    MemtoReg <= 0;
                                    ALUSrc <= 0;
                                    RegDst <= 0;
                                    Branch <= 0;
                                    Jump <= 0;
                                    ALUOp <= 3'b110; // Divisão
                                end
                                6'b011000: begin // mult
                                    RegWrite <= 0;
                                    MemWrite <= 0;
                                    MemRead <= 0;
                                    MemtoReg <= 0;
                                    ALUSrc <= 0;
                                    RegDst <= 0;
                                    Branch <= 0;
                                    Jump <= 0;
                                    ALUOp <= 3'b101; // Multiplicação
                                end
                                6'b001000: begin // jr
                                    RegWrite <= 0;
                                    MemWrite <= 0;
                                    MemRead <= 0;
                                    MemtoReg <= 0;
                                    ALUSrc <= 0;
                                    RegDst <= 0;
                                    Branch <= 0;
                                    Jump <= 1; // Jump register
                                    ALUOp <= 3'b000;
                                end
                                6'b010000: begin // mfhi
                                    RegWrite <= 1;
                                    MemWrite <= 0;
                                    MemRead <= 0;
                                    MemtoReg <= 0;
                                    ALUSrc <= 0;
                                    RegDst <= 1;
                                    Branch <= 0;
                                    Jump <= 0;
                                    ALUOp <= 3'b111; // Move from HI
                                end
                                6'b010010: begin // mflo
                                    RegWrite <= 1;
                                    MemWrite <= 0;
                                    MemRead <= 0;
                                    MemtoReg <= 0;
                                    ALUSrc <= 0;
                                    RegDst <= 1;
                                    Branch <= 0;
                                    Jump <= 0;
                                    ALUOp <= 3'b111; // Move from LO
                                end
                                6'b000100: begin // sll (funct code corrected to 000100)
                                    RegWrite <= 1;
                                    MemWrite <= 0;
                                    MemRead <= 0;
                                    MemtoReg <= 0;
                                    ALUSrc <= 0;
                                    RegDst <= 1;
                                    Branch <= 0;
                                    Jump <= 0;
                                    ALUOp <= 3'b100; // Shift left logical
                                end
                                6'b101010: begin // slt
                                    RegWrite <= 1;
                                    MemWrite <= 0;
                                    MemRead <= 0;
                                    MemtoReg <= 0;
                                    ALUSrc <= 0;
                                    RegDst <= 1;
                                    Branch <= 0;
                                    Jump <= 0;
                                    ALUOp <= 3'b001; // Set less than
                                end
                                6'b000011: begin // sra
                                    RegWrite <= 1;
                                    MemWrite <= 0;
                                    MemRead <= 0;
                                    MemtoReg <= 0;
                                    ALUSrc <= 0;
                                    RegDst <= 1;
                                    Branch <= 0;
                                    Jump <= 0;
                                    ALUOp <= 3'b100; // Shift right arithmetic
                                end
                                6'b100010: begin // sub
                                    RegWrite <= 1;
                                    MemWrite <= 0;
                                    MemRead <= 0;
                                    MemtoReg <= 0;
                                    ALUSrc <= 0;
                                    RegDst <= 1;
                                    Branch <= 0;
                                    Jump <= 0;
                                    ALUOp <= 3'b001; // Subtração
                                end
                                default: begin // Instruções não suportadas
                                    RegWrite <= 0;
                                    MemWrite <= 0;
                                    MemRead <= 0;
                                    MemtoReg <= 0;
                                    ALUSrc <= 0;
                                    RegDst <= 0;
                                    Branch <= 0;
                                    Jump <= 0;
                                    ALUOp <= 3'b000;
                                end
                            endcase
                        end
                        6'b001000: begin // addi
                            RegWrite <= 1;
                            MemWrite <= 0;
                            MemRead <= 0;
                            MemtoReg <= 0;
                            ALUSrc <= 1;
                            RegDst <= 0;
                            Branch <= 0;
                            Jump <= 0;
                            ALUOp <= 3'b010; // Soma
                        end
                        6'b000101: begin // bne
                            RegWrite <= 0;
                            MemWrite <= 0;
                            MemRead <= 0;
                            MemtoReg <= 0;
                            ALUSrc <= 0;
                            RegDst <= 0;
                            Branch <= 1;
                            Jump <= 0;
                            ALUOp <= 3'b001; // Subtração para comparar registradores
                        end
                        6'b100000: begin // lb
                            RegWrite <= 1;
                            MemWrite <= 0;
                            MemRead <= 1;
                            MemtoReg <= 1;
                            ALUSrc <= 1;
                            RegDst <= 0;
                            Branch <= 0;
                            Jump <= 0;
                            ALUOp <= 3'b000; // Soma para calcular o endereço
                        end
                        6'b001111: begin // lui
                            RegWrite <= 1;
                            MemWrite <= 0;
                            MemRead <= 0;
                            MemtoReg <= 0;
                            ALUSrc <= 1;
                            RegDst <= 0;
                            Branch <= 0;
                            Jump <= 0;
                            ALUOp <= 3'b000; // Load upper immediate
                        end
                        6'b101000: begin // sb
                            RegWrite <= 0;
                            MemWrite <= 1;
                            MemRead <= 0;
                            MemtoReg <= 0;
                            ALUSrc <= 1;
                            RegDst <= 0;
                            Branch <= 0;
                            Jump <= 0;
                            ALUOp <= 3'b000; // Soma para calcular o endereço
                        end
                        6'b000011: begin // jal
                            RegWrite <= 1;
                            MemWrite <= 0;
                            MemRead <= 0;
                            MemtoReg <= 0;
                            ALUSrc <= 0;
                            RegDst <= 0;
                            Branch <= 0;
                            Jump <= 1;
                            ALUOp <= 3'b000; // Jump and link
                        end
                        default: begin // Instruções não suportadas
                            RegWrite <= 0;
                            MemWrite <= 0;
                            MemRead <= 0;
                            MemtoReg <= 0;
                            ALUSrc <= 0;
                            RegDst <= 0;
                            Branch <= 0;
                            Jump <= 0;
                            ALUOp <= 3'b000;
                        end
                    endcase
                    state <= EXECUTE;
                end

                EXECUTE: begin
                    // Verifica se ocorreu overflow
                    if (Overflow == 1'b1) begin
                        state <= OVERFLOW; // Vai para o estado de tratamento de overflow
                    end else begin
                        state <= MEMORY;   // Continua para o estado MEMORY
                    end
                end

                MEMORY: begin
                    // Acesso à memória (se necessário)
                    state <= WRITEBACK;
                end

                WRITEBACK: begin
                    // Escreve o resultado no banco de registradores
                    // Ajusta RegWrite para ativar somente se a instrução requer escrita
                    if (RegWrite == 1'b1) begin
                        RegWrite <= 1;
                    end else begin
                        RegWrite <= 0;
                    end
                    state <= FETCH; // Retorna ao estado FETCH após a escrita
                end

                OVERFLOW: begin
                    // Estado de tratamento de overflow
                    // Desativa todos os sinais de controle
                    RegWrite <= 0;
                    MemWrite <= 0;
                    MemRead <= 0;
                    MemtoReg <= 0;
                    ALUSrc <= 0;
                    RegDst <= 0;
                    Branch <= 0;
                    Jump <= 0;
                    ALUOp <= 3'b000;

                    // Redireciona o PC para o endereço de tratamento de overflow
                    state <= FETCH; // Retorna ao estado FETCH após o tratamento
                end

                default: begin
                    // Estado inválido, retorna ao FETCH
                    state <= FETCH;
                end
            endcase
        end
    end
endmodule