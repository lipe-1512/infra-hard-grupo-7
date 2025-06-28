// control_unit.v
module control_unit (
    input wire clk, reset,
    input wire [5:0] opcode,
    input wire [5:0] funct,
    // Saídas de Controle
    output reg PCWrite, PCWriteCond, IorD, MemRead, MemWrite, IRWrite, RegWrite,
    output reg MemtoReg, RegDst, ALUSrcA,
    output reg [1:0] ALUSrcB, PCSource,
    output reg [3:0] ALUOp
);

    // Definição dos Estados
    parameter FETCH       = 0, DECODE = 1,
              MEM_ADDR    = 2, LW_READ = 3, LW_WB = 4,
              SW_WRITE    = 5, R_EXECUTE = 6, R_WB = 7,
              BRANCH_EXEC = 8, JUMP_EXEC = 9,
              ADDI_EXEC   = 10, ADDI_WB = 11;
              // Adicionar mais estados para mult, div, etc. se necessário

    reg [3:0] state, next_state;

    // Lógica do próximo estado (combinacional)
    always @(*) begin
        case (state)
            FETCH:      next_state = DECODE;
            DECODE: begin
                case (opcode)
                    6'b000000: next_state = R_EXECUTE;  // R-type
                    6'b100011: next_state = MEM_ADDR;   // lw
                    6'b101011: next_state = MEM_ADDR;   // sw
                    6'b000100: next_state = BRANCH_EXEC; // beq
                    6'b001000: next_state = ADDI_EXEC;  // addi
                    6'b000010: next_state = JUMP_EXEC;  // j
                    default:   next_state = FETCH;    // Opcode inválido (tratar exceção aqui)
                endcase
            end
            MEM_ADDR: begin
                if (opcode == 6'b100011) next_state = LW_READ; // lw
                else next_state = SW_WRITE; // sw
            end
            LW_READ:    next_state = LW_WB;
            SW_WRITE:   next_state = FETCH;
            LW_WB:      next_state = FETCH;
            R_EXECUTE:  next_state = R_WB;
            R_WB:       next_state = FETCH;
            BRANCH_EXEC:next_state = FETCH;
            JUMP_EXEC:  next_state = FETCH;
            ADDI_EXEC:  next_state = ADDI_WB;
            ADDI_WB:    next_state = FETCH;
            default:    next_state = FETCH;
        endcase
    end
    
    // Lógica de atualização de estado (sequencial)
    always @(posedge clk or posedge reset) begin
        if (reset) state <= FETCH;
        else state <= next_state;
    end

    // Lógica de saída (gera sinais de controle baseado no estado atual)
    always @(*) begin
        // Valores padrão (inativos)
        PCWrite=0; PCWriteCond=0; IorD=0; MemRead=0; MemWrite=0; IRWrite=0;
        RegWrite=0; MemtoReg=0; RegDst=0; ALUSrcA=0; ALUSrcB=2'b00;
        PCSource=2'b00; ALUOp=4'b0000;

        case (state)
            FETCH: begin // Busca da Instrução
                MemRead = 1;
                IRWrite = 1;
                PCWrite = 1;
                ALUSrcA = 0; // PC
                ALUSrcB = 2'b01; // 4
                ALUOp = 4'b0001; // Soma
                PCSource = 2'b00; // Saída da ALU (PC+4)
            end
            DECODE: begin // Decodificação e busca de operandos
                ALUSrcA = 0; // PC
                ALUSrcB = 2'b11; // imm << 2
                ALUOp = 4'b0001; // Soma (cálculo do end. de branch)
            end
            MEM_ADDR: begin // lw/sw - cálculo do endereço
                ALUSrcA = 1; // Reg A
                ALUSrcB = 2'b10; // SignExt imm
                ALUOp = 4'b0001; // Soma
            end
            LW_READ: begin // lw - leitura da memória
                MemRead = 1;
                IorD = 1;
            end
            LW_WB: begin // lw - escrita no registrador
                RegWrite = 1;
                MemtoReg = 1;
                RegDst = 0; // rt
            end
            SW_WRITE: begin // sw - escrita na memória
                MemWrite = 1;
                IorD = 1;
            end
            R_EXECUTE: begin // R-type - execução
                ALUSrcA = 1; // Reg A
                ALUSrcB = 2'b00; // Reg B
                // ALUOp depende do funct
                case (funct)
                    6'b100000: ALUOp = 4'b0001; // add -> Soma
                    6'b100010: ALUOp = 4'b0010; // sub -> Sub
                    6'b100100: ALUOp = 4'b0011; // and -> And
                    // ... adicionar outros R-type
                    default:   ALUOp = 4'b0000;
                endcase
            end
            R_WB: begin // R-type - escrita no registrador
                RegWrite = 1;
                MemtoReg = 0;
                RegDst = 1; // rd
            end
            BRANCH_EXEC: begin // beq - desvio condicional
                ALUSrcA = 1; // Reg A
                ALUSrcB = 2'b00; // Reg B
                ALUOp = 4'b0010; // Subtração para comparar
                PCWriteCond = 1;
                PCSource = 2'b01; // ALUOut (resultado do cálculo de branch do estado DECODE)
            end
            JUMP_EXEC: begin // j - desvio incondicional
                PCWrite = 1;
                PCSource = 2'b10; // Endereço de jump
            end
            ADDI_EXEC: begin // addi - execução
                ALUSrcA = 1; // Reg A
                ALUSrcB = 2'b10; // SignExt imm
                ALUOp = 4'b0001; // Soma
            end
            ADDI_WB: begin // addi - escrita no registrador
                RegWrite = 1;
                MemtoReg = 0;
                RegDst = 0; // rt
            end
        endcase
    end
endmodule