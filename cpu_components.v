//****************************************************************************
// Módulo: cpu_control_unit
// Descrição: Uma Máquina de Estados Finitos (FSM) correta para o
//              caminho de dados multiciclo mostrado no diagrama.
//****************************************************************************
module cpu_control_unit (
    input wire clk,
    input wire reset,
    input wire [5:0] opcode,
    input wire [5:0] funct,
    output reg PCWrite,
    output reg IorD,
    output reg MemRead,
    output reg MemWrite,
    output reg IRWrite,
    output reg RegWrite,
    output reg [1:0] MemtoReg,
    output reg [1:0] RegDst,
    output reg AWrite,
    output reg BWrite,
    output reg ALUOutWrite,
    output reg [1:0] ALUSrcA,
    output reg [1:0] ALUSrcB,
    output reg [2:0] ALUOp,
    output reg [1:0] PCSrc
);

    // Estados da FSM
    parameter S_FETCH = 0, S_DECODE = 1, S_MEM_ADDR = 2, S_MEM_READ = 3;
    parameter S_MEM_WB = 4, S_MEM_WRITE = 5, S_EXEC_R = 6, S_WB_R = 7;
    parameter S_BRANCH = 8, S_JUMP = 9, S_JR = 10, S_ADDI_EXEC = 11, S_ADDI_WB = 12;

    reg [3:0] state, next_state;

    // Opcodes das instruções para decodificação
    localparam OP_RTYPE = 6'b000000;
    localparam OP_ADDI  = 6'b001000;
    localparam OP_LW    = 6'b100011;
    localparam OP_SW    = 6'b101011;
    localparam OP_BEQ   = 6'b000100;
    localparam OP_J     = 6'b000010;
    // Códigos de função (funct) para tipo R
    localparam F_JR = 6'b001000;

    // Lógica sequencial da FSM (registrador de estado)
    always @(posedge clk or posedge reset) begin
        if (reset) state <= S_FETCH;
        else state <= next_state;
    end

    // Lógica do próximo estado (combinacional)
    always @(*) begin
        case (state)
            S_FETCH: next_state = S_DECODE;
            S_DECODE: begin
                case (opcode)
                    OP_RTYPE: begin
                        if (funct == F_JR) next_state = S_JR;
                        else next_state = S_EXEC_R;
                    end
                    OP_LW:    next_state = S_MEM_ADDR;
                    OP_SW:    next_state = S_MEM_ADDR;
                    OP_BEQ:   next_state = S_BRANCH;
                    OP_J:     next_state = S_JUMP;
                    OP_ADDI:  next_state = S_ADDI_EXEC;
                    default:  next_state = S_FETCH; // Instrução não suportada
                endcase
            end
            S_MEM_ADDR: begin
                if (opcode == OP_LW) next_state = S_MEM_READ;
                else next_state = S_MEM_WRITE; // SW
            end
            S_MEM_READ: next_state = S_MEM_WB;
            S_EXEC_R:   next_state = S_WB_R;
            S_ADDI_EXEC:next_state = S_ADDI_WB;
            S_MEM_WB, S_MEM_WRITE, S_WB_R, S_BRANCH, S_JUMP, S_JR, S_ADDI_WB:
                 next_state = S_FETCH;
            default: next_state = S_FETCH;
        endcase
    end

    // Lógica de saída da FSM (combinacional)
    always @(*) begin
        // Valores padrão (todos os sinais desativados)
        PCWrite=0; IorD=0; MemRead=0; MemWrite=0; IRWrite=0; RegWrite=0;
        MemtoReg=0; RegDst=0; AWrite=0; BWrite=0; ALUOutWrite=0;
        ALUSrcA=0; ALUSrcB=0; ALUOp=0; PCSrc=0;

        case (state)
            S_FETCH: begin
                // Mem[PC] -> IR, PC+4 -> PC
                MemRead = 1; IRWrite = 1; IorD = 0;
                ALUSrcA = 2'b00; // PC
                ALUSrcB = 2'b01; // 4
                ALUOp = 3'b001;  // add
                PCWrite = 1;
                PCSrc = 2'b00;   // Resultado da ULA
            end
            S_DECODE: begin
                // Decodifica, lê registradores, calcula end. de branch
                AWrite = 1; BWrite = 1;
                ALUSrcA = 2'b00; // PC
                ALUSrcB = 2'b11; // sign-ext << 2
                ALUOp = 3'b001;  // add
                ALUOutWrite = 1;
            end
            S_MEM_ADDR: begin // Para LW e SW
                // A + sign-ext(imm) -> ALUOut
                ALUSrcA = 2'b01; // A
                ALUSrcB = 2'b10; // sign-ext
                ALUOp = 3'b001;  // add
                ALUOutWrite = 1;
            end
            S_MEM_READ: begin // Para LW
                // Mem[ALUOut] -> MDR
                MemRead = 1; IorD = 1;
            end
            S_MEM_WB: begin // Para LW (Write-Back)
                // MDR -> Reg[rt]
                RegWrite = 1;
                MemtoReg = 2'b01; // Dado da memória
                RegDst = 2'b00;   // Escreve em rt
            end
            S_MEM_WRITE: begin // Para SW
                // B -> Mem[ALUOut]
                MemWrite = 1; IorD = 1;
            end
            S_EXEC_R: begin // Para Tipo-R
                // A op B -> ALUOut
                ALUSrcA = 2'b01; // A
                ALUSrcB = 2'b00; // B
                ALUOp = 3'b010;  // Vem do funct (precisaria de um decodificador de ULA)
                ALUOutWrite = 1;
            end
            S_WB_R: begin // Para Tipo-R (Write-Back)
                // ALUOut -> Reg[rd]
                RegWrite = 1;
                MemtoReg = 2'b00; // Dado da ULA
                RegDst = 2'b01;   // Escreve em rd
            end
            S_BRANCH: begin // Para BEQ
                // if (A == B) PC = ALUOut
                ALUSrcA = 2'b01; // A
                ALUSrcB = 2'b00; // B
                ALUOp = 3'b010;  // sub para comparar
                PCSrc = 2'b01;   // Endereço de Branch
                // PCWrite será ativado condicionalmente (se Zero=1)
            end
            S_JUMP: begin // Para J
                PCWrite = 1;
                PCSrc = 2'b10;   // Endereço de Jump
            end
            S_JR: begin // Para JR
                PCWrite = 1;
                PCSrc = 2'b11;   // Registrador A
            end
            S_ADDI_EXEC: begin // Para ADDI
                // A + sign-ext(imm) -> ALUOut
                ALUSrcA = 2'b01; // A
                ALUSrcB = 2'b10; // imediato com ext. de sinal
                ALUOp = 3'b001;  // add
                ALUOutWrite = 1;
            end
            S_ADDI_WB: begin // Para ADDI (Write-Back)
                // ALUOut -> Reg[rt]
                RegWrite = 1;
                MemtoReg = 2'b00; // Dado da ULA
                RegDst = 2'b00;   // Escreve em rt
            end
        endcase
    end
endmodule


//****************************************************************************
// Módulos de Multiplexadores (MUX)
//****************************************************************************
module mux2_1_32b(input [31:0] d0, d1, input sel, output [31:0] y);
    assign y = sel ? d1 : d0;
endmodule

module mux4_1_32b(input [31:0] d0, d1, d2, d3, input [1:0] sel, output [31:0] y);
    assign y = sel[1] ? (sel[0] ? d3 : d2) : (sel[0] ? d1 : d0);
endmodule

module mux2_1_5b(input [4:0] d0, d1, input sel, output [4:0] y);
    assign y = sel ? d1 : d0;
endmodule