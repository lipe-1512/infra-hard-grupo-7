// control_unit.v
// Unidade de Controle COMPLETA e FINAL baseada em FSM para o processador MIPS multi-ciclo.
// Gerencia o fluxo de estados para TODAS as instruções da especificação do projeto.
module control_unit (
    input wire clk, reset,
    input wire [5:0] opcode,
    input wire [5:0] funct,
    // Sinais de status do datapath
    input wire mult_done_in, div_done_in,

    // Saídas de Controle para o datapath
    output reg PCWrite, PCWriteCond, IorD, MemRead, MemWrite, IRWrite, RegWrite,
    output reg [1:0] RegDst,
    output reg ALUSrcA,
    output reg [1:0] ALUSrcB,
    output reg [1:0] PCSource,
    output reg [3:0] ALUOp,
    output reg HIWrite, LOWrite, MultStart, DivStart,
    output reg InvalidOpcodeOut,
    output reg [1:0] WBDataSrc // Fonte do dado para Write-Back: 00=ALUOut, 01=MDR, 10=HI, 11=LO
);

    // =================================================================
    // --- DEFINIÇÃO DE ESTADOS DA FSM ---
    // =================================================================
    parameter S_FETCH        = 0,  S_DECODE       = 1,
              S_MEM_ADDR     = 2,  S_LW_READ      = 3,  S_LW_WB        = 4,
              S_SW_WRITE     = 5,  S_R_EXECUTE    = 6,  S_R_WB         = 7,
              S_BRANCH_EXEC  = 8,  S_JUMP_EXEC    = 9,  S_I_TYPE_EXEC  = 10,
              S_LUI_EXEC     = 11, S_JAL_EXEC     = 12,
              S_MULT_START   = 13, S_MULT_WAIT    = 14, S_DIV_START    = 15,
              S_DIV_WAIT     = 16, S_MFHI_WB      = 17, S_MFLO_WB      = 18;

    // =================================================================
    // --- OPCODES E FUNCTS (para clareza) ---
    // =================================================================
    // Opcodes
    localparam OP_RTYPE = 6'b000000;
    localparam OP_ADDI  = 6'b001000;
    localparam OP_SLTI  = 6'b001010;
    localparam OP_LW    = 6'b100011;
    localparam OP_SW    = 6'b101011;
    localparam OP_BEQ   = 6'b000100;
    localparam OP_BNE   = 6'b000101;
    localparam OP_LUI   = 6'b001111;
    localparam OP_J     = 6'b000010;
    localparam OP_JAL   = 6'b000011;

    // Functs (para R-Type)
    localparam F_ADD  = 6'b100000; localparam F_SUB  = 6'b100010;
    localparam F_AND  = 6'b100100; localparam F_OR   = 6'b100101;
    localparam F_SLT  = 6'b101010; localparam F_JR   = 6'b001000;
    localparam F_MULT = 6'b011000; localparam F_DIV  = 6'b011010;
    localparam F_MFHI = 6'b010000; localparam F_MFLO = 6'b010010;

    // =================================================================
    // --- LÓGICA DE ESTADO ---
    // =================================================================
    reg [4:0] state, next_state;

    // Lógica de Transição de Estado (Combinacional)
    always @(*) begin
        case (state)
            S_FETCH: next_state = S_DECODE;
            S_DECODE: begin
                case (opcode)
                    OP_RTYPE: begin
                        case (funct)
                            F_ADD, F_SUB, F_AND, F_OR, F_SLT: next_state = S_R_EXECUTE;
                            F_JR:   next_state = S_JUMP_EXEC;
                            F_MULT: next_state = S_MULT_START;
                            F_DIV:  next_state = S_DIV_START;
                            F_MFHI: next_state = S_MFHI_WB;
                            F_MFLO: next_state = S_MFLO_WB;
                            default: next_state = S_FETCH; // Invalid funct
                        endcase
                    end
                    OP_LW, OP_SW: next_state = S_MEM_ADDR;
                    OP_ADDI, OP_SLTI: next_state = S_I_TYPE_EXEC;
                    OP_BEQ, OP_BNE: next_state = S_BRANCH_EXEC;
                    OP_LUI: next_state = S_LUI_EXEC;
                    OP_J:   next_state = S_JUMP_EXEC;
                    OP_JAL: next_state = S_JAL_EXEC;
                    default: next_state = S_FETCH; // Invalid opcode
                endcase
            end
            S_MEM_ADDR:   (opcode == OP_LW) ? (next_state = S_LW_READ) : (next_state = S_SW_WRITE);
            S_LW_READ:    next_state = S_LW_WB;
            S_LW_WB:      next_state = S_FETCH;
            S_SW_WRITE:   next_state = S_FETCH;
            S_R_EXECUTE:  next_state = S_R_WB;
            S_I_TYPE_EXEC:next_state = S_R_WB;
            S_LUI_EXEC:   next_state = S_R_WB;
            S_R_WB:       next_state = S_FETCH;
            S_BRANCH_EXEC:next_state = S_FETCH;

            S_JUMP_EXEC:  next_state = S_FETCH;
            S_JAL_EXEC:   next_state = S_FETCH;

            S_MULT_START: next_state = S_MULT_WAIT;
            S_MULT_WAIT:  (mult_done_in) ? (next_state = S_FETCH) : (next_state = S_MULT_WAIT);
            
            S_DIV_START:  next_state = S_DIV_WAIT;
            S_DIV_WAIT:   (div_done_in) ? (next_state = S_FETCH) : (next_state = S_DIV_WAIT);

            S_MFHI_WB:    next_state = S_FETCH;
            S_MFLO_WB:    next_state = S_FETCH;
            default:      next_state = S_FETCH;
        endcase
    end
    
    // Lógica de Atualização de Estado (Sequencial)
    always @(posedge clk or posedge reset) begin
        if (reset) state <= S_FETCH;
        else state <= next_state;
    end

    // =================================================================
    // --- LÓGICA DE GERAÇÃO DE SINAIS DE CONTROLE ---
    // =================================================================
    always @(*) begin
        // Valores padrão (inativos)
        PCWrite=0; PCWriteCond=0; IorD=0; MemRead=0; MemWrite=0; IRWrite=0; RegWrite=0;
        RegDst=2'b00; ALUSrcA=1; ALUSrcB=2'b00; PCSource=2'b00; ALUOp=4'b0000;
        HIWrite=0; LOWrite=0; MultStart=0; DivStart=0; InvalidOpcodeOut=0; WBDataSrc=2'b00;

        case (state)
            S_FETCH: begin // Ciclo 1: Busca de Instrução
                MemRead = 1; IRWrite = 1; PCWrite = 1;
                ALUSrcA = 0; ALUSrcB = 2'b01; ALUOp = 4'b0010; // ALUOp para Soma (PC+4)
            end
            S_DECODE: begin // Ciclo 2: Decodificação e cálculo de end. de branch
                ALUSrcA = 0; ALUSrcB = 2'b11; ALUOp = 4'b0010; // ALUOp Soma (PC+4 + imm<<2)
            end
            S_MEM_ADDR: begin // Ciclo 3 (lw/sw): Cálculo do endereço
                IorD = 1; ALUSrcB = 2'b10; ALUOp = 4'b0010; // ALUOp Soma (rs + imm)
            end
            S_LW_READ: begin // Ciclo 4 (lw): Leitura da memória de dados
                MemRead = 1; IorD = 1;
            end
            S_LW_WB: begin // Ciclo 5 (lw): Escrita no registrador
                RegWrite = 1; RegDst = 2'b00; WBDataSrc = 2'b01; // Dado vem do MDR
            end
            S_SW_WRITE: begin // Ciclo 4 (sw): Escrita na memória de dados
                MemWrite = 1; IorD = 1;
            end
            S_R_EXECUTE: begin // Ciclo 3 (R-type): Execução na ALU
                ALUSrcB = 2'b00;
                case (funct)
                    F_ADD: ALUOp = 4'b0010; // add
                    F_SUB: ALUOp = 4'b0110; // sub
                    F_AND: ALUOp = 4'b0000; // and
                    F_OR:  ALUOp = 4'b0001; // or
                    F_SLT: ALUOp = 4'b0111; // slt
                    default: InvalidOpcodeOut = 1;
                endcase
            end
            S_I_TYPE_EXEC: begin // Ciclo 3 (I-type): Execução
                ALUSrcB = 2'b10;
                case (opcode)
                    OP_ADDI: ALUOp = 4'b0010; // addi
                    OP_SLTI: ALUOp = 4'b0111; // slti
                endcase
            end
            S_LUI_EXEC: begin // Ciclo 3 (lui): Execução
                ALUSrcB = 2'b10; ALUOp = 4'b1100; // ALUOp customizado para LUI (imm << 16)
            end
            S_R_WB: begin // Ciclo 4 (R-type, I-type): Escrita no registrador
                RegWrite = 1;
                RegDst = (opcode == OP_RTYPE) ? 2'b01 : 2'b00; // rd para R-type, rt para I-type
                WBDataSrc = 2'b00; // Dado vem da ALU
            end
            S_BRANCH_EXEC: begin // Ciclo 3 (beq/bne): Comparação e desvio
                ALUSrcB = 2'b00; ALUOp = 4'b0110; // Subtração para comparar
                PCWriteCond = (opcode == OP_BEQ) ? 1'b1 : ~1'b1; // PCWrite se Z=1 para beq, Z=0 para bne
                PCSource = 2'b01;
            end
            S_JUMP_EXEC: begin // Ciclo 3 (j/jr): Desvio incondicional
                PCWrite = 1; PCSource = (opcode == OP_J) ? 2'b10 : 2'b11; // Endereço de J ou de JR
            end
            S_JAL_EXEC: begin // Ciclo 3 (jal): Salva PC+4 em $ra e salta
                PCWrite = 1; RegWrite = 1;
                PCSource = 2'b10; // Saltar para o endereço de J
                RegDst = 2'b10;   // Forçar escrita em $ra (31)
                WBDataSrc = 2'b00;  // Fonte é a ALU
                ALUSrcA = 0; ALUSrcB = 2'b01; ALUOp = 4'b0010; // Recalcula PC+4 para salvar
            end
            S_MULT_START: MultStart = 1;
            S_MULT_WAIT: if (mult_done_in) begin HIWrite=1; LOWrite=1; end
            S_DIV_START: DivStart = 1;
            S_DIV_WAIT: if (div_done_in) begin HIWrite=1; LOWrite=1; end
            S_MFHI_WB: begin RegWrite = 1; RegDst = 2'b01; WBDataSrc = 2'b10; end // Dado vem de HI
            S_MFLO_WB: begin RegWrite = 1; RegDst = 2'b01; WBDataSrc = 2'b11; end // Dado vem de LO
        endcase
    end
endmodule