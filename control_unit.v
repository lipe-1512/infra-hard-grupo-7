// control_unit.v
// FSM com estado de reset síncrono para controlar a inicialização do processador.
module control_unit (
    input wire clk, reset,
    input wire [5:0] opcode,
    input wire [5:0] funct,
    input wire mult_done_in, div_done_in,

    // Sinais de controle existentes
    output reg PCWrite, PCWriteCond, PCWriteCondNeg,
    output reg IorD, MemRead, MemWrite, IRWrite, RegWrite,
    output reg [1:0] RegDst,
    output reg ALUSrcA,
    output reg [1:0] ALUSrcB,
    output reg [1:0] PCSource,
    output reg [3:0] ALUOp,
    output reg HIWrite, LOWrite, MultStart, DivStart,
    output reg [2:0] WBDataSrc,
    output reg MemDataInSrc,
    
    // Novos sinais para o reset síncrono
    output reg PCClear,      // Sinal para limpar o Program Counter
    output reg RegsClear    // Sinal para limpar o Banco de Regs e HI/LO
);

    // Adiciona o novo estado S_RESET
    parameter S_RESET            = 0, S_FETCH            = 1, S_DECODE           = 2,
              S_MEM_ADDR         = 3, S_LW_READ          = 4, S_LW_WB            = 5,
              S_SW_WRITE         = 6, S_R_EXECUTE        = 7, S_R_WB             = 8,
              S_BRANCH_EXEC      = 9, S_JUMP_EXEC        = 10, S_I_TYPE_EXEC     = 11,
              S_SHIFT_EXEC       = 12, S_MULT_START      = 13, S_MULT_WAIT       = 14,
              S_DIV_START        = 15, S_DIV_WAIT        = 16, S_MFHI_WB         = 17,
              S_MFLO_WB          = 18, S_LB_READ         = 19, S_LB_WB           = 20,
              S_SB_READ_WORD     = 21, S_SB_MODIFY_WRITE = 22, S_JAL_EXEC = 23;


    // Opcodes e Functs
    localparam OP_RTYPE = 6'b000000; localparam OP_ADDI = 6'b001000;
    localparam OP_LW    = 6'b100011; localparam OP_SW   = 6'b101011;
    localparam OP_BEQ   = 6'b000100; localparam OP_BNE  = 6'b000101;
    localparam OP_LUI   = 6'b001111; localparam OP_J    = 6'b000010;
    localparam OP_JAL   = 6'b000011; localparam OP_LB   = 6'b100000;
    localparam OP_SB    = 6'b101000;

    localparam F_ADD  = 6'b100000; localparam F_SUB  = 6'b100010; localparam F_AND  = 6'b100100;
    localparam F_SLT  = 6'b101010; localparam F_JR   = 6'b001000;
    localparam F_MULT = 6'b011000; localparam F_DIV  = 6'b011010; localparam F_MFHI = 6'b010000;
    localparam F_MFLO = 6'b010010; localparam F_SLL  = 6'b000000; localparam F_SRA  = 6'b000011;

    reg [4:0] state, next_state;

    // Lógica de Transição de Estado
    always @(*) begin
        case (state)
            S_RESET: next_state = S_FETCH;
            S_FETCH: next_state = S_DECODE;
            S_DECODE: case (opcode)
                OP_RTYPE: case (funct)
                    F_ADD, F_SUB, F_AND, F_SLT: next_state = S_R_EXECUTE;
                    F_SLL, F_SRA: next_state = S_SHIFT_EXEC;
                    F_JR:   next_state = S_JUMP_EXEC;
                    F_MULT: next_state = S_MULT_START;
                    F_DIV:  next_state = S_DIV_START;
                    F_MFHI: next_state = S_MFHI_WB;
                    F_MFLO: next_state = S_MFLO_WB;
                    default: next_state = S_FETCH;
                endcase
                OP_LW, OP_SW, OP_LB, OP_SB: next_state = S_MEM_ADDR;
                OP_ADDI, OP_LUI: next_state = S_I_TYPE_EXEC;
                OP_BEQ, OP_BNE: next_state = S_BRANCH_EXEC;
                OP_J:     next_state = S_JUMP_EXEC;
                OP_JAL:   next_state = S_JAL_EXEC;
                default:  next_state = S_FETCH;
            endcase
            S_MEM_ADDR: case (opcode)
                OP_LW: next_state = S_LW_READ;
                OP_SW: next_state = S_SW_WRITE;
                OP_LB: next_state = S_LB_READ;
                OP_SB: next_state = S_SB_READ_WORD;
                default: next_state = S_FETCH;
            endcase
            S_R_EXECUTE, S_I_TYPE_EXEC, S_SHIFT_EXEC, S_MFHI_WB, S_MFLO_WB: next_state = S_R_WB;
            S_LW_READ:  next_state = S_LW_WB;
            S_LB_READ:  next_state = S_LB_WB;
            S_SB_READ_WORD: next_state = S_SB_MODIFY_WRITE;
            S_LW_WB, S_SW_WRITE, S_LB_WB, S_SB_MODIFY_WRITE, S_R_WB, S_BRANCH_EXEC, S_JUMP_EXEC, S_JAL_EXEC: next_state = S_FETCH;
            S_MULT_START: next_state = S_MULT_WAIT;
            S_MULT_WAIT:  if (mult_done_in) next_state = S_FETCH; else next_state = S_MULT_WAIT;
            S_DIV_START:  next_state = S_DIV_WAIT;
            S_DIV_WAIT:   if (div_done_in) next_state = S_FETCH; else next_state = S_DIV_WAIT;
            default:      next_state = S_RESET;
        endcase
    end
    
    // Atualização de Estado (Síncrono)
    always @(posedge clk) begin
        if (reset) state <= S_RESET;
        else state <= next_state;
    end

    // Lógica de Geração de Sinais
    always @(*) begin
        // Valores Padrão
        {PCWrite, PCWriteCond, PCWriteCondNeg, IorD, MemRead, MemWrite, IRWrite, RegWrite} = 8'b0;
        {RegDst, ALUSrcB, PCSource} = 6'b0;
        ALUSrcA = 1'b1; ALUOp = 4'b0; HIWrite = 0; LOWrite = 0;
        {MultStart, DivStart, MemDataInSrc, PCClear, RegsClear} = 5'b0;
        WBDataSrc = 3'b0;

        case (state)
            S_RESET: {PCClear, RegsClear} = 2'b11;
            S_FETCH: {MemRead, IRWrite, PCWrite, ALUSrcA} = 4'b1110; {ALUSrcB, ALUOp} = {2'b01, 4'b0010};
            S_DECODE: {ALUSrcB, ALUOp} = {2'b11, 4'b0010};
            S_MEM_ADDR: {ALUSrcB, ALUOp} = {2'b10, 4'b0010};
            S_LW_READ, S_LB_READ, S_SB_READ_WORD: {MemRead, IorD} = 2'b11;
            S_LW_WB: {RegWrite, RegDst, WBDataSrc} = {1'b1, 2'b00, 3'b001};
            S_LB_WB: {RegWrite, RegDst, WBDataSrc} = {1'b1, 2'b00, 3'b100};
            S_SW_WRITE, S_SB_MODIFY_WRITE: {MemWrite, IorD} = 2'b11; MemDataInSrc = (opcode == OP_SB);
            S_R_EXECUTE: ALUSrcB = 2'b00; case(funct) F_ADD: ALUOp=4'b0010; F_SUB: ALUOp=4'b0110; F_AND: ALUOp=4'b0000; F_SLT: ALUOp=4'b0111; endcase
            S_SHIFT_EXEC: {ALUSrcA, ALUSrcB} = 2'b00; case(funct) F_SLL: ALUOp=4'b1000; F_SRA: ALUOp=4'b1001; endcase
            S_I_TYPE_EXEC: {ALUSrcB, ALUOp} = {2'b10, (opcode==OP_LUI ? 4'b1100 : 4'b0010)};
            S_R_WB: RegWrite=1; RegDst=(opcode==OP_RTYPE)?2'b01:2'b00; WBDataSrc=(funct==F_MFHI)?3'b010:((funct==F_MFLO)?3'b011:3'b000);
            S_BRANCH_EXEC: {ALUSrcB, ALUOp, PCSource} = {2'b00, 4'b0110, 2'b01}; {PCWriteCond, PCWriteCondNeg} = {(opcode==OP_BEQ), (opcode==OP_BNE)};
            S_JUMP_EXEC: PCWrite=1; PCSource=(funct==F_JR)?2'b11:2'b10;
            S_JAL_EXEC: {PCWrite, RegWrite, PCSource, RegDst, ALUSrcA, ALUSrcB, ALUOp} = {1'b1, 1'b1, 2'b10, 2'b10, 1'b0, 2'b01, 4'b0010};
            S_MULT_START: MultStart=1;
            S_DIV_START: DivStart=1;
            S_MULT_WAIT: if(mult_done_in) {HIWrite, LOWrite} = 2'b11;
            S_DIV_WAIT: if(div_done_in) {HIWrite, LOWrite} = 2'b11;
        endcase
    end
endmodule