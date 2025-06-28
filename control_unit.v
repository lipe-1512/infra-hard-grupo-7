// control_unit.v
// Unidade de Controle FINAL e POLIDA. Gerencia o fluxo para TODAS as instruções.
module control_unit (
    input wire clk, reset,
    input wire [5:0] opcode,
    input wire [5:0] funct,
    input wire mult_done_in, div_done_in,

    output reg PCWrite, PCWriteCond, PCWriteCondNeg,
    output reg IorD, MemRead, MemWrite, IRWrite, RegWrite,
    output reg [1:0] RegDst,
    output reg ALUSrcA,
    output reg [1:0] ALUSrcB,
    output reg [1:0] PCSource,
    output reg [3:0] ALUOp,
    output reg HIWrite, LOWrite, MultStart, DivStart,
    output reg InvalidOpcodeOut,
    output reg [2:0] WBDataSrc,      // 000=ALUOut, 001=MDR, 010=HI, 011=LO, 100=ByteFromMDR
    output reg MemDataInSrc,   // 0=RegB, 1=ModifiedMDR for sb
    output reg ByteOpEnable    // Habilita lógica de byte no datapath para lb/sb
);

    // Estados da FSM
    parameter S_FETCH            = 0, S_DECODE           = 1,
              S_MEM_ADDR         = 2, S_LW_READ          = 3, S_LW_WB            = 4,
              S_SW_WRITE         = 5, S_R_EXECUTE        = 6, S_R_WB             = 7,
              S_BRANCH_EXEC      = 8, S_JUMP_EXEC        = 9, S_I_TYPE_EXEC      = 10,
              S_LUI_EXEC         = 11,S_JAL_EXEC         = 12,
              S_MULT_START       = 13,S_MULT_WAIT        = 14,S_DIV_START        = 15,
              S_DIV_WAIT         = 16,S_MFHI_WB          = 17,S_MFLO_WB          = 18,
              S_SHIFT_EXEC       = 19,
              S_LB_READ          = 20,S_LB_WB            = 21,
              S_SB_READ_WORD     = 22,S_SB_MODIFY_WRITE  = 23;

    // Opcodes e Functs
    localparam OP_RTYPE = 6'b000000; localparam OP_ADDI = 6'b001000;
    localparam OP_LW    = 6'b100011; localparam OP_SW   = 6'b101011;
    localparam OP_BEQ   = 6'b000100; localparam OP_BNE  = 6'b000101;
    localparam OP_LUI   = 6'b001111; localparam OP_J    = 6'b000010;
    localparam OP_JAL   = 6'b000011; localparam OP_LB   = 6'b100000;
    localparam OP_SB    = 6'b101000;

    localparam F_ADD  = 6'b100000; localparam F_SUB  = 6'b100010; localparam F_AND  = 6'b100100;
    localparam F_OR   = 6'b100101; localparam F_SLT  = 6'b101010; localparam F_JR   = 6'b001000;
    localparam F_MULT = 6'b011000; localparam F_DIV  = 6'b011010; localparam F_MFHI = 6'b010000;
    localparam F_MFLO = 6'b010010; localparam F_SLL  = 6'b000000; localparam F_SRA  = 6'b000011;

    reg [4:0] state, next_state;

    // Lógica de Transição de Estado
    always @(*) begin
        case (state)
            S_FETCH: next_state = S_DECODE;
            S_DECODE: begin
                case (opcode)
                    OP_RTYPE: begin
                        case (funct)
                            F_ADD, F_SUB, F_AND, F_OR, F_SLT: next_state = S_R_EXECUTE;
                            F_SLL, F_SRA: next_state = S_SHIFT_EXEC;
                            F_JR:   next_state = S_JUMP_EXEC;   F_MULT: next_state = S_MULT_START;
                            F_DIV:  next_state = S_DIV_START;   F_MFHI: next_state = S_MFHI_WB;
                            F_MFLO: next_state = S_MFLO_WB;     default: next_state = S_FETCH; // Invalid funct
                        endcase
                    end
                    OP_LW:    next_state = S_MEM_ADDR;
                    OP_SW:    next_state = S_MEM_ADDR;
                    OP_LB:    next_state = S_MEM_ADDR;
                    OP_SB:    next_state = S_MEM_ADDR;
                    OP_ADDI:  next_state = S_I_TYPE_EXEC;
                    OP_BEQ, OP_BNE: next_state = S_BRANCH_EXEC;
                    OP_LUI:   next_state = S_LUI_EXEC;
                    OP_J:     next_state = S_JUMP_EXEC;
                    OP_JAL:   next_state = S_JAL_EXEC;
                    default:  next_state = S_FETCH; // Invalid Opcode
                endcase
            end
            S_MEM_ADDR: begin
                case (opcode)
                    OP_LW: next_state = S_LW_READ;
                    OP_SW: next_state = S_SW_WRITE;
                    OP_LB: next_state = S_LB_READ;
                    OP_SB: next_state = S_SB_READ_WORD; // Inicia Read-Modify-Write
                    default: next_state = S_FETCH;
                endcase
            end
            S_LW_READ:  next_state = S_LW_WB;
            S_LW_WB:    next_state = S_FETCH;
            S_SW_WRITE: next_state = S_FETCH;
            S_LB_READ:  next_state = S_LB_WB;
            S_LB_WB:    next_state = S_FETCH;
            S_SB_READ_WORD:    next_state = S_SB_MODIFY_WRITE;
            S_SB_MODIFY_WRITE: next_state = S_FETCH;
            S_R_EXECUTE:  next_state = S_R_WB;
            S_SHIFT_EXEC: next_state = S_R_WB;
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
    
    // Atualização de Estado
    always @(posedge clk or posedge reset) begin
        if (reset) state <= S_FETCH;
        else state <= next_state;
    end

    // Lógica de Geração de Sinais
    always @(*) begin
        PCWrite=0; PCWriteCond=0; PCWriteCondNeg=0; IorD=0; MemRead=0; MemWrite=0;
        IRWrite=0; RegWrite=0; RegDst=2'b00; ALUSrcA=1; ALUSrcB=2'b00;
        PCSource=2'b00; ALUOp=4'b0000; HIWrite=0; LOWrite=0;
        MultStart=0; DivStart=0; InvalidOpcodeOut=0; WBDataSrc=3'b000;
        MemDataInSrc=0; ByteOpEnable=0;

        case (state)
            S_FETCH:         begin MemRead=1; IRWrite=1; PCWrite=1; ALUSrcA=0; ALUSrcB=2'b01; ALUOp=4'b0010; end // PC+4
            S_DECODE:        begin ALUSrcA=0; ALUSrcB=2'b11; ALUOp=4'b0010; end // Branch Addr Calc
            S_MEM_ADDR:      begin ALUSrcB=2'b10; ALUOp=4'b0010; end // Addr = A + imm
            S_LW_READ:       begin MemRead=1; IorD=1; end
            S_LW_WB:         begin RegWrite=1; RegDst=2'b00; WBDataSrc=3'b001; end // WB from MDR
            S_SW_WRITE:      begin MemWrite=1; IorD=1; MemDataInSrc=0; end
            S_LB_READ:       begin MemRead=1; IorD=1; end
            S_LB_WB:         begin RegWrite=1; RegDst=2'b00; WBDataSrc=3'b100; ByteOpEnable=1; end // WB byte from MDR
            S_SB_READ_WORD:  begin MemRead=1; IorD=1; end // Read a palavra para modificar
            S_SB_MODIFY_WRITE: begin MemWrite=1; IorD=1; MemDataInSrc=1; ByteOpEnable=1; end // Escreve palavra modificada
            S_R_EXECUTE:     begin ALUSrcB=2'b00; case(funct) F_ADD: ALUOp=4'b0010; F_SUB: ALUOp=4'b0110; F_AND: ALUOp=4'b0000; F_OR: ALUOp=4'b0001; F_SLT: ALUOp=4'b0111; default: InvalidOpcodeOut=1; endcase end
            S_SHIFT_EXEC:    begin ALUSrcA=0; case(funct) F_SLL: ALUOp=4'b1000; F_SRA: ALUOp=4'b1001; endcase end // Use ALU for shift
            S_I_TYPE_EXEC:   begin ALUSrcB=2'b10; ALUOp=4'b0010; end // ADDI
            S_LUI_EXEC:      begin ALUSrcB=2'b10; ALUOp=4'b1100; end // LUI custom ALU op
            S_R_WB:          begin RegWrite=1; RegDst=(opcode==OP_RTYPE)?2'b01:2'b00; WBDataSrc=3'b000; end // WB from ALU
            S_BRANCH_EXEC:   begin ALUSrcB=2'b00; ALUOp=4'b0110; PCSource=2'b01; if(opcode==OP_BEQ)PCWriteCond=1; else PCWriteCondNeg=1; end
            S_JUMP_EXEC:     begin PCWrite=1; PCSource=(funct==F_JR)?2'b11:2'b10; end // J or JR
            S_JAL_EXEC:      begin PCWrite=1; RegWrite=1; PCSource=2'b10; RegDst=2'b10; WBDataSrc=3'b000; ALUSrcA=0; ALUSrcB=2'b01; ALUOp=4'b0010; end // Save PC+4 in $31
            S_MULT_START:    begin MultStart=1; end
            S_MULT_WAIT:     if(mult_done_in)begin HIWrite=1; LOWrite=1; end
            S_DIV_START:     begin DivStart=1; end
            S_DIV_WAIT:      if(div_done_in)begin HIWrite=1; LOWrite=1; end
            S_MFHI_WB:       begin RegWrite=1; RegDst=2'b01; WBDataSrc=3'b010; end // WB from HI
            S_MFLO_WB:       begin RegWrite=1; RegDst=2'b01; WBDataSrc=3'b011; end // WB from LO
        endcase
    end
endmodule