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
    output reg [1:0] WBDataSrc // 00=ALUOut, 01=MDR, 10=HI, 11=LO
);

    parameter S_FETCH        = 0,  S_DECODE       = 1,
              S_MEM_ADDR     = 2,  S_LW_READ      = 3,  S_LW_WB        = 4,
              S_SW_WRITE     = 5,  S_R_EXECUTE    = 6,  S_R_WB         = 7,
              S_BRANCH_EXEC  = 8,  S_JUMP_EXEC    = 9,  S_I_TYPE_EXEC  = 10,
              S_LUI_EXEC     = 11, S_JAL_EXEC     = 12,
              S_MULT_START   = 13, S_MULT_WAIT    = 14, S_DIV_START    = 15,
              S_DIV_WAIT     = 16, S_MFHI_WB      = 17, S_MFLO_WB      = 18;

    localparam OP_RTYPE = 6'b000000; localparam OP_ADDI = 6'b001000;
    localparam OP_SLTI  = 6'b001010; localparam OP_LW   = 6'b100011;
    localparam OP_SW    = 6'b101011; localparam OP_BEQ  = 6'b000100;
    localparam OP_BNE   = 6'b000101; localparam OP_LUI  = 6'b001111;
    localparam OP_J     = 6'b000010; localparam OP_JAL  = 6'b000011;

    localparam F_ADD  = 6'b100000; localparam F_SUB  = 6'b100010; localparam F_AND  = 6'b100100;
    localparam F_OR   = 6'b100101; localparam F_SLT  = 6'b101010; localparam F_JR   = 6'b001000;
    localparam F_MULT = 6'b011000; localparam F_DIV  = 6'b011010; localparam F_MFHI = 6'b010000;
    localparam F_MFLO = 6'b010010;

    reg [4:0] state, next_state;

    always @(*) begin // Lógica de Transição de Estado
        case (state)
            S_FETCH: next_state = S_DECODE;
            S_DECODE: begin
                case (opcode)
                    OP_RTYPE: begin
                        case (funct)
                            F_ADD, F_SUB, F_AND, F_OR, F_SLT: next_state = S_R_EXECUTE;
                            F_JR:   next_state = S_JUMP_EXEC;   F_MULT: next_state = S_MULT_START;
                            F_DIV:  next_state = S_DIV_START;   F_MFHI: next_state = S_MFHI_WB;
                            F_MFLO: next_state = S_MFLO_WB;     default: next_state = S_FETCH;
                        endcase
                    end
                    OP_LW, OP_SW:      next_state = S_MEM_ADDR;
                    OP_ADDI, OP_SLTI:  next_state = S_I_TYPE_EXEC;
                    OP_BEQ, OP_BNE:    next_state = S_BRANCH_EXEC;
                    OP_LUI:            next_state = S_LUI_EXEC;
                    OP_J:              next_state = S_JUMP_EXEC;
                    OP_JAL:            next_state = S_JAL_EXEC;
                    default:           next_state = S_FETCH;
                endcase
            end
            S_MEM_ADDR:   (opcode == OP_LW) ? (next_state = S_LW_READ) : (next_state = S_SW_WRITE);
            S_LW_READ:    next_state = S_LW_WB;       S_LW_WB:      next_state = S_FETCH;
            S_SW_WRITE:   next_state = S_FETCH;
            S_R_EXECUTE:  next_state = S_R_WB;        S_I_TYPE_EXEC:next_state = S_R_WB;
            S_LUI_EXEC:   next_state = S_R_WB;        S_R_WB:       next_state = S_FETCH;
            S_BRANCH_EXEC:next_state = S_FETCH;       S_JUMP_EXEC:  next_state = S_FETCH;
            S_JAL_EXEC:   next_state = S_FETCH;
            S_MULT_START: next_state = S_MULT_WAIT;
            S_MULT_WAIT:  (mult_done_in) ? (next_state = S_FETCH) : (next_state = S_MULT_WAIT);
            S_DIV_START:  next_state = S_DIV_WAIT;
            S_DIV_WAIT:   (div_done_in) ? (next_state = S_FETCH) : (next_state = S_DIV_WAIT);
            S_MFHI_WB:    next_state = S_FETCH;       S_MFLO_WB:    next_state = S_FETCH;
            default:      next_state = S_FETCH;
        endcase
    end
    
    always @(posedge clk or posedge reset) begin // Atualização de Estado
        if (reset) state <= S_FETCH;
        else state <= next_state;
    end

    always @(*) begin // Lógica de Geração de Sinais
        PCWrite=0; PCWriteCond=0; PCWriteCondNeg=0; IorD=0; MemRead=0; MemWrite=0;
        IRWrite=0; RegWrite=0; RegDst=2'b00; ALUSrcA=1; ALUSrcB=2'b00;
        PCSource=2'b00; ALUOp=4'b0000; HIWrite=0; LOWrite=0;
        MultStart=0; DivStart=0; InvalidOpcodeOut=0; WBDataSrc=2'b00;

        case (state)
            S_FETCH:      begin MemRead=1; IRWrite=1; PCWrite=1; ALUSrcA=0; ALUSrcB=2'b01; ALUOp=4'b0010; end
            S_DECODE:     begin ALUSrcA=0; ALUSrcB=2'b11; ALUOp=4'b0010; end
            S_MEM_ADDR:   begin IorD=1; ALUSrcB=2'b10; ALUOp=4'b0010; end
            S_LW_READ:    begin MemRead=1; IorD=1; end
            S_LW_WB:      begin RegWrite=1; RegDst=2'b00; WBDataSrc=2'b01; end
            S_SW_WRITE:   begin MemWrite=1; IorD=1; end
            S_R_EXECUTE:  begin ALUSrcB=2'b00; case(funct) F_ADD: ALUOp=4'b0010; F_SUB: ALUOp=4'b0110; F_AND: ALUOp=4'b0000; F_OR: ALUOp=4'b0001; F_SLT: ALUOp=4'b0111; default: InvalidOpcodeOut=1; endcase end
            S_I_TYPE_EXEC:begin ALUSrcB=2'b10; case(opcode) OP_ADDI: ALUOp=4'b0010; OP_SLTI: ALUOp=4'b0111; endcase end
            S_LUI_EXEC:   begin ALUSrcB=2'b10; ALUOp=4'b1100; end
            S_R_WB:       begin RegWrite=1; RegDst=(opcode==OP_RTYPE)?2'b01:2'b00; WBDataSrc=2'b00; end
            S_BRANCH_EXEC:begin ALUSrcB=2'b00; ALUOp=4'b0110; PCSource=2'b01; if(opcode==OP_BEQ)PCWriteCond=1; else PCWriteCondNeg=1; end
            S_JUMP_EXEC:  begin PCWrite=1; PCSource=(opcode==OP_J)?2'b10:2'b11; end
            S_JAL_EXEC:   begin PCWrite=1; RegWrite=1; PCSource=2'b10; RegDst=2'b10; WBDataSrc=2'b00; ALUSrcA=0; ALUSrcB=2'b01; ALUOp=4'b0010; end
            S_MULT_START: MultStart=1;
            S_MULT_WAIT:  if(mult_done_in)begin HIWrite=1; LOWrite=1; end
            S_DIV_START:  DivStart=1;
            S_DIV_WAIT:   if(div_done_in)begin HIWrite=1; LOWrite=1; end
            S_MFHI_WB:    begin RegWrite=1; RegDst=2'b01; WBDataSrc=2'b10; end
            S_MFLO_WB:    begin RegWrite=1; RegDst=2'b01; WBDataSrc=2'b11; end
        endcase
    end
endmodule