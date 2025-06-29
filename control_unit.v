// control_unit.v
// FSM atualizada com estado de reset síncrono.
module control_unit (
    input wire clk, reset, // 'reset' agora é usado para forçar a FSM para o estado S_RESET
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
    output reg PCClear,      // Clear para o PC
    output reg RegsClear    // Clear para Banco de Registradores, HI, LO
);

    // Adiciona o novo estado S_RESET no início
    parameter S_RESET            = 0, S_FETCH            = 1, S_DECODE           = 2,
              S_MEM_ADDR         = 3, S_LW_READ          = 4, S_LW_WB            = 5,
              S_SW_WRITE         = 6, S_R_EXECUTE        = 7, S_R_WB             = 8,
              S_BRANCH_EXEC      = 9, S_JUMP_EXEC        = 10, S_I_TYPE_EXEC     = 11,
              S_LUI_EXEC         = 12, S_JAL_EXEC        = 13,
              S_MULT_START       = 14, S_MULT_WAIT       = 15, S_DIV_START       = 16,
              S_DIV_WAIT         = 17, S_MFHI_WB         = 18, S_MFLO_WB         = 19,
              S_SHIFT_EXEC       = 20,
              S_LB_READ          = 21, S_LB_WB           = 22,
              S_SB_READ_WORD     = 23, S_SB_MODIFY_WRITE = 24;

    // Opcodes e Functs (sem alteração)
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

    // Lógica de Transição de Estado (com S_RESET)
    always @(*) begin
        case (state)
            S_RESET: next_state = S_FETCH; // Após um ciclo de reset, vai para Fetch
            S_FETCH: next_state = S_DECODE;
            S_DECODE: begin
                case (opcode)
                    OP_RTYPE: begin
                        case (funct)
                            F_ADD, F_SUB, F_AND, F_SLT: next_state = S_R_EXECUTE;
                            F_SLL, F_SRA: next_state = S_SHIFT_EXEC;
                            F_JR:   next_state = S_JUMP_EXEC;   F_MULT: next_state = S_MULT_START;
                            F_DIV:  next_state = S_DIV_START;   F_MFHI: next_state = S_MFHI_WB;
                            F_MFLO: next_state = S_MFLO_WB;     default: next_state = S_FETCH;
                        endcase
                    end
                    OP_LW, OP_SW, OP_LB, OP_SB: next_state = S_MEM_ADDR;
                    OP_ADDI, OP_LUI:  next_state = S_I_TYPE_EXEC;
                    OP_BEQ, OP_BNE: next_state = S_BRANCH_EXEC;
                    OP_J:     next_state = S_JUMP_EXEC;
                    OP_JAL:   next_state = S_JAL_EXEC;
                    default:  next_state = S_FETCH;
                endcase
            end
            S_MEM_ADDR: begin
                case (opcode)
                    OP_LW: next_state = S_LW_READ;
                    OP_SW: next_state = S_SW_WRITE;
                    OP_LB: next_state = S_LB_READ;
                    OP_SB: next_state = S_SB_READ_WORD;
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
            S_R_WB:       next_state = S_FETCH;
            S_BRANCH_EXEC:next_state = S_FETCH;
            S_JUMP_EXEC:  next_state = S_FETCH;
            S_JAL_EXEC:   next_state = S_FETCH;
            S_MULT_START: next_state = S_MULT_WAIT;
            S_MULT_WAIT:  if (mult_done_in) next_state = S_FETCH; else next_state = S_MULT_WAIT;
            S_DIV_START:  next_state = S_DIV_WAIT;
            S_DIV_WAIT:   if (div_done_in) next_state = S_FETCH; else next_state = S_DIV_WAIT;
            S_MFHI_WB:    next_state = S_R_WB;
            S_MFLO_WB:    next_state = S_R_WB;
            default:      next_state = S_RESET; // Estado padrão seguro
        endcase
    end
    
    // Atualização de Estado (agora síncrono e sensível ao reset)
    always @(posedge clk) begin
        if (reset) // Agora o reset externo leva para o estado S_RESET
            state <= S_RESET;
        else
            state <= next_state;
    end

    // Lógica de Geração de Sinais (com S_RESET)
    always @(*) begin
        // Valores Padrão (incluindo os novos sinais de clear)
        PCWrite=0; PCWriteCond=0; PCWriteCondNeg=0; IorD=0; MemRead=0; MemWrite=0;
        IRWrite=0; RegWrite=0; RegDst=2'b00; ALUSrcA=1'b1; ALUSrcB=2'b00;
        PCSource=2'b00; ALUOp=4'b0000; HIWrite=0; LOWrite=0;
        MultStart=0; DivStart=0; WBDataSrc=3'b000; MemDataInSrc=0;
        PCClear=0; RegsClear=0; // Padrão é não limpar

        case (state)
            S_RESET: begin // ** NOVO ESTADO **
                // Ativa todos os sinais de clear por um ciclo
                PCClear = 1;
                RegsClear = 1; // Este sinal será usado para limpar Banco de Regs, HI e LO
            end
            S_FETCH:           begin MemRead=1; IRWrite=1; PCWrite=1; ALUSrcA=0; ALUSrcB=2'b01; ALUOp=4'b0010; end
            S_DECODE:          begin ALUSrcB=2'b11; ALUOp=4'b0010; end
            S_MEM_ADDR:        begin ALUSrcB=2'b10; ALUOp=4'b0010; end
            S_LW_READ:         begin MemRead=1; IorD=1; end
            S_LW_WB:           begin RegWrite=1; RegDst=2'b00; WBDataSrc=3'b001; end
            S_SW_WRITE:        begin MemWrite=1; IorD=1; end
            S_LB_READ:         begin MemRead=1; IorD=1; end
            S_LB_WB:           begin RegWrite=1; RegDst=2'b00; WBDataSrc=3'b100; end
            S_SB_READ_WORD:    begin MemRead=1; IorD=1; end
            S_SB_MODIFY_WRITE: begin MemWrite=1; IorD=1; MemDataInSrc=1; end
            S_R_EXECUTE:       begin ALUSrcB=2'b00; case(funct) F_ADD: ALUOp=4'b0010; F_SUB: ALUOp=4'b0110; F_AND: ALUOp=4'b0000; F_SLT: ALUOp=4'b0111; endcase end
            S_SHIFT_EXEC:      begin ALUSrcA=0; ALUSrcB=2'b00; case(funct) F_SLL: ALUOp=4'b1000; F_SRA: ALUOp=4'b1001; endcase end
            S_I_TYPE_EXEC:     begin ALUSrcB=2'b10; case(opcode) OP_ADDI: ALUOp=4'b0010; OP_LUI: ALUOp=4'b1100; endcase end
            S_R_WB:            begin RegWrite=1; RegDst=(opcode==OP_RTYPE)?2'b01:2'b00; WBDataSrc=(funct==F_MFHI)?3'b010:((funct==F_MFLO)?3'b011:3'b000); end
            S_BRANCH_EXEC:     begin ALUSrcB=2'b00; ALUOp=4'b0110; PCSource=2'b01; if(opcode==OP_BEQ)PCWriteCond=1; else PCWriteCondNeg=1; end
            S_JUMP_EXEC:       begin PCWrite=1; PCSource=(funct==F_JR)?2'b11:2'b10; end
            S_JAL_EXEC:        begin PCWrite=1; RegWrite=1; PCSource=2'b10; RegDst=2'b10; ALUSrcA=0; ALUSrcB=2'b01; ALUOp=4'b0010; end
            S_MULT_START:      begin MultStart=1; end
            S_MULT_WAIT:       if(mult_done_in) begin HIWrite=1; LOWrite=1; end
            S_DIV_START:       begin DivStart=1; end
            S_DIV_WAIT:        if(div_done_in) begin HIWrite=1; LOWrite=1; end
        endcase
    end
endmodule