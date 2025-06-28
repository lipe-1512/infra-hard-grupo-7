//****************************************************************************
// Módulo: cpu_control_unit_complete
// Descrição: FSM expandida para controlar multiplicação, divisão, shifts e exceções.
//****************************************************************************
module cpu_control_unit_complete (
    input wire clk,
    input wire reset,
    input wire [5:0] opcode,
    input wire [5:0] funct,
    // Sinais de status do datapath (ENTRADAS MODIFICADAS)
    input wire mult_done,
    input wire div_done,
    input wire exception_detected,
    // Sinais de controle para o datapath
    output reg PCWrite, PCWriteCond,
    output reg IorD, MemRead, MemWrite, IRWrite, RegWrite,
    output reg AWrite, BWrite, ALUOutWrite,
    output reg HIWrite, LOWrite, EPCWrite,
    output reg [1:0] MemtoReg,
    output reg [1:0] RegDst,
    output reg [1:0] ALUSrcA,
    output reg [2:0] ALUSrcB, // Expandido para 3 bits
    output reg [2:0] ALUOp,
    output reg [2:0] ShifterOp,
    output reg ALUResultSrc, // 0: ALU, 1: Shifter
    output reg [2:0] PCSrc,      // Expandido para 3 bits
    output reg Start_mult_div
);
    // Estados da FSM Expandida
    parameter S_FETCH = 0, S_DECODE = 1, S_EXEC_R = 2, S_WB_R = 3;
    parameter S_MEM_ADDR = 4, S_MEM_READ = 5, S_MEM_WB = 6, S_MEM_WRITE = 7;
    parameter S_BRANCH = 8, S_JUMP = 9, S_JR = 10, S_ADDI_EXEC = 11, S_ADDI_WB = 12;
    parameter S_MULT_DIV_WAIT = 13, S_MFHI_WB = 14, S_MFLO_WB = 15, S_EXCEPTION = 16;

    reg [4:0] state, next_state;

    // Opcodes e Functs para decodificação
    localparam OP_RTYPE = 6'b000000;
    localparam OP_ADDI  = 6'b001000;
    localparam OP_LW    = 6'b100011;
    localparam OP_SW    = 6'b101011;
    localparam OP_BEQ   = 6'b000100;
    localparam OP_J     = 6'b000010;
    // Functs
    localparam F_ADD = 6'b100000, F_SUB  = 6'b100010, F_AND = 6'b100100;
    localparam F_OR  = 6'b100101, F_SLT  = 6'b101010, F_JR  = 6'b001000;
    localparam F_SLL = 6'b000000, F_SRL  = 6'b000010, F_SRA = 6'b000011;
    localparam F_MULT= 6'b011000, F_DIV  = 6'b011010;
    localparam F_MFHI= 6'b010000, F_MFLO = 6'b010010;
    
    // *** LÓGICA MOVIDA PARA DENTRO DO MÓDULO CORRETO ***
    wire mult_div_done_internal;
    assign mult_div_done_internal = (opcode == OP_RTYPE) ? ((funct == F_MULT) ? mult_done : div_done) : 1'b0;


    // Lógica sequencial da FSM
    always @(posedge clk or posedge reset) begin
        if (reset) state <= S_FETCH;
        else if (exception_detected) state <= S_EXCEPTION; // Exceção tem prioridade
        else state <= next_state;
    end

    // Lógica de próximo estado
    always @(*) begin
        case (state)
            S_FETCH: next_state = S_DECODE;
            S_DECODE: begin
                case (opcode)
                    OP_RTYPE: begin
                        case (funct)
                            F_ADD, F_SUB, F_AND, F_OR, F_SLT, F_SLL, F_SRL, F_SRA: next_state = S_EXEC_R;
                            F_JR:   next_state = S_JR;
                            F_MULT, F_DIV: next_state = S_MULT_DIV_WAIT;
                            F_MFHI: next_state = S_MFHI_WB;
                            F_MFLO: next_state = S_MFLO_WB;
                            default: next_state = S_FETCH; // Instrução inválida
                        endcase
                    end
                    OP_LW, OP_SW: next_state = S_MEM_ADDR;
                    OP_ADDI:      next_state = S_ADDI_EXEC;
                    OP_BEQ:       next_state = S_BRANCH;
                    OP_J:         next_state = S_JUMP;
                    default:      next_state = S_FETCH; // Instrução inválida
                endcase
            end
            S_MEM_ADDR:  next_state = (opcode == OP_LW) ? S_MEM_READ : S_MEM_WRITE;
            S_MEM_READ:  next_state = S_MEM_WB;
            S_EXEC_R:    next_state = S_WB_R;
            S_ADDI_EXEC: next_state = S_ADDI_WB;
            // *** LÓGICA ATUALIZADA PARA USAR O SINAL INTERNO ***
            S_MULT_DIV_WAIT: next_state = mult_div_done_internal ? S_FETCH : S_MULT_DIV_WAIT;
            S_EXCEPTION: next_state = S_FETCH;
            default:     next_state = S_FETCH; // Estados de WB, Jumps, etc.
        endcase
    end

    // Lógica de Saída da FSM
    always @(*) begin
        // Valores padrão (desativados)
        PCWrite=0; PCWriteCond=0; IorD=0; MemRead=0; MemWrite=0; IRWrite=0; RegWrite=0;
        AWrite=0; BWrite=0; ALUOutWrite=0; HIWrite=0; LOWrite=0; EPCWrite=0;
        MemtoReg=0; RegDst=0; ALUSrcA=0; ALUSrcB=0; ALUOp=0; ShifterOp=0;
        ALUResultSrc=0; PCSrc=0; Start_mult_div=0;

        case (state)
            S_FETCH:     begin PCWrite=1; MemRead=1; IRWrite=1; IorD=0; ALUSrcA=2'b00; ALUSrcB=3'b001; ALUOp=3'b001; PCSrc=3'b000; end
            S_DECODE:    begin AWrite=1; BWrite=1; ALUSrcA=2'b00; ALUSrcB=3'b011; ALUOp=3'b001; ALUOutWrite=1; end
            S_MEM_ADDR:  begin ALUSrcA=2'b01; ALUSrcB=3'b010; ALUOp=3'b001; ALUOutWrite=1; end
            S_MEM_READ:  begin MemRead=1; IorD=1; end
            S_MEM_WB:    begin RegWrite=1; MemtoReg=2'b01; RegDst=2'b00; end
            S_MEM_WRITE: begin MemWrite=1; IorD=1; end
            S_EXEC_R:    begin ALUResultSrc=(funct==F_SLL||funct==F_SRL||funct==F_SRA); ShifterOp={1'b0,funct[1:0]}; ALUOp=3'b010; ALUSrcA=2'b01; ALUSrcB=3'b000; ALUOutWrite=1; end
            S_WB_R:      begin RegWrite=1; MemtoReg=2'b00; RegDst=2'b01; end
            S_ADDI_EXEC: begin ALUSrcA=2'b01; ALUSrcB=3'b010; ALUOp=3'b001; ALUOutWrite=1; end
            S_ADDI_WB:   begin RegWrite=1; MemtoReg=2'b00; RegDst=2'b00; end
            S_BRANCH:    begin ALUSrcA=2'b01; ALUSrcB=3'b000; ALUOp=3'b010; PCWriteCond=1; PCSrc=3'b001; end
            S_JUMP:      begin PCWrite=1; PCSrc=3'b010; end
            S_JR:        begin PCWrite=1; PCSrc=3'b011; end
            // *** LÓGICA ATUALIZADA PARA USAR O SINAL INTERNO ***
            S_MULT_DIV_WAIT: begin Start_mult_div=1; if(mult_div_done_internal) begin HIWrite=1; LOWrite=1; end end
            S_MFHI_WB:   begin RegWrite=1; MemtoReg=2'b10; RegDst=2'b01; end
            S_MFLO_WB:   begin RegWrite=1; MemtoReg=2'b11; RegDst=2'b01; end
            S_EXCEPTION: begin EPCWrite=1; PCWrite=1; PCSrc=3'b100; end // Salva EPC, pula para handler
        endcase
    end
endmodule


//****************************************************************************
// Módulos de Multiplexadores (MUX) - Sem alterações
//****************************************************************************
module mux2_1_32b(input [31:0] d0, d1, input sel, output [31:0] y); assign y = sel ? d1 : d0; endmodule
module mux4_1_32b(input [31:0] d0, d1, d2, d3, input [1:0] sel, output [31:0] y); assign y = sel[1] ? (sel[0] ? d3 : d2) : (sel[0] ? d1 : d0); endmodule
module mux2_1_5b(input [4:0] d0, d1, input sel, output [4:0] y); assign y = sel ? d1 : d0; endmodule
