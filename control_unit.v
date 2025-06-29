module control_unit (
    input wire clk,
    input wire reset,
    input wire [5:0] opcode,
    input wire [5:0] funct,

    input wire mult_done_in,
    input wire div_done_in,

    // Sinais de Controle da CPU
    output reg PCWrite,
    output reg PCWriteCond,
    output reg PCWriteCondNeg,
    output reg IorD,
    output reg MemRead,
    output reg MemWrite,
    output reg IRWrite,
    output reg RegWrite,
    output reg [1:0] RegDst,
    output reg ALUSrcA,
    output reg [1:0] ALUSrcB,
    output reg [1:0] PCSource,
    output reg [3:0] ALUOp,
    output reg HIWrite,
    output reg LOWrite,
    output reg MultStart,
    output reg DivStart,
    output reg [2:0] WBDataSrc,
    output reg MemDataInSrc,
    output reg PCClear,
    output reg RegsClear,
    output reg TempRegWrite,
    output reg MemtoRegA
);

    // Estados da FSM
    parameter S_RESET = 0,
              S_FETCH = 1,
              S_DECODE = 2,
              S_MEM_ADDR = 3,
              S_LW_READ = 4,
              S_LW_WB = 5,
              S_SW_WRITE = 6,
              S_R_EXECUTE = 7,
              S_R_WB = 8,
              S_BRANCH_EXEC = 9,
              S_JUMP_EXEC = 10,
              S_JAL_EXEC = 11,
              S_XCHG_READ_RS = 12,
              S_XCHG_SAVE_RS_READ_RT = 13,
              S_XCHG_WRITE_RS = 14,
              S_XCHG_WRITE_RT = 15,
              S_SLLM_READ = 16,
              S_SLLM_EXEC = 17,
              S_SLLM_WB = 18,
              S_MULT_START = 19,
              S_MULT_WAIT = 20,
              S_DIV_START = 21,
              S_DIV_WAIT = 22,
              S_MFHI_WB = 23,
              S_MFLO_WB = 24;

    reg [5:0] state;
    reg [5:0] next_state;

    // Definição dos opcodes e functs
    localparam OP_RTYPE = 6'b000000;
    localparam OP_ADDI = 6'b001000;
    localparam OP_LW = 6'b100011;
    localparam OP_SW = 6'b101011;
    localparam OP_BEQ = 6'b000100;
    localparam OP_BNE = 6'b000101;
    localparam OP_LUI = 6'b001111;
    localparam OP_J = 6'b000010;
    localparam OP_JAL = 6'b000011;
    localparam OP_LB = 6'b100000;
    localparam OP_SB = 6'b101000;
    localparam OP_SLLM = 6'b000001; // Instrução adicional

    localparam F_ADD = 6'b100000;
    localparam F_SUB = 6'b100010;
    localparam F_AND = 6'b100100;
    localparam F_OR = 6'b100101;
    localparam F_SLT = 6'b101010;
    localparam F_SLL = 6'b000000;
    localparam F_SRA = 6'b000011;
    localparam F_JR = 6'b001000;
    localparam F_MULT = 6'b011000;
    localparam F_DIV = 6'b011010;
    localparam F_MFHI = 6'b010000;
    localparam F_MFLO = 6'b010010;
    localparam F_XCHG = 6'b100011; // Instrução adicional

    // Vetor para armazenar o código da exceção
    reg [2:0] exception_code;

    // Sinal de exceção
    reg exception;

    // Endereço PC atual e novo
    reg [31:0] pc;
    reg [31:0] new_pc;

    // Sinal EPC (Endereço da instrução que causou exceção)
    reg [31:0] epc;

    // Lógica de transição de estado
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= S_RESET;
            pc <= 32'h00000000;
            exception <= 0;
            exception_code <= 3'b000;
        end else begin
            state <= next_state;
            pc <= new_pc;
        end
    end

    // Máquina de Estados (FSM)
    always @(*) begin
        case (state)
            S_RESET: begin
                next_state = S_FETCH;
                PCWrite = 0;
                PCWriteCond = 0;
                PCWriteCondNeg = 0;
                IorD = 0;
                MemRead = 0;
                MemWrite = 0;
                IRWrite = 0;
                RegWrite = 0;
                RegDst = 2'b00;
                ALUSrcA = 1'b0;
                ALUSrcB = 2'b00;
                PCSource = 2'b00;
                ALUOp = 4'b0000;
                HIWrite = 0;
                LOWrite = 0;
                MultStart = 0;
                DivStart = 0;
                WBDataSrc = 3'b000;
                MemDataInSrc = 0;
                PCClear = 0;
                RegsClear = 0;
                TempRegWrite = 0;
                MemtoRegA = 0;
            end

            S_FETCH: begin
                next_state = S_DECODE;
                PCWrite = 1;
                PCWriteCond = 0;
                PCWriteCondNeg = 0;
                IorD = 0;
                MemRead = 1;
                MemWrite = 0;
                IRWrite = 1;
                RegWrite = 0;
                RegDst = 2'b00;
                ALUSrcA = 1'b0;
                ALUSrcB = 2'b01;
                PCSource = 2'b00;
                ALUOp = 4'b0001;
                HIWrite = 0;
                LOWrite = 0;
                MultStart = 0;
                DivStart = 0;
                WBDataSrc = 3'b000;
                MemDataInSrc = 0;
                PCClear = 0;
                RegsClear = 0;
                TempRegWrite = 0;
                MemtoRegA = 0;
            end

            S_DECODE: begin
                case (opcode)
                    OP_RTYPE: case (funct)
                        F_ADD, F_SUB, F_AND, F_OR, F_SLT: next_state = S_R_EXECUTE;
                        F_SLL, F_SRA: next_state = S_SHIFT_EXEC;
                        F_JR: next_state = S_JUMP_EXEC;
                        F_MULT: next_state = S_MULT_START;
                        F_DIV: next_state = S_DIV_START;
                        F_MFHI: next_state = S_MFHI_WB;
                        F_MFLO: next_state = S_MFLO_WB;
                        F_XCHG: next_state = S_XCHG_READ_RS;
                        default: next_state = S_FETCH;
                    endcase
                    OP_LW: next_state = S_MEM_ADDR;
                    OP_SW: next_state = S_MEM_ADDR;
                    OP_LB: next_state = S_MEM_ADDR;
                    OP_SB: next_state = S_MEM_ADDR;
                    OP_ADDI: next_state = S_I_TYPE_EXEC;
                    OP_LUI: next_state = S_I_TYPE_EXEC;
                    OP_BEQ: next_state = S_BRANCH_EXEC;
                    OP_BNE: next_state = S_BRANCH_EXEC;
                    OP_J: next_state = S_JUMP_EXEC;
                    OP_JAL: next_state = S_JAL_EXEC;
                    OP_SLLM: next_state = S_SLLM_READ;
                    default: next_state = S_FETCH;
                endcase

                // Ajuste inicial dos sinais de controle
                PCWrite = 0;
                PCWriteCond = 0;
                PCWriteCondNeg = 0;
                IorD = 0;
                MemRead = 0;
                MemWrite = 0;
                IRWrite = 0;
                RegWrite = 0;
                RegDst = 2'b00;
                ALUSrcA = 1'b0;
                ALUSrcB = 2'b00;
                PCSource = 2'b00;
                ALUOp = 4'b0000;
                HIWrite = 0;
                LOWrite = 0;
                MultStart = 0;
                DivStart = 0;
                WBDataSrc = 3'b000;
                MemDataInSrc = 0;
                PCClear = 0;
                RegsClear = 0;
                TempRegWrite = 0;
                MemtoRegA = 0;
            end

            S_MEM_ADDR: begin
                ALUSrcA = 1'b1;
                ALUSrcB = 2'b10;
                ALUOp = 4'b0001; // ADD
                next_state = S_LW_READ;
                IorD = 1;
                MemRead = 1;
                MemWrite = 0;
                RegWrite = 0;
                RegDst = 2'b00;
                WBDataSrc = 3'b000;
                TempRegWrite = 0;
                MemtoRegA = 0;
            end

            S_LW_READ: begin
                MemRead = 1;
                next_state = S_LW_WB;
                WBDataSrc = 3'b001;
                RegWrite = 1;
                RegDst = 2'b00;
                MemtoRegA = 1;
            end

            S_LW_WB: begin
                next_state = S_FETCH;
                RegWrite = 1;
                WBDataSrc = 3'b001;
            end

            S_SW_WRITE: begin
                MemWrite = 1;
                MemDataInSrc = 1;
                next_state = S_FETCH;
            end

            S_R_EXECUTE: begin
                ALUSrcA = 1'b1;
                ALUSrcB = 2'b00;
                case (funct)
                    F_ADD: ALUOp = 4'b0001;
                    F_SUB: ALUOp = 4'b0010;
                    F_AND: ALUOp = 4'b0011;
                    F_OR: ALUOp = 4'b0012;
                    F_SLT: ALUOp = 4'b0110;
                    default: ALUOp = 4'b0000;
                endcase
                next_state = S_R_WB;
                RegWrite = 1;
                RegDst = 2'b00;
                WBDataSrc = 3'b000;
            end

            S_R_WB: begin
                next_state = S_FETCH;
                RegWrite = 1;
                WBDataSrc = 3'b000;
            end

            S_BRANCH_EXEC: begin
                ALUSrcA = 1'b1;
                ALUSrcB = 2'b00;
                ALUOp = 4'b0111; // BEQ/BNE
                PCWriteCond = 1;
                next_state = S_FETCH;
                RegWrite = 0;
                MemRead = 0;
                MemWrite = 0;
                WBDataSrc = 3'b000;
            end

            S_JUMP_EXEC: begin
                PCSource = 2'b10;
                next_state = S_FETCH;
                RegWrite = 0;
                MemRead = 0;
                MemWrite = 0;
                WBDataSrc = 3'b000;
            end

            S_JAL_EXEC: begin
                RegWrite = 1;
                RegDst = 2'b11; // $ra
                WBDataSrc = 3'b000;
                PCWrite = 1;
                PCSource = 2'b10;
                next_state = S_FETCH;
            end

            S_XCHG_READ_RS: begin
                MemRead = 1;
                IorD = 1;
                next_state = S_XCHG_SAVE_RS_READ_RT;
                TempRegWrite = 1;
                MemtoRegA = 0;
            end

            S_XCHG_SAVE_RS_READ_RT: begin
                MemRead = 1;
                IorD = 1;
                next_state = S_XCHG_WRITE_RS;
                MemtoRegA = 0;
            end

            S_XCHG_WRITE_RS: begin
                MemWrite = 1;
                IorD = 1;
                next_state = S_XCHG_WRITE_RT;
                MemDataInSrc = 1;
            end

            S_XCHG_WRITE_RT: begin
                MemWrite = 1;
                IorD = 1;
                next_state = S_FETCH;
                MemDataInSrc = 1;
            end

            S_SLLM_READ: begin
                MemRead = 1;
                IorD = 1;
                next_state = S_SLLM_EXEC;
                MemtoRegA = 0;
            end

            S_SLLM_EXEC: begin
                ALUSrcA = 1'b0;
                ALUSrcB = 2'b00;
                ALUOp = 4'b1000; // SLL
                next_state = S_SLLM_WB;
                WBDataSrc = 3'b000;
            end

            S_SLLM_WB: begin
                next_state = S_FETCH;
                RegWrite = 1;
                WBDataSrc = 3'b000;
            end

            S_MULT_START: begin
                MultStart = 1;
                next_state = S_MULT_WAIT;
                RegWrite = 0;
                MemRead = 0;
                MemWrite = 0;
                WBDataSrc = 3'b000;
            end

            S_MULT_WAIT: begin
                if (mult_done_in) begin
                    next_state = S_FETCH;
                    RegWrite = 1;
                    RegDst = 2'b10; // $hi
                    WBDataSrc = 3'b010;
                end else begin
                    next_state = S_MULT_WAIT;
                end
            end

            S_DIV_START: begin
                if (rt != 0) begin
                    DivStart = 1;
                    next_state = S_DIV_WAIT;
                end else begin
                    // Exceção: divisão por zero
                    exception = 1;
                    exception_code = 3'b010;
                    epc = pc;
                    new_pc = 32'h000000FF;
                    next_state = S_FETCH;
                end
            end

            S_DIV_WAIT: begin
                if (div_done_in) begin
                    next_state = S_FETCH;
                    RegWrite = 1;
                    RegDst = 2'b11; // $lo
                    WBDataSrc = 3'b011;
                end else begin
                    next_state = S_DIV_WAIT;
                end
            end

            S_MFHI_WB: begin
                RegWrite = 1;
                RegDst = 2'b10; // $hi
                WBDataSrc = 3'b010;
                next_state = S_FETCH;
            end

            S_MFLO_WB: begin
                RegWrite = 1;
                RegDst = 2'b11; // $lo
                WBDataSrc = 3'b011;
                next_state = S_FETCH;
            end

            default: next_state = S_FETCH;
        endcase
    end

    // Detecção de exceções
    always @(*) begin
        if (opcode == 6'bXXXXXX) begin
            exception = 1;
            exception_code = 3'b100; // Opcode Inválido
            epc = pc;
            new_pc = { pc[31:8], 8'd255 }; // Lê byte 255 da memória
        end else if (opcode == OP_DIV && rt == 0) begin
            exception = 1;
            exception_code = 3'b010; // Divisão por Zero
            epc = pc;
            new_pc = { pc[31:8], 8'd254 };
        end else if (overflow) begin
            exception = 1;
            exception_code = 3'b001; // Overflow
            epc = pc;
            new_pc = { pc[31:8], 8'd253 };
        end else begin
            exception = 0;
            new_pc = pc + 4;
        end
    end

endmodule