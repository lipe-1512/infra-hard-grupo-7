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
    output reg [1:0] MemAddrSrc,
    output reg MemDataInSrc,
    output reg PCClear,
    output reg RegsClear,
    output reg TempRegWrite,
    output reg MemtoRegA,

    // Saídas para tratamento de exceções
    output reg [2:0] exception_code,
    output reg [31:0] epc,
    output reg [31:0] new_pc,

    // Entrada do Program Counter
    input wire [31:0] pc
);

    // Estados da FSM - Limitados a 6 estados principais
    parameter S_RESET = 0,
              S_FETCH = 1,
              S_DECODE = 2,
              S_EXECUTE = 3,
              S_MEMORY = 4,
              S_WRITEBACK = 5;

    reg [2:0] state;
    reg [2:0] next_state;

    // Constantes de Opcode e Funct
    localparam OP_RTYPE = 6'b000000;
    localparam OP_ADDI = 6'b001000;
    localparam OP_LUI = 6'b001111;
    localparam OP_LW = 6'b100011;
    localparam OP_SW = 6'b101011;
    localparam OP_BEQ = 6'b000100;
    localparam OP_BNE = 6'b000101;
    localparam OP_J = 6'b000010;
    localparam OP_JAL = 6'b000011;
    localparam OP_XCHG = 6'b100011; // Instrução adicional
    localparam OP_SLLM = 6'b000001; // Instrução adicional
    localparam OP_DIV = 6'b000011;   // Adicionado conforme necessário

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

    // Variáveis auxiliares
    wire [4:0] rs, rt, rd;
    wire [15:0] immediate;
    wire alu_zero;
    wire overflow;
    wire branch_taken;
    wire jump_taken;
    wire jal_taken;

    // Lógica de transição de estado
    always @(posedge clk or posedge reset) begin
        if (reset)
            state <= S_RESET;
        else
            state <= next_state;
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
                MemAddrSrc = 0;
                MemDataInSrc = 0;
                PCClear = 0;
                RegsClear = 0;
                TempRegWrite = 0;
                MemtoRegA = 0;
                exception_code = 3'b000;
                epc = 32'h00000000;
                new_pc = 32'h00000000;
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
                MemAddrSrc = 0;
                MemDataInSrc = 0;
                PCClear = 0;
                RegsClear = 0;
                TempRegWrite = 0;
                MemtoRegA = 0;
                exception_code = 3'b000;
                epc = 32'h00000000;
                new_pc = 32'h00000000;
            end

            S_DECODE: begin
                next_state = S_EXECUTE;

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
                MemAddrSrc = 0;
                MemDataInSrc = 0;
                PCClear = 0;
                RegsClear = 0;
                TempRegWrite = 0;
                MemtoRegA = 0;
                exception_code = 3'b000;
                epc = 32'h00000000;
                new_pc = 32'h00000000;

                // Configurações condicionais com base no opcode
                case (opcode)
                    OP_RTYPE: begin
                        case (funct)
                            F_ADD, F_SUB, F_AND, F_OR, F_SLT: ALUOp = 4'b0001; // ADD, SUB, AND, OR, SLT
                            F_SLL, F_SRA: ALUOp = 4'b1000; // Deslocamento
                            F_JR: begin
                                PCWrite = 1;
                                PCSource = 2'b01;
                            end
                            F_MULT: begin
                                MultStart = 1;
                                RegWrite = 0;
                            end
                            F_DIV: begin
                                if (rt != 0) begin
                                    DivStart = 1;
                                    RegWrite = 0;
                                end else begin
                                    epc = pc;
                                    new_pc = { pc[31:8], 8'd254 };
                                    exception_code = 3'b010;
                                    next_state = S_FETCH;
                                end
                            end
                            F_MFHI: begin
                                RegWrite = 1;
                                RegDst = 2'b10; // $hi
                                WBDataSrc = 3'b010;
                            end
                            F_MFLO: begin
                                RegWrite = 1;
                                RegDst = 2'b11; // $lo
                                WBDataSrc = 3'b011;
                            end
                            default: next_state = S_FETCH;
                        endcase
                    end

                    OP_ADDI: begin
                        ALUSrcA = 1'b1;
                        ALUSrcB = 2'b10;
                        ALUOp = 4'b0001; // ADD
                        RegWrite = 1;
                        RegDst = 2'b01; // rt
                        WBDataSrc = 3'b000;
                    end

                    OP_LUI: begin
                        ALUSrcA = 1'b0;
                        ALUSrcB = 2'b11;
                        ALUOp = 4'b0000; // LUI
                        RegWrite = 1;
                        RegDst = 2'b01; // rt
                        WBDataSrc = 3'b000;
                    end

                    OP_LW: begin
                        ALUSrcA = 1'b1;
                        ALUSrcB = 2'b10;
                        ALUOp = 4'b0001; // ADD
                        IorD = 1;
                        MemRead = 1;
                        WBDataSrc = 3'b001;
                        RegWrite = 1;
                        RegDst = 2'b00;
                        MemtoRegA = 1;
                    end

                    OP_SW: begin
                        ALUSrcA = 1'b1;
                        ALUSrcB = 2'b10;
                        ALUOp = 4'b0001; // ADD
                        IorD = 1;
                        MemWrite = 1;
                        MemDataInSrc = 1;
                    end

                    OP_BEQ, OP_BNE: begin
                        ALUSrcA = 1'b1;
                        ALUSrcB = 2'b00;
                        ALUOp = 4'b0111; // BEQ/BNE
                        PCWriteCond = 1;
                        RegWrite = 0;
                    end

                    OP_J: begin
                        PCWrite = 1;
                        PCSource = 2'b10;
                        RegWrite = 0;
                        next_state = S_FETCH;
                    end

                    OP_JAL: begin
                        RegWrite = 1;
                        RegDst = 2'b11; // $ra
                        WBDataSrc = 3'b000;
                        PCWrite = 1;
                        PCSource = 2'b10;
                        next_state = S_FETCH;
                    end

                    OP_XCHG: begin
                        ALUSrcA = 1'b1;
                        ALUSrcB = 2'b00;
                        ALUOp = 4'b0000; // Nenhuma operação na XCHG
                        RegWrite = 1;
                        RegDst = 2'b00;
                        WBDataSrc = 3'b000;
                        TempRegWrite = 1;
                        MemtoRegA = 1;
                    end

                    OP_SLLM: begin
                        ALUSrcA = 1'b0;
                        ALUSrcB = 2'b00;
                        ALUOp = 4'b1000; // SLL
                        RegWrite = 1;
                        RegDst = 2'b00;
                        WBDataSrc = 3'b000;
                    end

                    default: next_state = S_FETCH;
                endcase
            end

            S_EXECUTE: begin
                next_state = S_MEMORY;

                case (opcode)
                    OP_RTYPE: begin
                        case (funct)
                            F_ADD, F_SUB, F_AND, F_OR, F_SLT: ALUOp = 4'b0001; // ADD, SUB, AND, OR, SLT
                            F_SLL, F_SRA: ALUOp = 4'b1000; // Deslocamento
                            default: ALUOp = 4'b0000;
                        endcase
                    end

                    OP_ADDI, OP_LUI: begin
                        ALUSrcA = 1'b1;
                        ALUSrcB = 2'b10;
                        ALUOp = 4'b0001; // ADD
                    end

                    OP_BEQ, OP_BNE: begin
                        ALUSrcA = 1'b1;
                        ALUSrcB = 2'b00;
                        ALUOp = 4'b0111; // BEQ/BNE
                        PCWriteCond = 1;
                    end

                    OP_J: begin
                        PCWrite = 1;
                        PCSource = 2'b10;
                    end

                    OP_JAL: begin
                        RegWrite = 1;
                        RegDst = 2'b11; // $ra
                        WBDataSrc = 3'b000;
                        PCWrite = 1;
                        PCSource = 2'b10;
                    end

                    OP_LW: begin
                        ALUSrcA = 1'b1;
                        ALUSrcB = 2'b10;
                        ALUOp = 4'b0001; // ADD
                        IorD = 1;
                        MemRead = 1;
                    end

                    OP_SW: begin
                        ALUSrcA = 1'b1;
                        ALUSrcB = 2'b10;
                        ALUOp = 4'b0001; // ADD
                        IorD = 1;
                        MemWrite = 1;
                        MemDataInSrc = 1;
                    end

                    OP_XCHG: begin
                        ALUSrcA = 1'b1;
                        ALUSrcB = 2'b00;
                        ALUOp = 4'b0000; // Nenhuma operação na XCHG
                        RegWrite = 1;
                        RegDst = 2'b00;
                        WBDataSrc = 3'b000;
                        TempRegWrite = 1;
                        MemtoRegA = 1;
                    end

                    OP_SLLM: begin
                        ALUSrcA = 1'b0;
                        ALUSrcB = 2'b00;
                        ALUOp = 4'b1000; // SLL
                        RegWrite = 1;
                        RegDst = 2'b00;
                        WBDataSrc = 3'b000;
                    end

                    default: next_state = S_FETCH;
                endcase
            end

            S_MEMORY: begin
                next_state = S_WRITEBACK;

                case (opcode)
                    OP_LW: begin
                        MemRead = 1;
                        WBDataSrc = 3'b001;
                        RegWrite = 1;
                    end

                    OP_SW: begin
                        MemWrite = 1;
                        MemDataInSrc = 1;
                    end

                    default: next_state = S_WRITEBACK;
                endcase
            end

            S_WRITEBACK: begin
                next_state = S_FETCH;
                RegWrite = 1;
                WBDataSrc = 3'b000;
            end

            default: next_state = S_FETCH;
        endcase
    end

    // Detecção de exceções - Atualização ocorre apenas na borda positiva do clock
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            exception_code <= 3'b000;
            epc <= 32'h00000000;
            new_pc <= 32'h00000000;
        end else begin
            if (opcode == 6'bXXXXXX) begin
                exception_code <= 3'b100; // Opcode Inválido
                epc <= pc;
                new_pc <= { pc[31:8], 8'd255 };
            end else if (opcode == OP_DIV && rt == 0) begin
                exception_code <= 3'b010; // Divisão por Zero
                epc <= pc;
                new_pc <= { pc[31:8], 8'd254 };
            end else if (overflow) begin
                exception_code <= 3'b001; // Overflow
                epc <= pc;
                new_pc <= { pc[31:8], 8'd253 };
            end else begin
                exception_code <= 3'b000;
                new_pc <= pc + 4;
            end
        end
    end

endmodule