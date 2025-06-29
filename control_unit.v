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
    output reg PCWriteCondNeg, // Não utilizado no datapath corrigido, mas mantido
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
    output reg PCClear,         // Não utilizado no datapath corrigido, mas mantido
    output reg RegsClear,       // Não utilizado no datapath corrigido, mas mantido
    output reg TempRegWrite,    // Não utilizado no datapath corrigido, mas mantido
    output reg MemtoRegA        // Não utilizado no datapath corrigido, mas mantido
);

    // Estados da FSM - 0 a 5
    parameter S_RESET     = 3'd0,
              S_FETCH     = 3'd1,
              S_DECODE    = 3'd2,
              S_EXECUTE   = 3'd3,
              S_MEMORY    = 3'd4,
              S_WRITEBACK = 3'd5;

    reg [2:0] state;
    reg [2:0] next_state;

    // Constantes de Opcode e Funct
    localparam OP_RTYPE = 6'b000000;
    localparam OP_ADDI  = 6'b001000;
    localparam OP_LUI   = 6'b001111;
    localparam OP_LW    = 6'b100011;
    localparam OP_SW    = 6'b101011;
    localparam OP_BEQ   = 6'b000100;
    localparam OP_BNE   = 6'b000101;
    localparam OP_J     = 6'b000010;
    localparam OP_JAL   = 6'b000011;
    
    localparam F_ADD    = 6'b100000;
    localparam F_SUB    = 6'b100010;
    localparam F_AND    = 6'b100100;
    localparam F_OR     = 6'b100101;
    localparam F_SLT    = 6'b101010;
    localparam F_SLL    = 6'b000000;
    localparam F_SRA    = 6'b000011;
    localparam F_JR     = 6'b001000;
    localparam F_MULT   = 6'b011000;
    localparam F_DIV    = 6'b011010;
    localparam F_MFHI   = 6'b010000;
    localparam F_MFLO   = 6'b010010;
    localparam F_XCHG   = 6'b100011; // Instrução adicional

    // Lógica de transição de estado
    always @(posedge clk or posedge reset) begin
        if (reset)
            state <= S_RESET;
        else
            state <= next_state;
    end

    // Máquina de Estados (FSM) - Lógica Combinacional
    always @(*) begin
        // Valores padrão para os sinais de controle (inativos)
        PCWrite = 0;
        PCWriteCond = 0;
        PCWriteCondNeg = 0;
        IorD = 0;
        MemRead = 0;
        MemWrite = 0;
        IRWrite = 0;
        RegWrite = 0;
        RegDst = 2'b00;    // Default: rt
        ALUSrcA = 1'b0;    // Default: PC
        ALUSrcB = 2'b00;   // Default: RegB
        PCSource = 2'b00;  // Default: ALU output (PC+4 or branch address)
        ALUOp = 4'b0000;   // Default: NOP
        HIWrite = 0;
        LOWrite = 0;
        MultStart = 0;
        DivStart = 0;
        WBDataSrc = 3'b000; // Default: ALU output
        MemDataInSrc = 0;
        PCClear = 0;
        RegsClear = 0;
        TempRegWrite = 0;
        MemtoRegA = 0;
        next_state = S_FETCH; // Valor padrão para transição

        case (state)
            S_RESET: begin
                // Apenas transição para S_FETCH
                next_state = S_FETCH;
            end

            S_FETCH: begin
                // Busca de instrução: IR = Mem[PC], PC = PC + 4
                MemRead = 1;
                IRWrite = 1;
                ALUSrcA = 1'b0;    // PC
                ALUSrcB = 2'b01;   // Constante 4
                ALUOp = 4'b0001;   // ADD para PC = PC + 4
                PCWrite = 1;
                PCSource = 2'b00;  // PC <= Saída da ALU
                next_state = S_DECODE;
            end

            S_DECODE: begin
                // Decodificação e busca de operandos
                // Configuração baseada no opcode para o próximo estado (Execute)
                ALUSrcA = 1'b1;  // Prepara para usar RegA
                ALUSrcB = 2'b10; // Prepara para usar imediato estendido
                next_state = S_EXECUTE;

                // Transições diretas para J, JAL, JR
                case (opcode)
                    OP_J: begin
                        PCWrite = 1;
                        PCSource = 2'b10; // jump_address
                        next_state = S_FETCH;
                    end
                    OP_JAL: begin
                        RegWrite = 1;
                        WBDataSrc = 3'b000; // PC+4 (já calculado no fetch e na ALU)
                        RegDst = 2'b11;    // Destino é $ra (não implementado no RegDst, mas sinaliza a intenção)
                        PCWrite = 1;
                        PCSource = 2'b10; // jump_address
                        next_state = S_FETCH;
                    end
                    OP_RTYPE: begin
                        if (funct == F_JR) begin
                           PCWrite = 1;
                           PCSource = 2'b01; // reg_a_out (rs)
                           next_state = S_FETCH;
                        end else begin
                           next_state = S_EXECUTE;
                        end
                    end
                    default: begin
                        next_state = S_EXECUTE;
                    end
                endcase
            end

            S_EXECUTE: begin
                case (opcode)
                    OP_RTYPE: begin
                        ALUSrcA = 1'b1;
                        ALUSrcB = 2'b00; // ALU usa RegA e RegB
                        // Define ALUOp com base no funct
                        case (funct)
                            F_ADD: ALUOp = 4'b0001;
                            F_SUB: ALUOp = 4'b0010;
                            F_AND: ALUOp = 4'b0011;
                            F_OR:  ALUOp = 4'b0100; // ula32 não tem OR, mas mantemos
                            F_SLT: ALUOp = 4'b0010; // Usa SUB e flags
                            // SLL/SRA precisariam de um shifter
                            F_MULT: MultStart = 1; // Inicia multiplicação
                            F_DIV:  DivStart = 1;  // Inicia divisão
                            F_MFHI: begin RegWrite = 1; RegDst = 2'b10; WBDataSrc = 3'b010; next_state = S_FETCH; end
                            F_MFLO: begin RegWrite = 1; RegDst = 2'b10; WBDataSrc = 3'b011; next_state = S_FETCH; end
                            default: next_state = S_FETCH; // Funct inválido
                        endcase
                        next_state = S_WRITEBACK;
                    end
                    OP_LW, OP_SW: begin
                        // Calcula endereço: RegA + sign_ext(imm)
                        ALUSrcA = 1'b1;
                        ALUSrcB = 2'b10;
                        ALUOp = 4'b0001; // ADD
                        next_state = S_MEMORY;
                    end
                    OP_BEQ, OP_BNE: begin
                        // Compara registradores: RegA - RegB
                        ALUSrcA = 1'b1;
                        ALUSrcB = 2'b00;
                        ALUOp = 4'b0010; // SUB (para setar flag 'zero')
                        PCWriteCond = 1;
                        PCSource = 2'b00; // PC <= alu_out (branch_address)
                        next_state = S_FETCH;
                    end
                    OP_ADDI: begin
                        // Calcula: RegA + sign_ext(imm)
                        ALUSrcA = 1'b1;
                        ALUSrcB = 2'b10;
                        ALUOp = 4'b0001; // ADD
                        next_state = S_WRITEBACK;
                    end
                    OP_LUI: begin
                        // Carrega imediato nos bits superiores: imm << 16
                        ALUSrcA = 1'b0; // Não usa RegA
                        ALUSrcB = 2'b11; // Usa imediato deslocado (lógica no MUX)
                        ALUOp = 4'b0000; // Passa a entrada B para a saída
                        next_state = S_WRITEBACK;
                    end
                    default: next_state = S_FETCH; // Opcode inválido
                endcase
            end

            S_MEMORY: begin
                case (opcode)
                    OP_LW: begin
                        // Lê da memória: MDR = Mem[ALU_out]
                        MemRead = 1;
                        next_state = S_WRITEBACK;
                    end
                    OP_SW: begin
                        // Escreve na memória: Mem[ALU_out] = RegB
                        MemWrite = 1;
                        next_state = S_FETCH; // SW não tem writeback
                    end
                    /*
                    // Opcodes LB/SB não foram definidos
                    OP_LB, OP_SB: begin
                        MemRead = (opcode == OP_LB);
                        MemWrite = (opcode == OP_SB);
                        MemDataInSrc = 1;
                        next_state = (opcode == OP_LB) ? S_WRITEBACK : S_FETCH;
                    end
                    */
                    default: begin
                        next_state = S_FETCH; // Não deveria chegar aqui
                    end
                endcase
            end

            S_WRITEBACK: begin
                next_state = S_FETCH;
                RegWrite = 1; // Habilita escrita no banco de registradores

                case (opcode)
                    OP_RTYPE: begin
                        RegDst = 2'b10; // Destino é rd
                        WBDataSrc = 3'b000; // Fonte é ALU_out
                    end
                    OP_LW: begin
                        RegDst = 2'b01; // Destino é rt
                        WBDataSrc = 3'b001; // Fonte é MDR (saída da memória)
                    end
                    OP_ADDI: begin
                        RegDst = 2'b01; // Destino é rt
                        WBDataSrc = 3'b000; // Fonte é ALU_out
                    end
                    OP_LUI: begin
                        RegDst = 2'b01; // Destino é rt
                        WBDataSrc = 3'b000; // Fonte é ALU_out
                    end
                    default: begin
                        RegWrite = 0; // Nenhuma escrita para outros casos
                    end
                endcase
            end

            default: next_state = S_FETCH;
        endcase
    end

endmodule