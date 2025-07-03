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
    output reg [2:0] WBDataSrc,

    // Portas não utilizadas mantidas para compatibilidade, mas sem lógica dirigida.
    output reg PCWriteCondNeg, 
    output reg MemAddrSrc,
    output reg MemDataInSrc,
    output reg PCClear,
    output reg RegsClear,
    output reg TempRegWrite,
    output reg MemtoRegA,
    output reg HIWrite,
    output reg LOWrite,
    output reg MultStart,
    output reg DivStart
);

    // Estados da FSM (usando os 6 estados originais)
    parameter S_RESET = 0, S_FETCH = 1, S_DECODE = 2, S_EXECUTE = 3, S_MEMORY = 4, S_WRITEBACK = 5;
    reg [2:0] state, next_state;

    // Constantes de Opcode e Funct
    localparam OP_RTYPE = 6'b000000;
    localparam OP_ADDI  = 6'b001000;
    localparam OP_LW    = 6'b100011;
    localparam OP_SW    = 6'b101011;
    localparam OP_BEQ   = 6'b000100;
    localparam OP_J     = 6'b000010;
    localparam OP_JAL   = 6'b000011;
    localparam F_ADD    = 6'b100000;
    localparam F_SUB    = 6'b100010;
    localparam F_SLT    = 6'b101010;

    // Lógica de transição de estado síncrona
    always @(posedge clk or posedge reset) begin
        if (reset) state <= S_RESET;
        else state <= next_state;
    end

    // Lógica de saída e próximo estado (combinacional)
    always @(*) begin
        // Valores Padrão (Inativos)
        PCWrite=0; PCWriteCond=0; IorD=0; MemRead=0; MemWrite=0; IRWrite=0; RegWrite=0;
        RegDst=2'b00; ALUSrcA=0; ALUSrcB=2'b00; PCSource=2'b00; ALUOp=4'b0000; WBDataSrc=3'b000;
        // Zera as portas não utilizadas para evitar valores indefinidos
        PCWriteCondNeg=0; MemAddrSrc=0; MemDataInSrc=0; PCClear=0; RegsClear=0; TempRegWrite=0; MemtoRegA=0;
        HIWrite=0; LOWrite=0; MultStart=0; DivStart=0;
        
        next_state = state; // Default: permanece no estado atual

        case (state)
            S_RESET: next_state = S_FETCH;
            
            // Ciclo 1: Busca da Instrução
            S_FETCH: begin
                MemRead = 1; IRWrite = 1; PCWrite = 1;
                ALUSrcA = 0; ALUSrcB = 2'b01; ALUOp = 4'b0001; // PC = PC + 4
                next_state = S_DECODE;
            end
            
            // Ciclo 2: Decodificação e Leitura dos Registradores
            S_DECODE: begin
                // A leitura dos registradores ocorre no datapath. A FSM apenas decide o próximo passo.
                case (opcode)
                    OP_LW | OP_SW | OP_ADDI | OP_RTYPE | OP_BEQ: next_state = S_EXECUTE;
                    OP_J:     begin PCWrite = 1; PCSource = 2'b10; next_state = S_FETCH; end
                    OP_JAL:   begin RegWrite=1; RegDst=2'b11; WBDataSrc=3'b111; PCWrite=1; PCSource=2'b10; next_state=S_FETCH; end
                    default:  next_state = S_FETCH; // Opcode inválido
                endcase
            end
            
            // Ciclo 3: Execução
            S_EXECUTE: begin
                case(opcode)
                    OP_LW, OP_SW: begin ALUSrcA=1; ALUSrcB=2'b10; ALUOp=4'b0001; next_state = S_MEMORY; end // Addr = R[rs] + imm
                    OP_RTYPE:     begin ALUSrcA=1; ALUSrcB=2'b00; // Op = R[rs] op R[rt]
                        case (funct)
                           F_ADD: ALUOp = 4'b0001;
                           F_SUB: ALUOp = 4'b0010;
                           F_SLT: ALUOp = 4'b0010;
                           default: ALUOp = 4'b0000;
                        endcase
                        next_state = S_WRITEBACK;
                    end
                    OP_BEQ:  begin ALUSrcA=1; ALUSrcB=2'b00; ALUOp=4'b0010; PCWriteCond=1; next_state=S_FETCH; end // Subtrai para comparar
                    OP_ADDI: begin ALUSrcA=1; ALUSrcB=2'b10; ALUOp=4'b0001; next_state = S_WRITEBACK; end // Soma imediato
                    default: next_state = S_FETCH;
                endcase
            end

            // Ciclo 4: Acesso à Memória
            S_MEMORY: begin
                IorD = 1; // Endereço vem da ALU
                case(opcode)
                    OP_LW: begin MemRead=1; next_state = S_WRITEBACK; end
                    OP_SW: begin MemWrite=1; next_state = S_FETCH; end
                    default: next_state = S_FETCH; // Não deveria acontecer
                endcase
            end

            // Ciclo 4 ou 5: Escrita de Volta (Write-Back)
            S_WRITEBACK: begin
                RegWrite = 1; // Habilita a escrita no banco de registradores
                case(opcode)
                    OP_LW:    begin RegDst=2'b00; WBDataSrc=3'b001; end // LW: R[rt] <- MDR
                    OP_RTYPE: begin RegDst=2'b01; WBDataSrc=3'b000; if (funct == F_SLT) WBDataSrc = 3'b101; end // R-type: R[rd] <- ALU_out
                    OP_ADDI:  begin RegDst=2'b00; WBDataSrc=3'b000; end // ADDI: R[rt] <- ALU_out
                    default: RegWrite = 0; // Nenhuma escrita
                endcase
                next_state = S_FETCH;
            end
            default: next_state = S_FETCH;
        endcase
    end
endmodule