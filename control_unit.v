// control_unit.v
// CORRIGIDO: FSM reestruturada para um fluxo multi-ciclo verdadeiro,
// garantindo que cada etapa (Fetch, Decode, Execute, etc.) ocorra em um ciclo de clock separado.
// Isto resolve o bug de lógica visto nas formas de onda.

module control_unit (
    input wire clk,
    input wire reset,
    input wire [5:0] opcode,
    input wire [5:0] funct,

    // Entradas de unidades multi-ciclo (não utilizadas nesta versão simplificada)
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
    output reg HIWrite,
    output reg LOWrite,
    output reg MultStart,
    output reg DivStart,
    output reg [2:0] WBDataSrc,
    // Sinais mantidos para compatibilidade de porta, mas não dirigidos nesta lógica simplificada
    output reg PCWriteCondNeg, 
    output reg MemAddrSrc,
    output reg MemDataInSrc,
    output reg PCClear,
    output reg RegsClear,
    output reg TempRegWrite,
    output reg MemtoRegA
);

    // Estados da FSM
    parameter S_RESET = 0, S_FETCH = 1, S_DECODE = 2, S_EXECUTE_R = 3, S_EXECUTE_I = 4, S_MEMORY_ADDR = 5, S_MEMORY_READ = 6, S_MEMORY_WRITE = 7, S_WRITEBACK = 8;
    reg [3:0] state, next_state;

    // Constantes de Opcode e Funct
    localparam OP_RTYPE = 6'b000000;
    localparam OP_ADDI  = 6'b001000;
    localparam OP_LW    = 6'b100011;
    localparam OP_SW    = 6'b101011;
    localparam OP_BEQ   = 6'b000100;
    localparam OP_J     = 6'b000010;
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
        // Valores padrão (inativos) para todos os sinais de controle
        PCWrite=0; PCWriteCond=0; IorD=0; MemRead=0; MemWrite=0; IRWrite=0; RegWrite=0;
        RegDst=2'b00; ALUSrcA=0; ALUSrcB=2'b00; PCSource=2'b00; ALUOp=4'b0000;
        HIWrite=0; LOWrite=0; MultStart=0; DivStart=0; WBDataSrc=3'b000;
        PCWriteCondNeg=0; MemAddrSrc=0; MemDataInSrc=0; PCClear=0; RegsClear=0; TempRegWrite=0; MemtoRegA=0;
        
        next_state = state; // Default: permanece no mesmo estado

        case (state)
            S_RESET: next_state = S_FETCH;
            
            // Ciclo 1: Busca da Instrução (Comum a todas as instruções)
            S_FETCH: begin
                MemRead = 1; IRWrite = 1; PCWrite = 1;
                ALUSrcA = 0; ALUSrcB = 2'b01; ALUOp = 4'b0001; // PC = PC + 4
                next_state = S_DECODE;
            end
            
            // Ciclo 2: Decodificação e Busca de Operandos
            S_DECODE: begin
                // A leitura dos registradores ocorre no datapath.
                // A FSM apenas decide para onde ir em seguida.
                case (opcode)
                    OP_LW:    next_state = S_EXECUTE_I;   // Calcula endereço
                    OP_SW:    next_state = S_EXECUTE_I;   // Calcula endereço
                    OP_RTYPE: next_state = S_EXECUTE_R;   // Executa R-type
                    OP_BEQ:   next_state = S_EXECUTE_R;   // Compara (subtrai)
                    OP_ADDI:  next_state = S_EXECUTE_I;   // Executa I-type
                    OP_J:     begin PCWrite = 1; PCSource = 2'b10; next_state = S_FETCH; end
                    default:  next_state = S_FETCH; // Opcode inválido
                endcase
            end
            
            // Ciclo 3: Execução para instruções R-Type e Branch
            S_EXECUTE_R: begin
                ALUSrcA = 1; ALUSrcB = 2'b00; // Entradas da ALU são RegA e RegB
                case (opcode)
                    OP_RTYPE: begin
                        case (funct) // Define a operação da ULA
                           F_ADD: ALUOp = 4'b0001;
                           F_SUB: ALUOp = 4'b0010;
                           F_SLT: ALUOp = 4'b0010;
                           default: ;
                        endcase
                        next_state = S_WRITEBACK; // Próximo estado é escrever o resultado
                    end
                    OP_BEQ: begin
                        ALUOp = 4'b0010; // Subtração para comparação
                        PCWriteCond = 1; // Habilita desvio condicional
                        next_state = S_FETCH;
                    end
                endcase
            end

            // Ciclo 3: Execução para instruções I-Type (cálculo de endereço/operação)
            S_EXECUTE_I: begin
                ALUSrcA = 1; ALUSrcB = 2'b10; // Entradas da ALU são RegA e Imediato
                ALUOp = 4'b0001; // Soma
                case(opcode)
                    OP_LW, OP_SW: next_state = S_MEMORY_ADDR; // Próximo estado é acessar a memória
                    OP_ADDI:      next_state = S_WRITEBACK;   // Próximo estado é escrever o resultado
                endcase
            end

            // Ciclo 4 (apenas LW/SW): Acessa a memória com o endereço calculado
            S_MEMORY_ADDR: begin
                IorD = 1; // Endereço vem da ALU
                case (opcode)
                    OP_LW: begin MemRead = 1; next_state = S_WRITEBACK; end // Lê e vai para o write-back
                    OP_SW: begin MemWrite = 1; next_state = S_FETCH; end   // Escreve e termina
                endcase
            end
            
            // Ciclo 4 ou 5: Escreve o resultado no Banco de Registradores
            S_WRITEBACK: begin
                RegWrite = 1; // Habilita a escrita
                case(opcode)
                    OP_RTYPE: begin RegDst=2'b01; WBDataSrc=3'b000; 
                                   if(funct == F_SLT) WBDataSrc = 3'b101; end // R[rd] <- ALU_out
                    OP_ADDI:  begin RegDst=2'b00; WBDataSrc=3'b000; end // R[rt] <- ALU_out
                    OP_LW:    begin RegDst=2'b00; WBDataSrc=3'b001; end // R[rt] <- MDR
                endcase
                next_state = S_FETCH;
            end
            default: next_state = S_FETCH;
        endcase
    end
endmodule