// Módulo: shifter_general
// Descrição: Implementa deslocamentos lógicos (sll, srl) e aritmético (sra)
//            baseado em uma entrada de controle. Traduzido de RegDesloc.vhd.
module shifter_general (
    input  wire [31:0] in,       // Dado de entrada (de Reg B)
    input  wire [4:0]  shamt,    // Quantidade de deslocamento (da instrução)
    input  wire [2:0]  sh_op,    // Operação de shift (do controle da ULA)
    output reg  [31:0] out       // Resultado do deslocamento
);

    // Operações baseadas no MIPS
    localparam OP_SLL = 3'b010; // Shift Left Logical
    localparam OP_SRL = 3'b011; // Shift Right Logical
    localparam OP_SRA = 3'b100; // Shift Right Arithmetic

    always @(*) begin
        case (sh_op)
            OP_SLL: out = in << shamt;
            OP_SRL: out = in >> shamt;
            OP_SRA: out = $signed(in) >>> shamt; // Deslocamento aritmético com sinal
            default: out = 32'b0;
        endcase
    end
endmodule