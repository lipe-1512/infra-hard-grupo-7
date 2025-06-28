// mux_ALUsrc.v
// Seleciona a segunda entrada da ALU (Sinal B).
module mux_ALUsrc (
    input [31:0] reg_b_data,      // 00: Vem do Registrador B
    input [31:0] constant_4,      // 01: Constante 4
    input [31:0] sign_ext_imm,    // 10: Imediato com extensão de sinal
    input [31:0] shifted_imm,     // 11: Imediato com ext. de sinal e shift left 2
    input [1:0]  sel,             // Sinal de controle ALUSrcB
    output reg [31:0] out
);
    always @(*) begin
        case (sel)
            2'b00:   out = reg_b_data;
            2'b01:   out = constant_4;
            2'b10:   out = sign_ext_imm;
            2'b11:   out = shifted_imm;
            default: out = 32'hxxxxxxxx; // Indefinido
        endcase
    end
endmodule