module mux_RegDst (
    input [4:0] in1, in2,  // Entrada 2 (5 bits)
    input sel,        // Sinal de seleção (1 bit)
    output reg [4:0] out // Saída (5 bits)
);
    always @(*) begin
        case (sel)
            1'b0: out = in1; // Seleciona in1
            1'b1: out = in2; // Seleciona in2
            default: out = 5'b0; // Default (opcional)
        endcase
    end
endmodule