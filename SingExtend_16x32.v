module SingExtend_16x32 (
    input [15:0] in1,
    output [31:0] out
);
    assign out = {{16{in1[15]}}, in1}; // Extensão de sinal para 32 bits
endmodule