module SingExtend_1x32 (
    input in1,
    output [31:0] out
);
    assign out = {{31{in1}}, in1}; // Extensão de sinal para 32 bits
endmodule