module shift_left_2 (
    input [31:0] in,
    output [31:0] out
);
    assign out = in << 2; // Desloca 2 bits para a esquerda
endmodule