module zero_detector (
    input [31:0] in,  // Entrada
    output zero       // Saída
);

    assign zero = (in == 32'b0); // Verifica se a entrada é zero

endmodule