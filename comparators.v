module equal_comparator (
    input [31:0] a, b, // Entradas
    output equal        // Saída
);

    assign equal = (a == b); // Verifica se a == b

endmodule

module no_equal_comparator (
    input [31:0] a, b, // Entradas
    output no_equal     // Saída
);

    assign no_equal = (a != b); // Verifica se a != b

endmodule