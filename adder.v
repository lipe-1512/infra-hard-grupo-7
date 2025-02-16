module adder (
    input [31:0] a, b, // Entradas
    output [31:0] out  // Saída
);

    assign out = a + b; // Soma as entradas

endmodule