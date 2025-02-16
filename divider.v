module divider (
    input [31:0] a, b,        // Operandos
    input start,              // Sinal de início
    input clk,               // Sinal de clock (adicionado)
    output reg [31:0] quotient, // Quociente
    output reg [31:0] remainder, // Resto
    output reg done,          // Sinal de conclusão
    output reg div_by_zero    // Sinal de divisão por zero
);

    reg [31:0] dividend;
    reg [31:0] divisor;
    reg [5:0] count;

    always @(posedge start) begin
        if (start) begin
            if (b == 32'b0) begin
                div_by_zero = 1;
                done = 1;
            end else begin
                dividend = a;
                divisor = b;
                quotient = 32'b0;
                remainder = 32'b0;
                count = 0;
                done = 0;
                div_by_zero = 0;
            end
        end
    end

    always @(posedge clk) begin
        if (count < 32 && !div_by_zero) begin
            remainder = remainder << 1;
            remainder[0] = dividend[31];
            dividend = dividend << 1;
            if (remainder >= divisor) begin
                remainder = remainder - divisor;
                quotient[count] = 1;
            end
            count = count + 1;
        end else if (count == 32) begin
            done = 1;
        end
    end

endmodule