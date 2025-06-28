// divider.v
// Módulo que implementa divisão 32x32 com sinal.
// Leva 32 ciclos.
module divider (
    input wire signed [31:0] a,         // Dividendo
    input wire signed [31:0] b,         // Divisor
    input wire               start,
    input wire               clk,
    input wire               reset,
    output reg signed [31:0] quotient,    // Quociente
    output reg signed [31:0] remainder,   // Resto
    output reg               done,
    output reg               div_by_zero
);

    reg [31:0] abs_a, abs_b;
    reg [63:0] dividend_reg; // Remainder (upper 32) and quotient (lower 32)
    reg [5:0]  count;
    reg        sign_quotient;
    reg        sign_remainder;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            quotient <= 32'd0;
            remainder <= 32'd0;
            done <= 1'b0;
            div_by_zero <= 1'b0;
            count <= 6'd0;
        end else if (start) begin
            done <= 1'b0;
            div_by_zero <= 1'b0;
            if (b == 32'd0) begin
                div_by_zero <= 1'b1;
                done <= 1'b1;
            end else begin
                // Armazena sinais e usa valores absolutos
                abs_a = (a[31]) ? -a : a;
                abs_b = (b[31]) ? -b : b;
                sign_quotient = a[31] ^ b[31];
                sign_remainder = a[31];

                dividend_reg <= {33'd0, abs_a}; // Começa com o dividendo
                count <= 6'd0;
            end
        end else if (!done && !div_by_zero) begin
            if (count < 32) begin
                // Algoritmo de divisão restauradora
                dividend_reg <= dividend_reg << 1; // Shift left
                
                if (dividend_reg[63:32] >= abs_b) begin
                    dividend_reg[63:32] <= dividend_reg[63:32] - abs_b;
                    dividend_reg[0] <= 1'b1; // Seta bit do quociente
                end
                
                count <= count + 1;
            end else begin
                // Ajusta sinais no final
                quotient <= (sign_quotient) ? -dividend_reg[31:0] : dividend_reg[31:0];
                remainder <= (sign_remainder) ? -dividend_reg[63:32] : dividend_reg[63:32];
                done <= 1'b1;
            end
        end
    end
endmodule