module register_file (
    input wire clk,
    input wire reset,
    input wire [4:0] rs,
    input wire [4:0] rt,
    input wire [4:0] rd,
    input wire [31:0] data_in,
    input wire RegWrite,
    input wire RegDst,
    output reg [31:0] reg_a_out,
    output reg [31:0] reg_b_out
);

    // Banco de 32 registradores de 32 bits cada
    reg [31:0] registers [0:31];

    integer i;

    // Inicialização dos registradores no reset
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 32; i = i + 1) begin
                registers[i] <= 32'b0;
            end
        end else if (RegWrite) begin
            if (RegDst == 1'b0) begin
                registers[rd] <= data_in;
            end else begin
                registers[rt] <= data_in;
            end
        end
    end

    // Leitura combinacional dos registradores
    assign reg_a_out = registers[rs];
    assign reg_b_out = registers[rt];

endmodule