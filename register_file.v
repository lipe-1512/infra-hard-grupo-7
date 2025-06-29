module register_file (
    input wire clk,
    input wire reset,
    input wire [4:0] rs,
    input wire [4:0] rt,
    input wire [4:0] rd,
    input wire [31:0] data_in,
    input wire RegWrite,
    input wire RegDst,        // 0 para rt, 1 para rd
    output [31:0] reg_a_out,  // CORRIGIDO: removido 'reg'
    output [31:0] reg_b_out   // CORRIGIDO: removido 'reg'
);

    // Banco de 32 registradores de 32 bits cada.
    // 'reg' aqui significa que 'registers' é uma variável que armazena estado.
    reg [31:0] registers [0:31];
    
    // Wire para o endereço do registrador de destino
    wire [4:0] write_addr;

    integer i;

    // Lógica de Escrita (Síncrona)
    // Atualiza o registrador de destino na borda de subida do clock se RegWrite estiver ativo.
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            // No reset, zera todos os registradores.
            for (i = 0; i < 32; i = i + 1) begin
                registers[i] <= 32'b0;
            end
        end else if (RegWrite && (write_addr != 5'd0)) begin
            // Escreve o dado de entrada no registrador de destino.
            // Impede a escrita no registrador $zero.
            registers[write_addr] <= data_in;
        end
    end

    // MUX para selecionar o registrador de destino (Combinacional)
    // Se RegDst for 0 (I-type), o destino é 'rt'.
    // Se RegDst for 1 (R-type), o destino é 'rd'.
    assign write_addr = (RegDst == 1'b0) ? rt : rd;

    // Lógica de Leitura (Combinacional)
    // As saídas refletem o conteúdo dos registradores 'rs' e 'rt' instantaneamente.
    // A leitura do registrador $zero sempre retorna 0.
    assign reg_a_out = (rs == 5'd0) ? 32'd0 : registers[rs];
    assign reg_b_out = (rt == 5'd0) ? 32'd0 : registers[rt];

endmodule