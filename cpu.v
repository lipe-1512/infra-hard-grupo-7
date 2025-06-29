module cpu (
    input wire clk,
    input wire reset,

    output wire [31:0] pc_out,
    output wire [31:0] alu_result,
    output wire [31:0] hi_out,
    output wire [31:0] lo_out
);

    // Sinais de controle da CPU
    wire PCWrite;
    wire PCWriteCond;
    wire IorD;
    wire MemRead;
    wire MemWrite;
    wire IRWrite;
    wire RegWrite;
    wire [1:0] RegDst;
    wire ALUSrcA;
    wire [1:0] ALUSrcB;
    wire [1:0] PCSource;
    wire [3:0] ALUOp;
    wire HIWrite;
    wire LOWrite;
    wire MultStart;
    wire DivStart;
    wire [2:0] WBDataSrc;
    wire [1:0] MemAddrSrc;
    wire [1:0] MemDataSrc;

    // Sinais de dados
    wire [31:0] instruction_memory_data;
    wire [31:0] data_memory_read;
    wire [31:0] reg_a_out;
    wire [31:0] reg_b_out;
    wire [31:0] alu_in_a;
    wire [31:0] alu_in_b;
    wire [31:0] alu_out;
    wire [31:0] pc;
    wire [31:0] next_pc;
    wire [5:0] opcode;
    wire [5:0] funct;
    wire [4:0] rs, rt, rd;
    wire [15:0] immediate;
    wire mult_done;
    wire div_done;
    wire [31:0] mult_result;
    wire [31:0] div_result;
    wire zero;
    wire overflow;
    wire branch_taken;
    wire jump_taken;
    wire jal_taken;

    // Instância do controlador
    control_unit u_control (
        .clk(clk),
        .reset(reset),
        .opcode(opcode),
        .funct(funct),
        .mult_done_in(mult_done),
        .div_done_in(div_done),
        .PCWrite(PCWrite),
        .PCWriteCond(PCWriteCond),
        .IorD(IorD),
        .MemRead(MemRead),
        .MemWrite(MemWrite),
        .IRWrite(IRWrite),
        .RegWrite(RegWrite),
        .RegDst(RegDst),
        .ALUSrcA(ALUSrcA),
        .ALUSrcB(ALUSrcB),
        .PCSource(PCSource),
        .ALUOp(ALUOp),
        .HIWrite(HIWrite),
        .LOWrite(LOWrite),
        .MultStart(MultStart),
        .DivStart(DivStart),
        .WBDataSrc(WBDataSrc),
        .MemAddrSrc(MemAddrSrc),
        .MemDataSrc(MemDataSrc)
    );

    // Banco de Registradores - Wrap para Banco_reg.vhd
    register_file u_register_file (
        .clk(clk),
        .reset(reset),
        .rs(rs),
        .rt(rt),
        .rd(rd),
        .data_in(alu_out),
        .RegWrite(RegWrite),
        .RegDst(RegDst),
        .reg_a_out(reg_a_out),
        .reg_b_out(reg_b_out)
    );

    // Unidade Lógica e Aritmética - Wrap para ula32.vhd
    alu u_alu (
        .a(reg_a_out),
        .b(reg_b_out),
        .alu_op(ALUOp),
        .result(alu_out),
        .zero(zero),
        .overflow(overflow)
    );

    // Memória - Wrap para Memoria.vhd
    memory u_memory (
        .address((IorD) ? alu_out : pc),
        .data_in(reg_b_out),
        .write_enable(MemWrite),
        .read_enable(MemRead),
        .clock(clk),
        .data_out(data_memory_read)
    );

    // Multiplicação Multi-ciclo
    multiplier u_multiplier (
        .a(reg_a_out),
        .b(reg_b_out),
        .start(MultStart),
        .done(mult_done),
        .result(mult_result)
    );

    // Divisão Multi-ciclo
    divider u_divider (
        .a(reg_a_out),
        .b(reg_b_out),
        .start(DivStart),
        .done(div_done),
        .result(div_result)
    );

    // Registradores Hi/Lo
    hi_lo_registers u_hi_lo (
        .clk(clk),
        .reset(reset),
        .hi_in(mult_result[63:32]),
        .lo_in(mult_result[31:0]),
        .hi_write(HIWrite),
        .lo_write(LOWrite),
        .hi_out(hi_out),
        .lo_out(lo_out)
    );

    // Program Counter
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pc <= 32'h00000000;
        end else if (PCWrite) begin
            pc <= next_pc;
        end else if (PCWriteCond && branch_taken) begin
            pc <= branch_address;
        end else if (jump_taken || jal_taken) begin
            pc <= jump_address;
        end else begin
            pc <= pc + 4;
        end
    end

    assign pc_out = pc;

    // Cálculo do endereço de salto
    wire [31:0] jump_address;
    assign jump_address = {pc[31:28], instruction_memory_data[25:0], 2'b00};

    // Endereço de desvio condicional
    wire [31:0] branch_address;
    wire [15:0] sign_extend;
    assign sign_extend = {{16{instruction_memory_data[15]}}, instruction_memory_data};
    assign branch_address = pc + (sign_extend << 2);

    // Detecta se há um desvio condicional
    wire branch_taken;
    assign branch_taken = (opcode == OP_BEQ || opcode == OP_BNE) && (zero ^ (opcode == OP_BNE));

    // Detecta se há salto incondicional
    wire jump_taken;
    assign jump_taken = (opcode == OP_J);

    // Detecta se há chamada a subrotina (jal)
    wire jal_taken;
    assign jal_taken = (opcode == OP_JAL);

    // Registrador de Instrução
    wire [31:0] ir;
    assign ir = instruction_memory_data;

    // Decodificação dos campos da instrução
    assign opcode = ir[31:26];
    assign funct = ir[5:0];
    assign rs = ir[25:21];
    assign rt = ir[20:16];
    assign rd = ir[15:11];
    assign immediate = ir[15:0];

    // Endereço do PC para próxima instrução
    assign next_pc = (PCSource == 2'b10) ? jump_address : (branch_taken ? branch_address : pc + 4);

    // Output do registrador b para memória
    wire [31:0] wb_data;
    wire [31:0] mdr_out;
    assign wb_data = (WBDataSrc == 3'b000) ? alu_out :
                     (WBDataSrc == 3'b001) ? mdr_out :
                     (WBDataSrc == 3'b010) ? hi_out :
                     (WBDataSrc == 3'b011) ? lo_out :
                     32'b0;

    // Mapeia saída da memória como MDR
    assign mdr_out = data_memory_read;

endmodule