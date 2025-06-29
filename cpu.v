module cpu(
    input wire clk,
    input wire reset,
    input wire [31:0] instruction_memory_data,
    output wire [31:0] pc_out,
    output wire [31:0] alu_result,
    output wire [31:0] hi_out,
    output wire [31:0] lo_out
);

    wire [31:0] pc;
    wire [31:0] ir;
    wire [31:0] reg_a_out, reg_b_out;
    wire [31:0] alu_in_a, alu_in_b;
    wire [31:0] alu_result_from_ula;
    wire [31:0] mem_data_out;

    wire [5:0] opcode, funct;
    wire [4:0] rs, rt, rd;
    wire [15:0] immediate;

    wire mult_done, div_done;
    wire [31:0] mult_result, div_result;

    wire PCWrite, PCWriteCond, IorD, MemRead, MemWrite, IRWrite, RegWrite;
    wire [1:0] RegDst, ALUSrcA, ALUSrcB, PCSource;
    wire [3:0] ALUOp;
    wire HIWrite, LOWrite;
    wire MultStart, DivStart;
    wire [2:0] WBDataSrc;
    wire exception;
    wire [31:0] epc;

    // Control Unit
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
        .exception(exception),
        .epc(epc)
    );

    // Banco de Registradores
    register_file u_register_file (
        .clk(clk),
        .reset(reset),
        .rs(rs),
        .rt(rt),
        .rd(rd),
        .data_in(alu_result),
        .RegWrite(RegWrite),
        .RegDst(RegDst),
        .reg_a_out(reg_a_out),
        .reg_b_out(reg_b_out)
    );

    // ALU Wrapper para ula32.vhd
    alu u_alu (
        .a(reg_a_out),
        .b(reg_b_out),
        .alu_op(ALUOp),
        .result(alu_result_from_ula),
        .zero(alu_zero),
        .overflow(ula_overflow),
        .negative(ula_negativo),
        .equal(ula_igual),
        .greater(ula_maior),
        .less(ula_menor)
    );

    // Memory Wrapper para Memoria.vhd
    memory u_memory (
        .address(IorD ? alu_result_from_ula : pc),
        .data_in(reg_b_out),
        .write_enable(MemWrite),
        .read_enable(MemRead),
        .clock(clk),
        .data_out(mem_data_out)
    );

    // Multiplier e Divider Units
    multiplier u_multiplier (
        .a(reg_a_out),
        .b(reg_b_out),
        .start(MultStart),
        .clk(clk),
        .reset(reset),
        .result(mult_result),
        .done(mult_done)
    );

    divider u_divider (
        .a(reg_a_out),
        .b(reg_b_out),
        .start(DivStart),
        .clk(clk),
        .reset(reset),
        .result(div_result),
        .done(div_done)
    );

    // HI/LO Registers
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

    // Program Counter (PC)
    reg [31:0] pc_reg;

    always @(posedge clk or posedge reset) begin
        if (reset)
            pc_reg <= 32'h00000000;
        else if (PCWrite)
            pc_reg <= next_pc;
        else if (PCWriteCond && cond_taken)
            pc_reg <= branch_address;
        else
            pc_reg <= pc_reg + 4;
    end

    assign pc_out = pc_reg;

endmodule