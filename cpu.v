// cpu.v
// Módulo Top-Level FINAL e POLIDO.
module cpu(
    input wire clk,
    input wire reset
);
    wire        PCWrite, PCWriteCond, PCWriteCondNeg;
    wire        IorD, MemRead, MemWrite, IRWrite, RegWrite;
    wire [1:0]  RegDst;
    wire        ALUSrcA;
    wire [1:0]  ALUSrcB;
    wire [1:0]  PCSource;
    wire [3:0]  ALUOp;
    wire        HIWrite, LOWrite, MultStart, DivStart;
    wire [1:0]  WBDataSrc;

    wire [31:0] pc_out, next_pc, mem_data_out, alu_out_reg, mdr_out, reg_a_out, reg_b_out;
    wire [31:0] read_data_1, read_data_2, sign_extended_imm, alu_result, write_data_mux_out;
    wire [4:0]  write_reg_mux_out;
    wire        alu_zero, ula_overflow, invalid_opcode_flag, div_by_zero_flag;
    wire [31:0] epc_out, exception_pc;
    wire        exception_detected;

    wire [5:0]  ir_opcode;
    wire [4:0]  ir_rs, ir_rt, ir_rd, ir_shamt;
    wire [15:0] ir_immediate;
    wire [5:0]  ir_funct;
    wire [25:0] ir_jump_addr;

    wire [63:0] mult_result;
    wire        mult_done;
    wire [31:0] div_quotient, div_remainder;
    wire        div_done;
    wire [31:0] hi_out, lo_out;

    assign ir_jump_addr = {ir_rs, ir_rt, ir_rd, ir_shamt, ir_funct};
    assign next_pc = (PCSource == 2'b00) ? alu_result : (PCSource == 2'b01) ? alu_out_reg :
                     (PCSource == 2'b10) ? {pc_out[31:28], ir_jump_addr, 2'b00} : reg_a_out;
    wire cond_taken = (PCWriteCond && alu_zero) || (PCWriteCondNeg && ~alu_zero);
    wire pc_write_enable = exception_detected ? 1'b1 : (PCWriteCond || PCWriteCondNeg) ? cond_taken : PCWrite;
    wire [31:0] pc_in = exception_detected ? exception_pc : next_pc;

    Registrador #(32) pc_reg (.clk(clk), .reset(reset), .Load(pc_write_enable), .Entrada(pc_in), .Saida(pc_out));
    wire [31:0] mem_address = IorD ? alu_out_reg : pc_out;
    Memoria memoria (.Address(mem_address), .Clock(clk), .Wr(MemWrite), .Datain(reg_b_out), .Dataout(mem_data_out));
    Instr_Reg ir_unit (.Clk(clk), .Reset(reset), .Load_ir(IRWrite), .Entrada(mem_data_out), .Instr31_26(ir_opcode), .Instr25_21(ir_rs), .Instr20_16(ir_rt), .Instr15_0(ir_immediate));
    assign ir_rd = ir_immediate[15:11];
    assign ir_shamt = ir_immediate[10:6];
    assign ir_funct = ir_immediate[5:0];

    Registrador #(32) mdr_reg (.clk(clk), .reset(reset), .Load(1'b1), .Entrada(mem_data_out), .Saida(mdr_out));
    Registrador #(32) reg_a (.clk(clk), .reset(reset), .Load(1'b1), .Entrada(read_data_1), .Saida(reg_a_out));
    Registrador #(32) reg_b (.clk(clk), .reset(reset), .Load(1'b1), .Entrada(read_data_2), .Saida(reg_b_out));
    Registrador #(32) alu_out_reg_inst (.clk(clk), .reset(reset), .Load(1'b1), .Entrada(alu_result), .Saida(alu_out_reg));

    mux_RegDst mux_write_reg (.in1(ir_rt), .in2(ir_rd), .sel(RegDst[0]), .out(write_reg_mux_out));
    assign write_data_mux_out = (WBDataSrc == 2'b00) ? alu_out_reg : (WBDataSrc == 2'b01) ? mdr_out :
                                (WBDataSrc == 2'b10) ? hi_out : lo_out;
    Banco_reg banco_registradores (.Clk(clk), .Reset(reset), .RegWrite(RegWrite & ~exception_detected), .ReadReg1(ir_rs), .ReadReg2(ir_rt), .WriteReg((RegDst == 2'b10) ? 5'd31 : write_reg_mux_out), .WriteData(write_data_mux_out), .ReadData1(read_data_1), .ReadData2(read_data_2));
    
    SingExtend_16x32 sign_extender (.in1(ir_immediate), .out(sign_extended_imm));
    wire [31:0] alu_in_a = ALUSrcA ? reg_a_out : pc_out;
    wire [31:0] alu_in_b;
    mux_ALUsrc alu_src_b_mux (.reg_b_data(reg_b_out), .constant_4(32'd4), .sign_ext_imm(sign_extended_imm), .shifted_imm(sign_extended_imm << 2), .sel(ALUSrcB), .out(alu_in_b));
    Ula32 ula (.A(alu_in_a), .B(alu_in_b), .Seletor(ALUOp[2:0]), .S(alu_result), .z(alu_zero), .Overflow(ula_overflow));
    
    multiplier mult_unit (.a(reg_a_out), .b(reg_b_out), .start(MultStart), .clk(clk), .result(mult_result), .done(mult_done));
    divider div_unit (.a(reg_a_out), .b(reg_b_out), .start(DivStart), .clk(clk), .quotient(div_quotient), .remainder(div_remainder), .done(div_done), .div_by_zero(div_by_zero_flag));
    
    wire [31:0] hi_in_data = (funct == 6'b011000) ? mult_result[63:32] : div_remainder; // F_MULT
    wire [31:0] lo_in_data = (funct == 6'b011000) ? mult_result[31:0]   : div_quotient;
    hi_lo_registers hi_lo_regs (.clk(clk), .reset(reset), .hi_in(hi_in_data), .lo_in(lo_in_data), .hi_write(HIWrite), .lo_write(LOWrite), .hi_out(hi_out), .lo_out(lo_out));
    
    Registrador #(32) epc_reg (.clk(clk), .reset(reset), .Load(exception_detected), .Entrada(pc_out), .Saida(epc_out));
    exception exception_unit (.overflow(ula_overflow), .div_by_zero(div_by_zero_flag), .invalid_opcode(invalid_opcode_flag), .pc(pc_out), .epc(), .exception(exception_detected), .new_pc(exception_pc));

    control_unit FSM (
        .clk(clk), .reset(reset), .opcode(ir_opcode), .funct(ir_funct), .mult_done_in(mult_done), .div_done_in(div_done),
        .PCWrite(PCWrite), .PCWriteCond(PCWriteCond), .PCWriteCondNeg(PCWriteCondNeg), .IorD(IorD), .MemRead(MemRead),
        .MemWrite(MemWrite), .IRWrite(IRWrite), .RegWrite(RegWrite), .RegDst(RegDst), .ALUSrcA(ALUSrcA), .ALUSrcB(ALUSrcB),
        .PCSource(PCSource), .ALUOp(ALUOp), .HIWrite(HIWrite), .LOWrite(LOWrite), .MultStart(MultStart), .DivStart(DivStart),
        .InvalidOpcodeOut(invalid_opcode_flag), .WBDataSrc(WBDataSrc)
    );
endmodule