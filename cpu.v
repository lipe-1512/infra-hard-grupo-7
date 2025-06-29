module cpu(
    input wire clk
);
    // Geração de reset para simulação
    reg reset;
    initial begin
        reset = 1'b1; #50; reset = 1'b0;
    end
    // Sinais de Controle da FSM
    wire        PCWrite, PCWriteCond, PCWriteCondNeg;
    wire        IorD, MemRead, MemWrite, IRWrite, RegWrite;
    wire [1:0]  RegDst;
    wire        ALUSrcA;
    wire [1:0]  ALUSrcB;
    wire [1:0]  PCSource;
    wire [3:0]  ALUOp;
    wire        HIWrite, LOWrite, MultStart, DivStart;
    wire [2:0]  WBDataSrc;
    wire        MemDataInSrc;
    wire        PCClear, RegsClear;
    // Sinais de controle adicionais para o datapath
    wire        TempRegWrite; 
    wire [1:0]  MemAddrSrc; // 00:PC, 01:ALUOut, 10:RegA, 11:RegB
    wire        MemDataSrc; // 0:RegB, 1:TempReg

    // Fios do Datapath
    wire signed [31:0] pc_out, next_pc, mem_data_out, alu_out_reg, mdr_out;
    wire signed [31:0] reg_a_out, reg_b_out, temp_reg_out;
    wire signed [31:0] read_data_1, read_data_2, sign_extended_imm, alu_result, alu_result_from_ula;
    wire signed [31:0] write_data_mux_out;
    wire [4:0]  write_reg_mux_out;
    wire        alu_zero, ula_overflow;
    wire signed [63:0] mult_result;
    wire signed [31:0] div_quotient, div_remainder;
    wire        mult_done, div_done, div_by_zero_flag;
    wire signed [31:0] hi_out, lo_out;
    wire        ula_negativo, ula_igual, ula_maior, ula_menor;
    wire signed [31:0] slt_result;

    // Campos do Registrador de Instrução (IR)
    wire [5:0]  ir_opcode;
    wire [4:0]  ir_rs, ir_rt, ir_rd, ir_shamt;
    wire [15:0] ir_immediate;
    wire [5:0]  ir_funct;
    wire [25:0] ir_jump_addr;

    // Lógica do PC (Program Counter)
    assign next_pc = (PCSource == 2'b00) ? alu_result : 
                     (PCSource == 2'b01) ? alu_out_reg :
                     (PCSource == 2'b10) ? {pc_out[31:28], ir_jump_addr, 2'b00} : 
                     reg_a_out;
    wire cond_taken = (PCWriteCond && alu_zero) || (PCWriteCondNeg && !alu_zero);
    wire pc_write_enable = PCWrite || cond_taken;
    registrador #(32) pc_reg (
        .clk(clk), .reset(reset), .Load(pc_write_enable), .Clear(PCClear), 
        .Entrada(next_pc), .Saida(pc_out)
    );

    // Lógica da Memória - Com multiplexadores para XCHG
    wire [31:0] mem_address;
    assign mem_address = (MemAddrSrc == 2'b00) ? pc_out :      // Fonte: PC (Fetch)
                         (MemAddrSrc == 2'b01) ? alu_out_reg : // Fonte: ALU (LW/SW/SLLM)
                         (MemAddrSrc == 2'b10) ? reg_a_out :   // Fonte: Reg A (XCHG - end. rs)
                         reg_b_out;                           // Fonte: Reg B (XCHG - end. rt)

    wire [31:0] mem_datain;
    assign mem_datain = MemDataSrc ? temp_reg_out : // Fonte: Temp Reg (para XCHG)
                        reg_b_out;                  // Fonte: Reg B (para SW e XCHG)
                        
    Memoria memoria (.Address(mem_address), .Clock(clk), .Wr(MemWrite), .Datain(mem_datain), .Dataout(mem_data_out));

    // Decodificação da Instrução
    Instr_Reg ir_unit (.Clk(clk), .Reset(reset), .Load_ir(IRWrite), .Entrada(mem_data_out), .Instr31_26(ir_opcode), .Instr25_21(ir_rs), .Instr20_16(ir_rt), .Instr15_0(ir_immediate));
    assign ir_rd = ir_immediate[15:11];
    assign ir_shamt = ir_immediate[10:6];
    assign ir_funct = ir_immediate[5:0];
    assign ir_jump_addr = {ir_rs, ir_rt, ir_immediate}; 

    // Registradores Internos do Pipeline
    registrador #(32) mdr_reg (.clk(clk), .reset(reset), .Load(1'b1), .Clear(1'b0), .Entrada(mem_data_out), .Saida(mdr_out));
    registrador #(32) reg_a (.clk(clk), .reset(reset), .Load(1'b1), .Clear(1'b0), .Entrada(read_data_1), .Saida(reg_a_out));
    registrador #(32) reg_b (.clk(clk), .reset(reset), .Load(1'b1), .Clear(1'b0), .Entrada(read_data_2), .Saida(reg_b_out));
    registrador #(32) alu_out_reg_inst (.clk(clk), .reset(reset), .Load(1'b1), .Clear(1'b0), .Entrada(alu_result), .Saida(alu_out_reg));
    registrador #(32) temp_reg (.clk(clk), .reset(reset), .Load(TempRegWrite), .Clear(1'b0), .Entrada(mdr_out), .Saida(temp_reg_out));

    // Lógica de Manipulação de Bytes
    wire signed [31:0] byte_extended = {{24{mdr_out[7]}}, mdr_out[7:0]};

    // Lógica do Banco de Registradores
    assign slt_result = {31'b0, ula_menor};
    mux_RegDst mux_write_reg (.in1(ir_rt), .in2(ir_rd), .sel(RegDst[0]), .out(write_reg_mux_out));
    assign write_data_mux_out = (WBDataSrc == 3'b001) ? mdr_out : (WBDataSrc == 3'b100) ? byte_extended : (WBDataSrc == 3'b010) ? hi_out : (WBDataSrc == 3'b011) ? lo_out : (WBDataSrc == 3'b101) ? slt_result : alu_out_reg;
    wire banco_reg_reset = reset || RegsClear;
    Banco_reg banco_registradores (.Clk(clk), .Reset(banco_reg_reset), .RegWrite(RegWrite), .ReadReg1(ir_rs), .ReadReg2(ir_rt), .WriteReg((RegDst == 2'b10) ? 5'd31 : write_reg_mux_out), .WriteData(write_data_mux_out), .ReadData1(read_data_1), .ReadData2(read_data_2));
    
    // Lógica da ALU
    SingExtend_16x32 sign_extender (.in1(ir_immediate), .out(sign_extended_imm));
    wire signed [31:0] alu_in_a = ALUSrcA ? reg_a_out : pc_out;
    wire signed [31:0] lui_result = {ir_immediate, 16'b0};
    wire signed [31:0] shifted_b_reg = (ALUOp == 4'b1000) ? ($signed(reg_b_out) << ((ir_opcode == 6'b000001) ? mdr_out[4:0] : ir_shamt)) : ($signed(reg_b_out) >>> ir_shamt);
    wire signed [31:0] alu_in_b;
    mux_ALUsrc alu_src_b_mux (.reg_b_data(reg_b_out), .constant_4(32'd4), .sign_ext_imm(sign_extended_imm), .shifted_imm(sign_extended_imm << 2), .sel(ALUSrcB), .out(alu_in_b));
    Ula32 ula (.A(alu_in_a), .B(alu_in_b), .Seletor(ALUOp[2:0]), .S(alu_result_from_ula), .z(alu_zero), .Overflow(ula_overflow), .Negativo(ula_negativo), .Igual(ula_igual), .Maior(ula_maior), .Menor(ula_menor));
    assign alu_result = (ALUOp == 4'b1100) ? lui_result : (ALUOp[3]) ? shifted_b_reg : alu_result_from_ula;

    // Unidades de Multiplicação/Divisão
    multiplier mult_unit (.a(reg_a_out), .b(reg_b_out), .start(MultStart), .clk(clk), .reset(reset), .result(mult_result), .done(mult_done));
    divider div_unit (.a(reg_a_out), .b(reg_b_out), .start(DivStart), .clk(clk), .reset(reset), .quotient(div_quotient), .remainder(div_remainder), .done(div_done), .div_by_zero(div_by_zero_flag));
    wire signed [31:0] hi_in_data  = (ir_opcode == 6'b000000 && ir_funct == 6'b011000) ? mult_result[63:32] : div_remainder;
    wire signed [31:0] lo_in_data  = (ir_opcode == 6'b000000 && ir_funct == 6'b011000) ? mult_result[31:0]  : div_quotient;
    wire hi_lo_reset = reset || RegsClear;
    hi_lo_registers hi_lo_regs (.clk(clk), .reset(hi_lo_reset), .hi_in(hi_in_data), .lo_in(lo_in_data), .hi_write(HIWrite), .lo_write(LOWrite), .hi_out(hi_out), .lo_out(lo_out));
    
    // Instanciação da FSM
    control_unit FSM (
        .clk(clk), .reset(reset), .opcode(ir_opcode), .funct(ir_funct), .mult_done_in(mult_done), .div_done_in(div_done),
        .PCWrite(PCWrite), .PCWriteCond(PCWriteCond), .PCWriteCondNeg(PCWriteCondNeg), .IorD(IorD), .MemRead(MemRead),
        .MemWrite(MemWrite), .IRWrite(IRWrite), .RegWrite(RegWrite), .RegDst(RegDst), .ALUSrcA(ALUSrcA), .ALUSrcB(ALUSrcB),
        .PCSource(PCSource), .ALUOp(ALUOp), .HIWrite(HIWrite), .LOWrite(LOWrite), .MultStart(MultStart), .DivStart(DivStart),
        .WBDataSrc(WBDataSrc), .MemDataInSrc(MemDataInSrc),
        .PCClear(PCClear), .RegsClear(RegsClear),
        .TempRegWrite(TempRegWrite), .MemAddrSrc(MemAddrSrc), .MemDataSrc(MemDataSrc)
    );
endmodule