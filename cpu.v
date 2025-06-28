// cpu.v
// Módulo Top-Level FINAL e POLIDO.
module cpu(
    input wire clk,
    input wire reset
);
    // Control Signals
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
    wire        ByteOpEnable;

    // Datapath Wires
    wire [31:0] pc_out, next_pc, mem_data_out, alu_out_reg, mdr_out;
    wire [31:0] reg_a_out, reg_b_out;
    wire [31:0] read_data_1, read_data_2, sign_extended_imm, alu_result, write_data_mux_out;
    wire [4:0]  write_reg_mux_out;
    wire        alu_zero;
    wire [31:0] hi_out, lo_out;
    wire        mult_done, div_done;

    // IR fields
    wire [5:0]  ir_opcode;
    wire [4:0]  ir_rs, ir_rt, ir_rd, ir_shamt;
    wire [15:0] ir_immediate;
    wire [5:0]  ir_funct;
    wire [25:0] ir_jump_addr;

    // PC Logic
    assign next_pc = (PCSource == 2'b00) ? alu_result : 
                     (PCSource == 2'b01) ? alu_out_reg :
                     (PCSource == 2'b10) ? {pc_out[31:28], ir_jump_addr, 2'b00} : 
                     reg_a_out; // PCSource == 2'b11 for JR
    wire cond_taken = (PCWriteCond && alu_zero) || (PCWriteCondNeg && ~alu_zero);
    wire pc_write_enable = PCWrite || cond_taken;

    registrador #(32) pc_reg (.clk(clk), .reset(reset), .Load(pc_write_enable), .Entrada(next_pc), .Saida(pc_out));

    // Memory Logic
    wire [31:0] mem_address = IorD ? alu_out_reg : pc_out;

    // Logic for sb (Read-Modify-Write)
    wire [31:0] modified_mdr;
    wire [3:0] byte_select = 4'b1 << alu_out_reg[1:0]; // Creates a one-hot mask
    assign modified_mdr = (byte_select[0]) ? {mdr_out[31:8],  reg_b_out[7:0]} :
                          (byte_select[1]) ? {mdr_out[31:16], reg_b_out[7:0], mdr_out[7:0]} :
                          (byte_select[2]) ? {mdr_out[31:24], reg_b_out[7:0], mdr_out[15:0]} :
                          {reg_b_out[7:0], mdr_out[23:0]};
    wire [31:0] mem_datain = (MemDataInSrc == 1'b0) ? reg_b_out : modified_mdr;
    
    Memoria memoria (.Address(mem_address), .Clock(clk), .Wr(MemWrite), .Datain(mem_datain), .Dataout(mem_data_out));

    // Instruction Register and Decode
    Instr_Reg ir_unit (.Clk(clk), .Reset(reset), .Load_ir(IRWrite), .Entrada(mem_data_out), .Instr31_26(ir_opcode), .Instr25_21(ir_rs), .Instr20_16(ir_rt), .Instr15_0(ir_immediate));
    assign ir_rd = ir_immediate[15:11];
    assign ir_shamt = ir_immediate[10:6];
    assign ir_funct = ir_immediate[5:0];
    assign ir_jump_addr = {ir_rs, ir_rt, ir_rd, ir_shamt, ir_funct}; // Formato J

    // Internal Registers A, B, MDR, ALUOut
    registrador #(32) mdr_reg (.clk(clk), .reset(reset), .Load(1'b1), .Entrada(mem_data_out), .Saida(mdr_out));
    registrador #(32) reg_a (.clk(clk), .reset(reset), .Load(1'b1), .Entrada(read_data_1), .Saida(reg_a_out));
    registrador #(32) reg_b (.clk(clk), .reset(reset), .Load(1'b1), .Entrada(read_data_2), .Saida(reg_b_out));
    registrador #(32) alu_out_reg_inst (.clk(clk), .reset(reset), .Load(1'b1), .Entrada(alu_result), .Saida(alu_out_reg));

    // Logic for lb (Load Byte)
    wire [7:0] byte_from_mdr;
    assign byte_from_mdr = (alu_out_reg[1:0] == 2'b00) ? mdr_out[7:0] :
                           (alu_out_reg[1:0] == 2'b01) ? mdr_out[15:8] :
                           (alu_out_reg[1:0] == 2'b10) ? mdr_out[23:16] :
                           mdr_out[31:24];
    wire signed [31:0] byte_extended = {{24{byte_from_mdr[7]}}, byte_from_mdr}; // Sign extension

    // Register File Write-Back Logic
    mux_RegDst mux_write_reg (.in1(ir_rt), .in2(ir_rd), .sel(RegDst[0]), .out(write_reg_mux_out));
    
    assign write_data_mux_out = (WBDataSrc == 3'b000) ? alu_out_reg :
                                (WBDataSrc == 3'b001) ? mdr_out :
                                (WBDataSrc == 3'b010) ? hi_out :
                                (WBDataSrc == 3'b011) ? lo_out :
                                (WBDataSrc == 3'b100) ? byte_extended :
                                32'hxxxxxxxx;

    Banco_reg banco_registradores (.Clk(clk), .Reset(reset), .RegWrite(RegWrite), .ReadReg1(ir_rs), .ReadReg2(ir_rt), .WriteReg((RegDst == 2'b10) ? 5'd31 : write_reg_mux_out), .WriteData(write_data_mux_out), .ReadData1(read_data_1), .ReadData2(read_data_2));
    
    // ALU Logic
    SingExtend_16x32 sign_extender (.in1(ir_immediate), .out(sign_extended_imm));
    wire [31:0] alu_in_a = ALUSrcA ? reg_a_out : pc_out;
    
    // Shifter logic for sll/sra
    wire [31:0] shifted_b_reg;
    assign shifted_b_reg = (ALUOp == 4'b1000) ? (reg_b_out << ir_shamt) : // sll
                                               ($signed(reg_b_out) >>> ir_shamt); // sra
    
    wire [31:0] alu_in_b;
    mux_ALUsrc alu_src_b_mux (.reg_b_data(reg_b_out), .constant_4(32'd4), .sign_ext_imm(sign_extended_imm), .shifted_imm(sign_extended_imm << 2), .sel(ALUSrcB), .out(alu_in_b));
    
    wire [31:0] alu_result_pre_shift = (ALUOp[3]) ? shifted_b_reg : alu_result_from_ula; // Mux for shift result
    
    Ula32 ula (.A(alu_in_a), .B(alu_in_b), .Seletor(ALUOp[2:0]), .S(alu_result_from_ula), .z(alu_zero));
    assign alu_result = alu_result_pre_shift;

    // Multiplier/Divider Units
    multiplier mult_unit (.a(reg_a_out), .b(reg_b_out), .start(MultStart), .clk(clk), .reset(reset), .result({hi_out, lo_out}), .done(mult_done));
    divider div_unit (.a(reg_a_out), .b(reg_b_out), .start(DivStart), .clk(clk), .reset(reset), .quotient(lo_out), .remainder(hi_out), .done(div_done));
    
    // HI/LO registers no longer needed as mult/div write directly
    
    // FSM Instantiation
    control_unit FSM (
        .clk(clk), .reset(reset), .opcode(ir_opcode), .funct(ir_funct), .mult_done_in(mult_done), .div_done_in(div_done),
        .PCWrite(PCWrite), .PCWriteCond(PCWriteCond), .PCWriteCondNeg(PCWriteCondNeg), .IorD(IorD), .MemRead(MemRead),
        .MemWrite(MemWrite), .IRWrite(IRWrite), .RegWrite(RegWrite), .RegDst(RegDst), .ALUSrcA(ALUSrcA), .ALUSrcB(ALUSrcB),
        .PCSource(PCSource), .ALUOp(ALUOp), .HIWrite(HIWrite), .LOWrite(LOWrite), .MultStart(MultStart), .DivStart(DivStart),
        .InvalidOpcodeOut(), .WBDataSrc(WBDataSrc), .MemDataInSrc(MemDataInSrc), .ByteOpEnable(ByteOpEnable)
    );
endmodule