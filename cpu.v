module cpu(
    input wire clk
);
    // Geração de reset para simulação
    reg reset;
    initial begin
        reset = 1'b1; #50; reset = 1'b0;
    end
    
    // --- Sinais de Controle da FSM ---
    wire        PCWrite, PCWriteCond, PCWriteCondNeg;
    wire        IorD, MemRead, MemWrite, IRWrite, RegWrite;
    wire [1:0]  RegDst;
    wire        ALUSrcA;
    wire [1:0]  ALUSrcB;
    wire [1:0]  PCSource;
    wire [3:0]  ALUOp;
    wire        HIWrite, LOWrite, MultStart, DivStart;
    wire [2:0]  WBDataSrc;
    wire        MemAddrSrc;
    wire        MemDataInSrc;
    wire        PCClear;
    wire        RegsClear;
    wire        TempRegWrite;
    wire        MemtoRegA;
    // Sinais de exceção da FSM original
    wire [2:0]  exception_code;
    wire [31:0] epc_out;
    wire [31:0] new_pc_from_exception;

    // --- Registradores de Estado e Wires do Datapath ---
    reg signed [31:0] pc;
    wire signed [31:0] pc_out = pc;
    wire signed [31:0] next_pc;

    wire cond_taken, pc_write_enable;

    reg signed [31:0] reg_a_out, reg_b_out, alu_out_reg, mdr_out;

    wire signed [31:0] mem_data_out, read_data_1, read_data_2, sign_extended_imm, alu_result, alu_result_from_ula;
    wire signed [31:0] write_data_mux_out;
    wire [4:0]  write_reg_mux_out;
    wire        alu_zero, ula_overflow, ula_negativo, ula_igual, ula_maior, ula_menor;
    wire signed [63:0] mult_result;
    wire signed [31:0] div_quotient, div_remainder;
    wire        mult_done, div_done, div_by_zero_flag;
    wire signed [31:0] hi_out, lo_out;
    wire signed [31:0] slt_result;

    // --- Campos do IR (Decodificação) ---
    wire [5:0]  ir_opcode;
    wire [4:0]  ir_rs, ir_rt, ir_rd, ir_shamt;
    wire [15:0] ir_immediate;
    wire [5:0]  ir_funct;
    wire [25:0] ir_jump_addr;

    // --- Lógica do PC (com 'registrador' substituído) ---
    always @(posedge clk or posedge reset) begin
        if (reset)
            pc <= 32'h00000000;
        else if (PCClear)
            pc <= 32'h00000000;
        else if (pc_write_enable)
            pc <= next_pc;
    end
    
    assign next_pc = (PCSource == 2'b00) ? alu_result : 
                     (PCSource == 2'b01) ? alu_out_reg :
                     (PCSource == 2'b10) ? {pc_out[31:28], ir_jump_addr, 2'b00} : 
                     reg_a_out;
    assign cond_taken = (PCWriteCond && alu_zero) || (PCWriteCondNeg && ~alu_zero);
    assign pc_write_enable = PCWrite || cond_taken;

    // --- Lógica da Memória ---
    wire [31:0] mem_address = IorD ? alu_out_reg : pc_out;
    wire [31:0] modified_mdr;
    assign modified_mdr = (alu_out_reg[1:0] == 2'b00) ? {mdr_out[31:8],  reg_b_out[7:0]} :
                          (alu_out_reg[1:0] == 2'b01) ? {mdr_out[31:16], reg_b_out[7:0], mdr_out[7:0]} :
                          (alu_out_reg[1:0] == 2'b10) ? {mdr_out[31:24], reg_b_out[7:0], mdr_out[15:0]} :
                                                        {reg_b_out[7:0], mdr_out[23:0]};
    wire [31:0] mem_datain = MemDataInSrc ? modified_mdr : reg_b_out;
    Memoria memoria (.Address(mem_address), .Clock(clk), .Wr(MemWrite), .Datain(mem_datain), .Dataout(mem_data_out));

    // --- Registrador de Instrução e Decodificação ---
    Instr_Reg ir_unit (.Clk(clk), .Reset(reset), .Load_ir(IRWrite), .Entrada(mem_data_out), .Instr31_26(ir_opcode), .Instr25_21(ir_rs), .Instr20_16(ir_rt), .Instr15_0(ir_immediate));
    assign ir_rd = ir_immediate[15:11];
    assign ir_shamt = ir_immediate[10:6];
    assign ir_funct = ir_immediate[5:0];
    assign ir_jump_addr = {ir_rs, ir_rt, ir_immediate}; 

    // --- Registradores Intermediários (com 'registrador' substituído) ---
    always @(posedge clk) begin
        if(reset) begin
            mdr_out <= 32'b0;
            reg_a_out <= 32'b0;
            reg_b_out <= 32'b0;
            alu_out_reg <= 32'b0;
        end else begin
            mdr_out <= mem_data_out;
            reg_a_out <= read_data_1;
            reg_b_out <= read_data_2;
            alu_out_reg <= alu_result;
        end
    end
    
    // --- Lógica de Manipulação de Byte ---
    wire [7:0] byte_from_mdr = (alu_out_reg[1:0] == 2'b00) ? mdr_out[7:0]   :
                               (alu_out_reg[1:0] == 2'b01) ? mdr_out[15:8]  :
                               (alu_out_reg[1:0] == 2'b10) ? mdr_out[23:16] : mdr_out[31:24];
    wire signed [31:0] byte_extended = {{24{byte_from_mdr[7]}}, byte_from_mdr};
    assign slt_result = {31'b0, ula_menor};

    // --- Lógica de Write-Back ---
    mux_RegDst mux_write_reg (.in1(ir_rt), .in2(ir_rd), .sel(RegDst[0]), .out(write_reg_mux_out));
    assign write_data_mux_out = (WBDataSrc == 3'b001) ? mdr_out :
                                (WBDataSrc == 3'b100) ? byte_extended :
                                (WBDataSrc == 3'b010) ? hi_out :
                                (WBDataSrc == 3'b011) ? lo_out :
                                (WBDataSrc == 3'b101) ? slt_result :
                                alu_out_reg;
    wire banco_reg_reset = reset || RegsClear;
    Banco_reg banco_registradores (.Clk(clk), .Reset(banco_reg_reset), .RegWrite(RegWrite), .ReadReg1(ir_rs), .ReadReg2(ir_rt), .WriteReg((RegDst == 2'b10) ? 5'd31 : write_reg_mux_out), .WriteData(write_data_mux_out), .ReadData1(read_data_1), .ReadData2(read_data_2));
    
    // --- Lógica da ALU ---
    SingExtend_16x32 sign_extender (.in1(ir_immediate), .out(sign_extended_imm));
    wire signed [31:0] alu_in_a = ALUSrcA ? ((ir_rs == 5'b0) ? 32'd0 : reg_a_out) : pc_out;
    wire signed [31:0] lui_result = sign_extended_imm << 16;
    wire signed [31:0] shifted_b_reg = (ALUOp == 4'b1000) ? ($signed(reg_b_out) << ir_shamt) : ($signed(reg_b_out) >>> ir_shamt);
    wire signed [31:0] alu_in_b;
    mux_ALUsrc alu_src_b_mux (.reg_b_data(reg_b_out), .constant_4(32'd4), .sign_ext_imm(sign_extended_imm), .shifted_imm(sign_extended_imm << 2), .sel(ALUSrcB), .out(alu_in_b));
    Ula32 ula (.A(alu_in_a), .B(alu_in_b), .Seletor(ALUOp[2:0]), .S(alu_result_from_ula), .z(alu_zero), .Overflow(ula_overflow), .Negativo(ula_negativo), .Igual(ula_igual), .Maior(ula_maior), .Menor(ula_menor));
    assign alu_result = (ALUOp == 4'b1100) ? lui_result : (ALUOp[3]) ? shifted_b_reg : alu_result_from_ula;

    // --- Unidades de Multiplicação e Divisão ---
    multiplier mult_unit (.a(reg_a_out), .b(reg_b_out), .start(MultStart), .clk(clk), .reset(reset), .result(mult_result), .done(mult_done));
    divider div_unit (.a(reg_a_out), .b(reg_b_out), .start(DivStart), .clk(clk), .reset(reset), .quotient(div_quotient), .remainder(div_remainder), .done(div_done), .div_by_zero(div_by_zero_flag));
    wire signed [31:0] hi_in_data  = (ir_opcode == 6'b0 && ir_funct == 6'b011000) ? mult_result[63:32] : div_remainder;
    wire signed [31:0] lo_in_data  = (ir_opcode == 6'b0 && ir_funct == 6'b011000) ? mult_result[31:0]  : div_quotient;
    wire hi_lo_reset = reset || RegsClear;
    hi_lo_registers hi_lo_regs (.clk(clk), .reset(hi_lo_reset), .hi_in(hi_in_data), .lo_in(lo_in_data), .hi_write(HIWrite), .lo_write(LOWrite), .hi_out(hi_out), .lo_out(lo_out));
    
    // --- Instanciação da FSM ---
    // CORRIGIDO: A instanciação agora corresponde exatamente à definição do módulo control_unit.
    control_unit FSM (
        .clk(clk), .reset(reset), .opcode(ir_opcode), .funct(ir_funct),
        .mult_done_in(mult_done), .div_done_in(div_done),
        .PCWrite(PCWrite), .PCWriteCond(PCWriteCond), .PCWriteCondNeg(PCWriteCondNeg),
        .IorD(IorD), .MemRead(MemRead), .MemWrite(MemWrite), .IRWrite(IRWrite),
        .RegWrite(RegWrite), .RegDst(RegDst), .ALUSrcA(ALUSrcA), .ALUSrcB(ALUSrcB),
        .PCSource(PCSource), .ALUOp(ALUOp), .HIWrite(HIWrite), .LOWrite(LOWrite),
        .MultStart(MultStart), .DivStart(DivStart), .WBDataSrc(WBDataSrc),
        .MemAddrSrc(MemAddrSrc), .MemDataInSrc(MemDataInSrc), .PCClear(PCClear),
        .RegsClear(RegsClear), .TempRegWrite(TempRegWrite), .MemtoRegA(MemtoRegA),
        .exception_code(exception_code), .epc(epc_out), .new_pc(new_pc_from_exception),
        .pc(pc)
    );
endmodule