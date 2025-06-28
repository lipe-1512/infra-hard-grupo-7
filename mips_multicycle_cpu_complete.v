`include "cpu_components_complete.v" // Incluindo a versão corrigida
`include "shifter_general.v"

module mips_multicycle_cpu_complete (
    input wire clk,
    input wire reset
);
    // Sinais de Controle da FSM expandida
    wire PCWrite, PCWriteCond, IorD, MemRead, MemWrite, IRWrite, RegWrite;
    wire AWrite, BWrite, ALUOutWrite, HIWrite, LOWrite, EPCWrite;
    wire [1:0] MemtoReg, RegDst;
    wire [1:0] ALUSrcA;
    wire [2:0] ALUSrcB;
    wire [2:0] ALUOp, ShifterOp;
    wire ALUResultSrc;
    wire [2:0] PCSrc;
    wire Start_mult_div;

    // Fios do Datapath
    wire [31:0] pc_in, pc_in_temp, pc_out, epc_out, exception_handler_addr;
    wire [31:0] mem_addr, mem_data_out, mdr_out;
    wire [31:0] reg_a_out, reg_b_out, reg_read_data1, reg_read_data2, reg_write_data;
    wire [31:0] alu_in_a, alu_in_b, alu_out_s, alu_out_reg_out;
    wire [31:0] shifter_out, alu_or_shifter_result;
    wire [31:0] hi_out, lo_out;
    wire [63:0] mult_result;
    wire [31:0] div_quotient, div_remainder;
    wire [31:0] sign_ext_out, sign_ext_shifted_out, jump_addr, pc_plus_4;
    wire [4:0]  write_reg_addr, shamt;
    wire [5:0]  op, funct;
    wire [4:0]  rs, rt, rd;
    wire [15:0] imm;
    wire alu_zero_flag, alu_overflow_flag, mult_done, div_done, exception_signal;

    //================================================================
    // Estágio 1 e 2: Fetch e Decode
    //================================================================
    Registrador pc_reg ( .Clk(clk), .Reset(reset), .Load(PCWrite | (PCWriteCond & alu_zero_flag)), .Entrada(pc_in), .Saida(pc_out) );
    assign pc_plus_4 = pc_out + 4;
    mux2_1_32b iord_mux ( .d0(pc_out), .d1(alu_out_reg_out), .sel(IorD), .y(mem_addr) );
    Memoria main_mem ( .Address(mem_addr), .Clock(clk), .Wr(MemWrite), .Datain(reg_b_out), .Dataout(mem_data_out) );
    Instr_Reg ir_unit ( .Clk(clk), .Reset(reset), .Load_ir(IRWrite), .Entrada(mem_data_out), .Instr31_26(op), .Instr25_21(rs), .Instr20_16(rt), .Instr15_0(imm) );
    assign rd = mem_data_out[15:11], funct = mem_data_out[5:0], shamt = mem_data_out[10:6];

    //================================================================
    // Componentes Adicionais e de Exceção
    //================================================================
    exception exc_unit ( .overflow(alu_overflow_flag), .div_by_zero(1'b0), .invalid_opcode(1'b0), .pc(pc_out), .exception(exception_signal), .new_pc(exception_handler_addr) );
    Registrador epc_reg ( .Clk(clk), .Reset(reset), .Load(EPCWrite), .Entrada(pc_out), .Saida(epc_out) );

    multiplier mult_unit ( .clk(clk), .a(reg_a_out), .b(reg_b_out), .start(Start_mult_div && (funct==6'b011000)), .result(mult_result), .done(mult_done) );
    divider div_unit ( .clk(clk), .a(reg_a_out), .b(reg_b_out), .start(Start_mult_div && (funct==6'b011010)), .quotient(div_quotient), .remainder(div_remainder), .done(div_done) );
    
    // *** LINHA COM ERRO REMOVIDA ***
    
    hi_lo_registers hi_lo_regs ( .clk(clk), .reset(reset), .hi_in( (funct==6'b011000) ? mult_result[63:32] : div_remainder ), .lo_in( (funct==6'b011000) ? mult_result[31:0] : div_quotient ), .hi_write(HIWrite), .lo_write(LOWrite), .hi_out(hi_out), .lo_out(lo_out) );

    //================================================================
    // Unidade de Controle
    //================================================================
    // *** INSTANCIAÇÃO ATUALIZADA ***
    cpu_control_unit_complete control_unit ( 
        .clk(clk), .reset(reset), .opcode(op), .funct(funct), 
        .mult_done(mult_done), .div_done(div_done), // Passando os sinais diretamente
        .exception_detected(exception_signal), 
        .PCWrite(PCWrite), .PCWriteCond(PCWriteCond), .IorD(IorD), .MemRead(MemRead), .MemWrite(MemWrite), 
        .IRWrite(IRWrite), .RegWrite(RegWrite), .AWrite(AWrite), .BWrite(BWrite), .ALUOutWrite(ALUOutWrite), 
        .HIWrite(HIWrite), .LOWrite(LOWrite), .EPCWrite(EPCWrite), .MemtoReg(MemtoReg), .RegDst(RegDst), 
        .ALUSrcA(ALUSrcA), .ALUSrcB(ALUSrcB), .ALUOp(ALUOp), .ShifterOp(ShifterOp), 
        .ALUResultSrc(ALUResultSrc), .PCSrc(PCSrc), .Start_mult_div(Start_mult_div) 
    );

    //================================================================
    // Datapath Principal
    //================================================================
    Banco_reg reg_bank ( .Clk(clk), .Reset(reset), .RegWrite(RegWrite), .ReadReg1(rs), .ReadReg2(rt), .WriteReg(write_reg_addr), .WriteData(reg_write_data), .ReadData1(reg_read_data1), .ReadData2(reg_read_data2) );
    Registrador reg_a (.Clk(clk), .Reset(reset), .Load(AWrite), .Entrada(reg_read_data1), .Saida(reg_a_out));
    Registrador reg_b (.Clk(clk), .Reset(reset), .Load(BWrite), .Entrada(reg_read_data2), .Saida(reg_b_out));

    SingExtend_16x32 sign_ext (.in1(imm), .out(sign_ext_out));
    shift_left_2 shift_left_imm (.in(sign_ext_out), .out(sign_ext_shifted_out));

    mux2_1_32b alu_src_a_mux ( .d0(pc_out), .d1(reg_a_out), .sel(ALUSrcA[0]), .y(alu_in_a) );
    mux4_1_32b alu_src_b_mux ( .d0(reg_b_out), .d1(32'd4), .d2(sign_ext_out), .d3(sign_ext_shifted_out), .sel(ALUSrcB[1:0]), .y(alu_in_b) );

    Ula32 alu_unit ( .A(alu_in_a), .B(alu_in_b), .Seletor(ALUOp), .S(alu_out_s), .z(alu_zero_flag), .Overflow(alu_overflow_flag) );
    shifter_general shifter ( .in(reg_b_out), .shamt(shamt), .sh_op(ShifterOp), .out(shifter_out) );
    mux2_1_32b alu_or_shifter_mux ( .d0(alu_out_s), .d1(shifter_out), .sel(ALUResultSrc), .y(alu_or_shifter_result) );
    Registrador reg_alu_out ( .Clk(clk), .Reset(reset), .Load(ALUOutWrite), .Entrada(alu_or_shifter_result), .Saida(alu_out_reg_out) );

    //================================================================
    // Estágio de Write-Back
    //================================================================
    Registrador reg_mdr ( .Clk(clk), .Reset(reset), .Load(MemRead), .Entrada(mem_data_out), .Saida(mdr_out) );
    mux4_1_32b mem_to_reg_mux ( .d0(alu_out_reg_out), .d1(mdr_out), .d2(hi_out), .d3(lo_out), .sel(MemtoReg), .y(reg_write_data) );
    mux2_1_5b reg_dst_mux ( .d0(rt), .d1(rd), .sel(RegDst[0]), .y(write_reg_addr) );

    //================================================================
    // Lógica de Atualização do PC
    //================================================================
    assign jump_addr = {pc_plus_4[31:28], mem_data_out[25:0], 2'b00};
    mux4_1_32b pc_src_mux ( .d0(alu_out_s), .d1(alu_out_reg_out), .d2(jump_addr), .d3(reg_a_out), .sel(PCSrc[1:0]), .y(pc_in_temp) );
    // MUX final para selecionar entre o próximo PC normal e o endereço de exceção
    mux2_1_32b pc_exc_mux ( .d0(pc_in_temp), .d1(exception_handler_addr), .sel(PCSrc[2]), .y(pc_in) );

endmodule