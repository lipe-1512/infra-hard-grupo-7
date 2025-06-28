/*
Top-level module for the multi-cycle CPU designed to match the architecture diagram provided.

This module connects the following components:
- Program Counter (PC)
- Instruction Memory
- Instruction Register
- Control Unit
- Register Bank
- ALU
- Memory Data
- Multiplexers for data path control
- Shift units
- Multiplier and Divider units
- HI and LO registers
- Exception and Overflow handling

Note: Signal widths and connections are based on existing modules and the architecture diagram.
*/

module top_level_cpu (
    input wire clk,
    input wire reset
);

    // Program Counter and next PC logic
    wire [31:0] pc_out;
    wire [31:0] pc_plus_4;
    wire [31:0] branch_target;
    wire [31:0] jump_target;
    wire [31:0] next_pc;

    // Instruction memory output
    wire [31:0] instruction;

    // Instruction Register outputs
    wire [5:0] opcode;
    wire [4:0] rs;
    wire [4:0] rt;
    wire [4:0] rd;
    wire [15:0] immediate;
    wire [5:0] funct;

    // Control signals
    wire RegWrite, MemWrite, MemRead, MemtoReg, ALUSrc, RegDst, Branch, Jump;
    wire [2:0] ALUOp;
    wire [2:0] state;

    // Register Bank signals
    wire [31:0] reg_data1, reg_data2;
    wire [4:0] write_reg;
    wire [31:0] write_data;

    // ALU signals
    wire [31:0] alu_src2;
    wire [31:0] alu_result;
    wire overflow;
    wire zero;

    // Memory data signals
    wire [31:0] mem_data_out;

    // HI and LO registers
    wire [31:0] hi_data, lo_data;

    // Exception signals
    wire exception;

    // PC + 4 calculation
    assign pc_plus_4 = pc_out + 4;

    // Branch target calculation (shift left 2 and add to PC+4)
    wire [31:0] immediate_ext;
    wire [31:0] immediate_shifted;
    SingExtend_16x32 sign_ext_inst (
        .in1(immediate),
        .out(immediate_ext)
    );

    assign immediate_shifted = immediate_ext << 2;
    assign branch_target = pc_plus_4 + immediate_shifted;

    // Jump target calculation
    assign jump_target = {pc_plus_4[31:28], instruction[25:0] << 2};

    // Next PC muxes
    wire pc_src_branch;
    assign pc_src_branch = Branch & zero;

    wire [31:0] pc_after_branch;
    mux_PC mux_branch (
        .in1(pc_plus_4),
        .in2(branch_target),
        .sel(pc_src_branch),
        .out(pc_after_branch)
    );

    mux_PC mux_jump (
        .in1(pc_after_branch),
        .in2(jump_target),
        .sel(Jump),
        .out(next_pc)
    );

    // Program Counter
    PC pc_inst (
        .clk(clk),
        .reset(reset),
        .pc_out(pc_out)
    );

    // Instruction Memory
    Memoria mem_inst (
        .Address(pc_out),
        .Clock(clk),
        .Wr(1'b0),
        .Datain(32'b0),
        .Dataout(instruction)
    );

    // Instruction Register
    Instr_Reg instr_reg_inst (
        .Clk(clk),
        .Reset(reset),
        .Load_ir(1'b1), // This should be controlled by state machine, simplified here
        .Entrada(instruction),
        .Instr31_26(opcode),
        .Instr25_21(rs),
        .Instr20_16(rt),
        .Instr15_0(immediate)
    );

    assign funct = instruction[5:0];

    // Control Unit
    control_unit control_inst (
        .clk(clk),
        .reset(reset),
        .opcode(opcode),
        .funct(funct),
        .Overflow(overflow),
        .RegWrite(RegWrite),
        .MemWrite(MemWrite),
        .MemRead(MemRead),
        .MemtoReg(MemtoReg),
        .ALUSrc(ALUSrc),
        .RegDst(RegDst),
        .Branch(Branch),
        .Jump(Jump),
        .ALUOp(ALUOp),
        .state(state)
    );

    // Register Bank
    Banco_reg reg_bank_inst (
        .Clk(clk),
        .Reset(reset),
        .RegWrite(RegWrite),
        .ReadReg1(rs),
        .ReadReg2(rt),
        .WriteReg(write_reg),
        .WriteData(write_data),
        .ReadData1(reg_data1),
        .ReadData2(reg_data2)
    );

    // Write register mux
    mux_RegDst mux_reg_dst (
        .in1(rt),
        .in2(rd),
        .sel(RegDst),
        .out(write_reg)
    );

    // ALU source mux
    mux_ALUsrc mux_alu_src (
        .in1(reg_data2),
        .in2(immediate_ext),
        .sel(ALUSrc),
        .out(alu_src2)
    );

    // ALU
    ula32 alu_inst (
        .A(reg_data1),
        .B(alu_src2),
        .Seletor(ALUOp),
        .S(alu_result),
        .Overflow(overflow),
        .Negativo(),
        .z(zero),
        .Igual(),
        .Maior(),
        .Menor()
    );

    // Memory Data
    Memoria mem_data_inst (
        .Address(alu_result),
        .Clock(clk),
        .Wr(MemWrite),
        .Datain(reg_data2),
        .Dataout(mem_data_out)
    );

    // MemtoReg mux
    mux_MemtoReg mux_mem_to_reg (
        .in1(alu_result),
        .in2(mem_data_out),
        .sel(MemtoReg),
        .out(write_data)
    );

    // HI and LO registers (simplified connections)
    hi_lo_registers hi_lo_inst (
        .clk(clk),
        .reset(reset),
        .hi_in(alu_result), // Example connection
        .lo_in(alu_result),
        .hi_out(hi_data),
        .lo_out(lo_data)
    );

    // Multiplier
    multiplier mult_inst (
        .clk(clk),
        .reset(reset),
        .start(state == EXECUTE && ALUOp == 3'b101), // Assuming 3'b101 is multiply
        .A(reg_data1),
        .B(reg_data2),
        .product_hi(hi_data),
        .product_lo(lo_data),
        .done()
    );

    // Divider
    divider div_inst (
        .clk(clk),
        .reset(reset),
        .start(state == EXECUTE && ALUOp == 3'b110), // Assuming 3'b110 is divide
        .A(reg_data1),
        .B(reg_data2),
        .quotient(lo_data),
        .remainder(hi_data),
        .done()
    );

    // Exception handling
    exception exception_inst (
        .clk(clk),
        .reset(reset),
        .overflow(overflow),
        .exception(exception)
    );

    // Shift units
    shift_left_2 shift_left_inst (
        .in(immediate_ext),
        .out(immediate_shifted)
    );

    RegDesloc shift_reg_inst (
        .in(reg_data2),
        .shift_amt(instruction[10:6]),
        .out()
    );

    SingExtend_1x32 sign_ext_1x32_inst (
        .in1(instruction[15:0]),
        .out()
    );

    // Other multiplexers and connections as per diagram to be added here

endmodule
