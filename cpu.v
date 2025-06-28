// cpu.v
// Módulo Top-Level que conecta todos os componentes do datapath multi-ciclo.
module cpu(
    input wire clk,
    input wire reset
);

    // --- Sinais de Controle ---
    wire PCWrite, PCWriteCond, IorD, MemRead, MemWrite, IRWrite, RegWrite;
    wire MemtoReg, RegDst, ALUSrcA;
    wire [1:0] ALUSrcB, PCSource;
    wire [3:0] ALUOp;
    
    // --- Fios e Barramentos do Datapath ---
    wire [31:0] pc_out, next_pc, instruction;
    wire [31:0] mem_data_out, alu_out_reg, mdr_out;
    wire [31:0] reg_a_out, reg_b_out;
    wire [31:0] read_data_1, read_data_2;
    wire [31:0] sign_extended_imm, zero_extended_imm;
    wire [31:0] alu_result;
    wire        alu_zero;
    wire [31:0] write_data_mux_out;
    wire [4:0]  write_reg_mux_out;
    
    // --- PC e Lógica de Próximo PC ---
    assign next_pc = (PCSource == 2'b00) ? alu_result :         // ALU (PC+4)
                     (PCSource == 2'b01) ? alu_out_reg :        // ALUOut (Branch)
                     (PCSource == 2'b10) ? {{pc_out[31:28]}, {instruction[25:0], 2'b00}} : // Jump
                                           reg_a_out;            // Jump Register

    // PC (Program Counter)
    Registrador #(32) pc_reg (
        .Clk(clk), .Reset(reset), .Load(PCWrite),
        .Entrada(next_pc), .Saida(pc_out)
    );

    // --- Memória (Instruções e Dados) ---
    wire [31:0] mem_address = IorD ? alu_out_reg : pc_out;
    Memoria memoria (
        .Address(mem_address), .Clock(clk), .Wr(MemWrite),
        .Datain(reg_b_out), .Dataout(mem_data_out)
    );

    // --- Registradores Intermediários ---
    // Instruction Register (IR)
    wire [31:0] ir_in = mem_data_out;
    // O registrador de instruções é implementado dentro da Unidade de Controle
    // para simplificar, mas os sinais de saída são extraídos aqui.
    assign instruction = ir_in; 

    // Memory Data Register (MDR)
    Registrador #(32) mdr_reg (
        .Clk(clk), .Reset(reset), .Load(MemRead), // Carrega na borda de subida após a leitura
        .Entrada(mem_data_out), .Saida(mdr_out)
    );
    
    // Registrador A (lê rs)
    Registrador #(32) reg_a (
        .Clk(clk), .Reset(reset), .Load(1'b1), // Sempre carrega
        .Entrada(read_data_1), .Saida(reg_a_out)
    );

    // Registrador B (lê rt)
    Registrador #(32) reg_b (
        .Clk(clk), .Reset(reset), .Load(1'b1), // Sempre carrega
        .Entrada(read_data_2), .Saida(reg_b_out)
    );
    
    // Registrador ALUOut
    Registrador #(32) alu_out (
        .Clk(clk), .Reset(reset), .Load(1'b1), // Sempre carrega
        .Entrada(alu_result), .Saida(alu_out_reg)
    );
    
    // --- Banco de Registradores ---
    mux_RegDst mux_write_reg (
        .in1(instruction[20:16]), // rt
        .in2(instruction[15:11]), // rd
        .sel(RegDst),
        .out(write_reg_mux_out)
    );
    
    mux_MemtoReg mux_write_data (
        .in1(alu_out_reg),
        .in2(mdr_out),
        .sel(MemtoReg),
        .out(write_data_mux_out)
    );

    Banco_reg banco_registradores (
        .Clk(clk), .Reset(reset), .RegWrite(RegWrite),
        .ReadReg1(instruction[25:21]), // rs
        .ReadReg2(instruction[20:16]), // rt
        .WriteReg(write_reg_mux_out),
        .WriteData(write_data_mux_out),
        .ReadData1(read_data_1),
        .ReadData2(read_data_2)
    );

    // --- Lógica da ALU ---
    SingExtend_16x32 sign_extender (
        .in1(instruction[15:0]),
        .out(sign_extended_imm)
    );
    
    wire [31:0] alu_in_a = ALUSrcA ? reg_a_out : pc_out;
    
    wire [31:0] alu_in_b;
    assign alu_in_b = (ALUSrcB == 2'b00) ? reg_b_out :
                      (ALUSrcB == 2'b01) ? 32'd4 :
                      (ALUSrcB == 2'b10) ? sign_extended_imm :
                                           (sign_extended_imm << 2);

    // Instanciação da ULA fornecida em VHDL
    Ula32 ula(
        .A(alu_in_a), .B(alu_in_b),
        .Seletor({1'b0, ALUOp[2:0]}), // A ULA do PDF tem seletor de 3 bits
        .S(alu_result),
        .z(alu_zero),
        .Overflow(), .Negativo(), .Igual(), .Maior(), .Menor() // Flags não usadas diretamente aqui
    );
    
    // --- Unidade de Controle ---
    // O sinal PCWriteCond depende do flag 'zero' da ALU
    assign PCWrite = PCWriteCond ? alu_zero : 1'b1;

    control_unit FSM (
        .clk(clk), .reset(reset),
        .opcode(instruction[31:26]),
        .funct(instruction[5:0]),
        // Saídas de Controle
        .PCWrite(PCWrite), .PCWriteCond(PCWriteCond),
        .IorD(IorD), .MemRead(MemRead), .MemWrite(MemWrite), .IRWrite(IRWrite),
        .RegWrite(RegWrite), .MemtoReg(MemtoReg), .RegDst(RegDst),
        .ALUSrcA(ALUSrcA), .ALUSrcB(ALUSrcB), .PCSource(PCSource),
        .ALUOp(ALUOp)
    );

endmodule