`include "cpu_components.v"

// Este módulo de topo implementa estruturalmente o caminho de dados do diagrama,
// instanciando e conectando todos os componentes necessários.
module mips_multicycle_cpu (
    input wire clk,
    input wire reset
);
    // Sinais de Controle da nova Unidade de Controle
    wire PCWrite, IorD, MemRead, MemWrite, IRWrite, RegWrite;
    wire [1:0] MemtoReg, RegDst;
    wire AWrite, BWrite, ALUOutWrite;
    wire [1:0] ALUSrcA, ALUSrcB;
    wire [2:0] ALUOp;
    wire [1:0] PCSrc;

    // Fios do Caminho de Dados (Datapath)
    wire [31:0] pc_in, pc_out;
    wire [31:0] instruction;
    wire [31:0] mem_addr, mem_data_out;
    wire [31:0] reg_a_out, reg_b_out;
    wire [31:0] alu_out_s, alu_out_reg_out;
    wire [31:0] mdr_out; // Saída do Registrador de Dados da Memória (MDR)
    wire [31:0] sign_ext_out, sign_ext_shifted_out;
    wire [31:0] reg_read_data1, reg_read_data2;
    wire [31:0] reg_write_data;
    wire [31:0] alu_in_a, alu_in_b;
    wire [31:0] jump_addr;
    wire [31:0] pc_plus_4;
    
    wire [4:0] write_reg_addr;
    
    // Campos da Instrução Decodificada
    wire [5:0] op;
    wire [4:0] rs, rt, rd;
    wire [5:0] funct;
    wire [15:0] imm;
    
    wire alu_zero_flag;

    //================================================================
    // 1. Estágio de Busca de Instrução (PC e Fetch)
    //================================================================
    
    // Registrador PC (Program Counter)
    Registrador pc_reg (
        .Clk(clk), .Reset(reset),
        .Load(PCWrite),
        .Entrada(pc_in),
        .Saida(pc_out)
    );
    
    // Somador para PC+4 (usado no cálculo do endereço de jump)
    assign pc_plus_4 = pc_out + 32'd4;
    
    // MUX IorD: Seleciona entre PC e ALUOut para o endereço de memória
    mux2_1_32b iord_mux (
        .d0(pc_out),
        .d1(alu_out_reg_out),
        .sel(IorD),
        .y(mem_addr)
    );

    // Memória Principal (Instruções e Dados)
    Memoria main_mem (
        .Address(mem_addr),
        .Clock(clk),
        .Wr(MemWrite),
        .Datain(reg_b_out), // O dado para escrita vem do Registrador B
        .Dataout(mem_data_out)
    );
    
    // Registrador de Instrução (IR)
    // A saída da memória é tratada como o barramento de instrução.
    // O módulo IR abaixo armazena o valor desse barramento quando IRWrite é ativado.
    Instr_Reg ir_unit (
        .Clk(clk), .Reset(reset),
        .Load_ir(IRWrite),
        .Entrada(mem_data_out),
        .Instr31_26(op),
        .Instr25_21(rs),
        .Instr20_16(rt),
        .Instr15_0(imm)
    );
    // Conexões diretas da saída da memória para decodificação
    assign rd = mem_data_out[15:11]; 
    assign funct = mem_data_out[5:0];

    //================================================================
    // 2. Estágio de Decodificação / Leitura de Registradores
    //================================================================
    
    // Unidade de Controle (FSM)
    cpu_control_unit control_unit (
        .clk(clk), .reset(reset), .opcode(op), .funct(funct),
        .PCWrite(PCWrite), .IorD(IorD), .MemRead(MemRead), .MemWrite(MemWrite),
        .IRWrite(IRWrite), .RegWrite(RegWrite), .MemtoReg(MemtoReg),
        .RegDst(RegDst), .AWrite(AWrite), .BWrite(BWrite),
        .ALUOutWrite(ALUOutWrite), .ALUSrcA(ALUSrcA), .ALUSrcB(ALUSrcB),
        .ALUOp(ALUOp), .PCSrc(PCSrc)
    );

    // Banco de Registradores
    Banco_reg reg_bank (
        .Clk(clk), .Reset(reset),
        .RegWrite(RegWrite),
        .ReadReg1(rs),
        .ReadReg2(rt),
        .WriteReg(write_reg_addr),
        .WriteData(reg_write_data),
        .ReadData1(reg_read_data1),
        .ReadData2(reg_read_data2)
    );

    // Registradores A e B (armazenam as saídas do banco de registradores)
    Registrador reg_a (.Clk(clk), .Reset(reset), .Load(AWrite), .Entrada(reg_read_data1), .Saida(reg_a_out));
    Registrador reg_b (.Clk(clk), .Reset(reset), .Load(BWrite), .Entrada(reg_read_data2), .Saida(reg_b_out));

    // Extensor de Sinal e Deslocador para valores imediatos
    SingExtend_16x32 sign_ext (.in1(imm), .out(sign_ext_out));
    shift_left_2 shift_left_imm (.in(sign_ext_out), .out(sign_ext_shifted_out));

    //================================================================
    // 3. Estágio de Execução
    //================================================================

    // Registrador ALUOut (armazena o resultado da ULA)
    Registrador reg_alu_out (
        .Clk(clk), .Reset(reset),
        .Load(ALUOutWrite),
        .Entrada(alu_out_s),
        .Saida(alu_out_reg_out)
    );

    // MUX para a Fonte A da ULA
    mux4_1_32b alu_src_a_mux (
        .d0(pc_out), .d1(reg_a_out), .d2(32'b0), .d3(32'b0), // d2, d3 não usados
        .sel(ALUSrcA),
        .y(alu_in_a)
    );

    // MUX para a Fonte B da ULA
    mux4_1_32b alu_src_b_mux (
        .d0(reg_b_out), .d1(32'd4), .d2(sign_ext_out), .d3(sign_ext_shifted_out),
        .sel(ALUSrcB),
        .y(alu_in_b)
    );
    
    // ULA (Unidade Lógica e Aritmética)
    Ula32 alu_unit (
        .A(alu_in_a), .B(alu_in_b),
        .Seletor(ALUOp),
        .S(alu_out_s),
        .z(alu_zero_flag),
        .Overflow(), .Negativo(), .Igual(), .Maior(), .Menor()
    );

    //================================================================
    // 4. Estágio de Acesso à Memória / Escrita de Volta (Write Back)
    //================================================================

    // Registrador de Dados da Memória (MDR)
    Registrador reg_mdr (
        .Clk(clk), .Reset(reset),
        .Load(MemRead), // Armazena o valor lido da memória
        .Entrada(mem_data_out),
        .Saida(mdr_out)
    );
    
    // MUX para o dado que será escrito de volta no banco de registradores
    mux4_1_32b mem_to_reg_mux (
        .d0(alu_out_reg_out), // Do resultado da ULA
        .d1(mdr_out),         // Do dado vindo da memória
        .d2(pc_plus_4),       // Para JAL (PC+4 -> $ra), necessita extensão da FSM
        .d3(32'b0),           // Não usado
        .sel(MemtoReg),
        .y(reg_write_data)
    );

    // MUX para o endereço do registrador de destino
    mux2_1_5b reg_dst_mux (
        .d0(rt),
        .d1(rd),
        .sel(RegDst[0]), // MUX 2-para-1 simples, como no diagrama
        .y(write_reg_addr)
    );

    //================================================================
    // 5. Lógica de Cálculo do Próximo PC
    //================================================================
    
    // Cálculo do Endereço de Jump
    assign jump_addr = {pc_plus_4[31:28], mem_data_out[25:0], 2'b00};

    // MUX para selecionar o valor do próximo PC
    // Nota: A escrita no PC para branches (desvios) é condicional.
    // Para BEQ, PCWrite só deve ser ativado se alu_zero_flag for 1.
    // Esta lógica é simplificada na FSM e na seleção do MUX.
    mux4_1_32b pc_src_mux (
        .d0(alu_out_s),         // Resultado do cálculo de PC+4
        .d1(alu_out_reg_out),   // Endereço de branch vindo do registrador ALUOut
        .d2(jump_addr),         // Endereço para instrução de jump
        .d3(reg_a_out),         // Endereço para instrução jr (jump register)
        .sel(PCSrc),
        .y(pc_in)
    );

endmodule