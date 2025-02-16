module pc (
    input wire clk,                // Clock global
    input wire reset,              // Sinal de reset
    output wire [31:0] pc_out      // Saída do Program Counter
);

    // Sinais internos
    wire [31:0] instruction;       // Instrução atual
    wire [31:0] reg_data1;         // Dado lido do registrador 1
    wire [31:0] reg_data2;         // Dado lido do registrador 2
    wire [31:0] alu_result;        // Resultado da ALU
    wire [31:0] mem_data_out;      // Dado lido da memória
    wire [31:0] immediate_ext;     // Valor imediato estendido
    wire [31:0] next_pc;           // Próximo valor do PC
    wire [31:0] branch_target;     // Endereço de branch
    wire [31:0] jump_target;       // Endereço de jump
    wire [31:0] alu_src2;          // Segundo operando da ALU
    wire [31:0] write_data;        // Dado a ser escrito no banco de registradores
    wire [4:0] write_reg;          // Registrador de destino
    wire [2:0] alu_control;        // Controle da ALU
    wire reg_write;                // Sinal de escrita no banco de registradores
    wire mem_write;                // Sinal de escrita na memória
    wire mem_read;                 // Sinal de leitura da memória
    wire mem_to_reg;               // Seleção de dado para escrita no registrador
    wire alu_src;                  // Seleção do segundo operando da ALU
    wire reg_dst;                  // Seleção do registrador de destino
    wire branch;                   // Sinal de branch
    wire jump;                     // Sinal de jump
    wire zero;                     // Sinal de zero da ALU
    wire overflow;                 // Sinal de overflow da ALU
    wire negativo;                 // Sinal de negativo da ALU
    wire igual;                    // Sinal de igualdade da ALU
    wire maior;                    // Sinal de maior da ALU
    wire menor;                    // Sinal de menor da ALU
    wire [2:0] state; // Declaração correta do sinal state (3 bits)

    // Módulo Program Counter
    registrador pc_reg (
        .clk(clk),
        .reset(reset),
        .load(1'b1),              // Sempre carrega o próximo valor
        .entrada(next_pc),
        .saida(pc_out)
    );

    // Memória de Instruções
    memoria mem (
        .Address(pc_out),
        .Clock(clk),
        .Wr(1'b0),                // Somente leitura
        .Datain(32'b0),           // Não escreve dados
        .Dataout(instruction)
    );

    // Banco de Registradores
    banco_reg reg_bank (
        .Clk(clk),
        .Reset(reset),
        .RegWrite(reg_write),
        .ReadReg1(instruction[25:21]), // rs
        .ReadReg2(instruction[20:16]), // rt
        .WriteReg(write_reg),
        .WriteData(write_data),
        .ReadData1(reg_data1),
        .ReadData2(reg_data2)
    );

    // Extensor de Sinal (Imediato)
    SingExtend_16x32 extensor16x32 (
        .in1(instruction[15:0]),
        .out(immediate_ext)
    );

    // Multiplexador ALUSrc
    mux_ALUsrc mux_alu_src (
        .in1(reg_data2),
        .in2(immediate_ext),
        .sel(alu_src),
        .out(alu_src2)
    );

    // Unidade Lógica e Aritmética (ALU)
    ula32 alu (
        .A(reg_data1),
        .B(alu_src2),
        .Seletor(alu_control),
        .S(alu_result),
        .Overflow(overflow),
        .Negativo(negativo),
        .z(zero),
        .Igual(igual),
        .Maior(maior),
        .Menor(menor)
    );

    // Memória de Dados
    memoria mem_data (
        .Address(alu_result),
        .Clock(clk),
        .Wr(mem_write),
        .Datain(reg_data2),
        .Dataout(mem_data_out)
    );

    // Multiplexador MemtoReg
    mux_MemtoReg mux_mem_to_reg (
        .in1(alu_result),
        .in2(mem_data_out),
        .sel(mem_to_reg),
        .out(write_data)
    );

    // Multiplexador RegDst
     mux_RegDst mux_reg_dst (
        .in1(instruction[20:16]), // rt (5 bits)
        .in2(instruction[15:11]), // rd (5 bits)
        .sel(reg_dst),            // Sinal de seleção (1 bit)
        .out(write_reg)           // Saída (5 bits)
    );

    // Cálculo do endereço de branch
    assign branch_target = pc_out + 4 + (immediate_ext << 2);

    // Multiplexador Branch
    mux_PC mux_branch (
        .in1(pc_out + 4),
        .in2(branch_target),
        .sel(branch & zero),
        .out(next_pc)
    );

    // Cálculo do endereço de jump
    assign jump_target = {pc_out[31:28], instruction[25:0] << 2};

    // Multiplexador Jump
    mux_PC mux_jump (
        .in1(next_pc),
        .in2(jump_target),
        .sel(jump),
        .out(next_pc)
    );

    // Unidade de Controle
    control_unit control (
    .clk(clk),
    .reset(reset),
    .opcode(instruction[31:26]),
    .funct(instruction[5:0]),
    .Overflow(overflow),
    .RegWrite(reg_write),
    .MemWrite(mem_write),
    .MemRead(mem_read),
    .MemtoReg(mem_to_reg),
    .ALUSrc(alu_src),
    .RegDst(reg_dst),
    .Branch(branch),
    .Jump(jump),
    .ALUOp(alu_control),
    .state(state) 
);

    // Registradores A, B, EPC, HI, LO
    registrador reg_A (
        .clk(clk),
        .reset(reset),
        .load(1'b1),              // Sempre carrega o próximo valor
        .entrada(reg_data1),
        .saida(reg_data1)
    );

    registrador reg_B (
        .clk(clk),
        .reset(reset),
        .load(1'b1),              // Sempre carrega o próximo valor
        .entrada(reg_data2),
        .saida(reg_data2)
    );

    registrador reg_EPC (
        .clk(clk),
        .reset(reset),
        .load(1'b1),              // Sempre carrega o próximo valor
        .entrada(alu_result),
        .saida(alu_result)
    );

    registrador reg_HI (
        .clk(clk),
        .reset(reset),
        .load(1'b1),              // Sempre carrega o próximo valor
        .entrada(alu_result),
        .saida(alu_result)
    );

    registrador reg_LO (
        .clk(clk),
        .reset(reset),
        .load(1'b1),              // Sempre carrega o próximo valor
        .entrada(alu_result),
        .saida(alu_result)
    );

endmodule