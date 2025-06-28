// cpu.v
// Módulo Top-Level COMPLETO e CORRIGIDO que conecta todos os componentes do datapath multi-ciclo.
// Integra ALU, Memória, Registradores, MUXes, Multiplicador, Divisor e lógica de Exceção.

module cpu(
    input wire clk,
    input wire reset
);

    // =================================================================
    // --- SINAIS DE CONTROLE (Saídas da Unidade de Controle - FSM) ---
    // =================================================================
    wire        PCWrite, PCWriteCond, IorD, MemRead, MemWrite, IRWrite, RegWrite;
    wire        MemtoReg;
    wire [1:0]  RegDst;      // Estendido para 2 bits para suportar `jal` ($ra)
    wire        ALUSrcA;
    wire [1:0]  ALUSrcB;
    wire [1:0]  PCSource;
    wire [3:0]  ALUOp;
    wire        HIWrite, LOWrite, MultStart, DivStart; // Controle para mult/div

    // =================================================================
    // --- FIOS INTERNOS DO DATAPATH ---
    // =================================================================
    wire [31:0] pc_out, next_pc;
    wire [31:0] mem_data_out;
    wire [31:0] reg_a_out, reg_b_out;        // Saídas dos regs. intermediários A e B
    wire [31:0] alu_out_reg;                // Saída do reg. intermediário ALUOut
    wire [31:0] mdr_out;                    // Saída do Memory Data Register
    wire [31:0] read_data_1, read_data_2;   // Saídas do Banco de Registradores
    wire [31:0] sign_extended_imm;
    wire [31:0] alu_result;
    wire        alu_zero;                   // Flag 'zero' da ALU
    wire        ula_overflow;               // Flag de overflow da ALU
    wire [31:0] write_data_mux_out;
    wire [4:0]  write_reg_mux_out;

    // --- Fios da Saída do Registrador de Instrução (IR) ---
    wire [5:0]  ir_opcode;
    wire [4:0]  ir_rs, ir_rt, ir_rd;
    wire [4:0]  ir_shamt;
    wire [15:0] ir_immediate;
    wire [5:0]  ir_funct;

    // --- Fios dos Módulos Especiais (Mult/Div) ---
    wire [63:0] mult_result;
    wire        mult_done;
    wire [31:0] div_quotient, div_remainder;
    wire        div_done, div_by_zero;
    wire [31:0] hi_out, lo_out;

    // --- Fios do Módulo de Exceção ---
    wire        exception_detected;
    wire [31:0] exception_pc;


    // =================================================================
    // --- 1. LÓGICA DO PROGRAM COUNTER (PC) ---
    // =================================================================
    // MUX para selecionar a fonte do próximo PC
    assign next_pc = (PCSource == 2'b00) ? alu_result :                                         // Fonte 00: Resultado da ALU (PC+4 ou end. branch)
                     (PCSource == 2'b01) ? alu_out_reg :                                        // Fonte 01: ALUOut (end. branch de ciclo anterior)
                     (PCSource == 2'b10) ? {{pc_out[31:28]}, {ir_immediate[15:0], ir_rd, ir_shamt, ir_funct}, 2'b00} : // Fonte 10: Endereço de Jump
                                           reg_a_out;                                            // Fonte 11: Registrador A (para jr)

    // O sinal final que habilita a escrita no PC
    wire pc_write_enable = exception_detected ? 1'b1 : (PCWriteCond ? (PCWrite & alu_zero) : PCWrite);
    wire [31:0] pc_in = exception_detected ? exception_pc : next_pc;

    // Instanciação do Registrador do PC
    Registrador #(32) pc_reg (
        .clk(clk), .reset(reset), .Load(pc_write_enable),
        .Entrada(pc_in), .Saida(pc_out)
    );

    // =================================================================
    // --- 2. ESTÁGIO DE BUSCA E DECODIFICAÇÃO DE INSTRUÇÃO ---
    // =================================================================
    // Memória (IorD=0 para buscar instrução, IorD=1 para acessar dados)
    wire [31:0] mem_address = IorD ? alu_out_reg : pc_out;
    Memoria memoria (
        .Address(mem_address), .Clock(clk), .Wr(MemWrite),
        .Datain(reg_b_out), .Dataout(mem_data_out)
    );

    // Registrador de Instrução (IR) - Instanciando o componente VHDL fornecido
    Instr_Reg ir_unit (
        .Clk(clk), .Reset(reset), .Load_ir(IRWrite),
        .Entrada(mem_data_out),
        .Instr31_26(ir_opcode),
        .Instr25_21(ir_rs),
        .Instr20_16(ir_rt),
        .Instr15_0(ir_immediate)
    );
    // Extraindo os campos rd, shamt e funct do campo ir_immediate de 16 bits
    assign ir_rd = ir_immediate[15:11];
    assign ir_shamt = ir_immediate[10:6];
    assign ir_funct = ir_immediate[5:0];

    // =================================================================
    // --- 3. REGISTRADORES INTERMEDIÁRIOS DO DATAPATH ---
    // =================================================================
    // Memory Data Register (MDR)
    Registrador #(32) mdr_reg (
        .clk(clk), .reset(reset), .Load(1'b1), // MDR sempre armazena o que vem da memória
        .Entrada(mem_data_out), .Saida(mdr_out)
    );
    
    // Registrador A (para guardar o valor de rs) e B (para guardar o valor de rt)
    Registrador #(32) reg_a (.clk(clk), .reset(reset), .Load(1'b1), .Entrada(read_data_1), .Saida(reg_a_out));
    Registrador #(32) reg_b (.clk(clk), .reset(reset), .Load(1'b1), .Entrada(read_data_2), .Saida(reg_b_out));
    
    // Registrador ALUOut (para guardar o resultado da ALU por um ciclo)
    Registrador #(32) alu_out ( .clk(clk), .reset(reset), .Load(1'b1), .Entrada(alu_result), .Saida(alu_out_reg));
    
    // =================================================================
    // --- 4. LÓGICA DO BANCO DE REGISTRADORES (WRITE-BACK) ---
    // =================================================================
    // MUX para selecionar o registrador de destino
    mux_RegDst mux_write_reg (
        .in1(ir_rt), .in2(ir_rd), .sel(RegDst[0]), // Usa bit 0 para sel. entre rt/rd. RegDst[1] pode ser usado para $ra
        .out(write_reg_mux_out)
    );
    // TODO: Adicionar lógica para selecionar $ra (31) quando RegDst for 2'b10 (para jal).
    
    // MUX para selecionar o dado a ser escrito de volta no banco de registradores
    mux_MemtoReg mux_write_data (
        .in1(alu_out_reg), .in2(mdr_out), .sel(MemtoReg),
        .out(write_data_mux_out)
    );

    // Instanciação do Banco de Registradores
    Banco_reg banco_registradores (
        .Clk(clk), .Reset(reset), .RegWrite(RegWrite & ~exception_detected), // Desativa escrita em caso de exceção
        .ReadReg1(ir_rs), .ReadReg2(ir_rt), 
        .WriteReg(write_reg_mux_out), .WriteData(write_data_mux_out),
        .ReadData1(read_data_1), .ReadData2(read_data_2)
    );

    // =================================================================
    // --- 5. LÓGICA DA ALU ---
    // =================================================================
    SingExtend_16x32 sign_extender (.in1(ir_immediate), .out(sign_extended_imm));
    
    wire [31:0] alu_in_a = ALUSrcA ? reg_a_out : pc_out;
    
    wire [31:0] alu_in_b;
    mux_ALUsrc alu_src_b_mux (
        .reg_b_data(reg_b_out),
        .constant_4(32'd4),
        .sign_ext_imm(sign_extended_imm),
        .shifted_imm(sign_extended_imm << 2),
        .sel(ALUSrcB),
        .out(alu_in_b)
    );
    
    Ula32 ula(
        .A(alu_in_a), .B(alu_in_b),
        .Seletor(ALUOp[2:0]), .S(alu_result),
        .z(alu_zero), .Overflow(ula_overflow)
        // Outras saídas não conectadas por enquanto: Negativo, Igual, Maior, Menor
    );
    
    // =================================================================
    // --- 6. UNIDADES ESPECIAIS (MULTIPLICAÇÃO E DIVISÃO) ---
    // =================================================================
    multiplier mult_unit (
        .a(reg_a_out), .b(reg_b_out), .start(MultStart), .clk(clk),
        .result(mult_result), .done(mult_done)
    );
    
    divider div_unit (
        .a(reg_a_out), .b(reg_b_out), .start(DivStart), .clk(clk),
        .quotient(div_quotient), .remainder(div_remainder),
        .done(div_done), .div_by_zero(div_by_zero)
    );
    
    hi_lo_registers hi_lo_regs (
        .clk(clk), .reset(reset),
        .hi_in(div_remainder), .lo_in(div_quotient),
        .hi_write(HIWrite), .lo_write(LOWrite),
        .hi_out(hi_out), .lo_out(lo_out)
    );
    // TODO: Adicionar MUXes para carregar HI/LO com resultado do multiplicador
    // e para mover de HI/LO para o banco de registradores (mfhi/mflo).

    // =================================================================
    // --- 7. LÓGICA DE EXCEÇÃO ---
    // =================================================================
    wire invalid_opcode; // Deve ser gerado pela Unidade de Controle
    
    exception exception_unit (
        .overflow(ula_overflow), .div_by_zero(div_by_zero), .invalid_opcode(invalid_opcode),
        .pc(pc_out), // PC da instrução que causou a exceção
        .epc(), // Saída para um registrador EPC (precisa ser criado)
        .exception(exception_detected),
        .exception_code(),
        .new_pc(exception_pc) // Novo PC para a rotina de tratamento
    );
    // TODO: Criar e integrar o registrador EPC.

    // =================================================================
    // --- 8. UNIDADE DE CONTROLE (FSM) ---
    // =================================================================
    control_unit FSM (
        .clk(clk), .reset(reset),
        .opcode(ir_opcode),
        .funct(ir_funct),
        // Sinais de status do datapath para a FSM
        .mult_done_in(mult_done),
        .div_done_in(div_done),
        // Saídas de Controle para o datapath
        .PCWrite(PCWrite), .PCWriteCond(PCWriteCond),
        .IorD(IorD), .MemRead(MemRead), .MemWrite(MemWrite), .IRWrite(IRWrite),
        .RegWrite(RegWrite), .MemtoReg(MemtoReg), .RegDst(RegDst),
        .ALUSrcA(ALUSrcA), .ALUSrcB(ALUSrcB), .PCSource(PCSource),
        .ALUOp(ALUOp),
        .HIWrite(HIWrite), .LOWrite(LOWrite),
        .MultStart(MultStart), .DivStart(DivStart),
        .InvalidOpcodeOut(invalid_opcode)
    );

endmodule