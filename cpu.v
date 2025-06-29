module cpu (
    input wire clk,
    input wire reset,

    output wire [31:0] pc_out,
    output wire [31:0] alu_result,
    output wire [31:0] hi_out,
    output wire [31:0] lo_out
);

    // --- Constantes de Opcode ---
    localparam OP_BEQ = 6'b000100;
    localparam OP_BNE = 6'b000101;
    localparam OP_J   = 6'b000010;
    localparam OP_JAL = 6'b000011;

    // --- Sinais de Controle da CPU ---
    wire PCWrite;
    wire PCWriteCond;
    wire PCWriteCondNeg;
    wire IorD;
    wire MemRead;
    wire MemWrite;
    wire IRWrite;
    wire RegWrite;
    wire [1:0] RegDst;
    wire ALUSrcA;
    wire [1:0] ALUSrcB;
    wire [1:0] PCSource;
    wire [3:0] ALUOp;
    wire HIWrite;
    wire LOWrite;
    wire MultStart;
    wire DivStart;
    wire [2:0] WBDataSrc;
    wire MemDataInSrc;
    wire PCClear;
    wire RegsClear;
    wire TempRegWrite;
    wire MemtoRegA;

    // --- Sinais de Dados do Datapath ---
    reg  [31:0] pc;
    wire [31:0] next_pc;
    wire [31:0] instruction_memory_data;
    wire [31:0] data_memory_read;
    wire [31:0] reg_a_out;
    wire [31:0] reg_b_out;
    wire [31:0] alu_in_a;
    wire [31:0] alu_in_b;
    wire [31:0] alu_out;
    wire [31:0] wb_data;
    wire [31:0] mdr_out;
    wire [31:0] mem_data_out;

    // --- Sinais da Instrução Decodificada ---
    wire [31:0] ir;
    wire [5:0]  opcode;
    wire [5:0]  funct;
    wire [4:0]  rs, rt, rd;
    wire [15:0] immediate;
    wire [31:0] sign_extend_out;
    
    // --- Sinais para o Banco de Registradores ---
    wire [4:0]  write_reg_addr;
    wire        final_RegWrite;

    // --- Sinais de Multiplicação e Divisão ---
    wire mult_done;
    wire div_done;
    wire div_by_zero_flag;
    wire [63:0] mult_result;
    wire [31:0] div_quotient;
    wire [31:0] div_remainder;

    // --- Sinais de Flags e Desvios ---
    wire zero;
    wire overflow;
    wire ula_negativo, ula_igual, ula_maior, ula_menor;
    wire branch_taken;
    wire [31:0] jump_address;
    wire [31:0] branch_address;
    wire [31:0] constant_4 = 32'd4;

    // =================================================================
    // Módulo de Controle
    // =================================================================
    control_unit u_control (
        .clk(clk),
        .reset(reset),
        .opcode(opcode),
        .funct(funct),
        .mult_done_in(mult_done),
        .div_done_in(div_done),
        .PCWrite(PCWrite),
        .PCWriteCond(PCWriteCond),
        .PCWriteCondNeg(PCWriteCondNeg),
        .IorD(IorD),
        .MemRead(MemRead),
        .MemWrite(MemWrite),
        .IRWrite(IRWrite),
        .RegWrite(RegWrite),
        .RegDst(RegDst),
        .ALUSrcA(ALUSrcA),
        .ALUSrcB(ALUSrcB),
        .PCSource(PCSource),
        .ALUOp(ALUOp),
        .HIWrite(HIWrite),
        .LOWrite(LOWrite),
        .MultStart(MultStart),
        .DivStart(DivStart),
        .WBDataSrc(WBDataSrc),
        .MemDataInSrc(MemDataInSrc),
        .PCClear(PCClear),
        .RegsClear(RegsClear),
        .TempRegWrite(TempRegWrite),
        .MemtoRegA(MemtoRegA)
    );

    // =================================================================
    // Datapath
    // =================================================================

    // --- Lógica do Program Counter (PC) ---
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pc <= 32'h00000000;
        end else if (PCWrite) begin
            pc <= next_pc;
        end else if (PCWriteCond && branch_taken) begin
            pc <= branch_address;
        end
    end
    
    assign pc_out = pc;
    assign branch_address = pc + (sign_extend_out << 2);
    assign jump_address = {pc[31:28], ir[25:0], 2'b00};
    assign next_pc = (PCSource == 2'b10) ? jump_address :
                     (PCSource == 2'b01) ? reg_a_out  :
                                           alu_out;

    // --- Decodificação da Instrução ---
    assign ir = instruction_memory_data;
    assign opcode = ir[31:26];
    assign funct = ir[5:0];
    assign rs = ir[25:21];
    assign rt = ir[20:16];
    assign rd = ir[15:11];
    assign immediate = ir[15:0];

    // --- Extensor de Sinal ---
    SingExtend_16x32 u_sign_extend (.in1(immediate), .out(sign_extend_out));

    // --- Lógica e Instanciação do Banco de Registradores ---
    // CORRIGIDO: Instanciando a entidade VHDL 'Banco_reg'
    
    // MUX para selecionar o registrador de destino (lógica RegDst)
    assign write_reg_addr = (RegDst[1] == 1'b0) ? rt : rd;
    
    // Lógica para impedir escrita no registrador $zero
    assign final_RegWrite = RegWrite && (write_reg_addr != 5'd0);

    Banco_reg u_register_file (
        .Clk(clk),
        .Reset(reset),
        .RegWrite(final_RegWrite),
        .ReadReg1(rs),
        .ReadReg2(rt),
        .WriteReg(write_reg_addr),
        .WriteData(wb_data),
        .ReadData1(reg_a_out),
        .ReadData2(reg_b_out)
    );

    // --- MUXes de entrada da ALU ---
    assign alu_in_a = (ALUSrcA == 1'b0) ? pc : reg_a_out;

    mux_ALUsrc u_mux_alusrc (
        .reg_b_data(reg_b_out),
        .constant_4(constant_4),
        .sign_ext_imm(sign_extend_out),
        .shifted_imm({immediate, 16'h0000}),
        .sel(ALUSrcB),
        .out(alu_in_b)
    );

    // --- Unidade Lógica e Aritmética (ULA) ---
    Ula32 u_alu (
        .A(alu_in_a),
        .B(alu_in_b),
        .Seletor(ALUOp[2:0]),
        .S(alu_out),
        .Overflow(overflow),
        .Negativo(ula_negativo),
        .z(zero),
        .Igual(ula_igual),
        .Maior(ula_maior),
        .Menor(ula_menor)
    );
    assign alu_result = alu_out;
    assign branch_taken = (opcode == OP_BEQ && zero) || (opcode == OP_BNE && !zero);

    // --- Memória Principal (Dados e Instruções) ---
    Memoria u_memory (
        .Address((IorD) ? alu_out : pc),
        .Clock(clk),
        .Wr(MemWrite),
        .Datain(reg_b_out),
        .Dataout(mem_data_out)
    );

    // Lógica para rotear a saída da memória
    assign instruction_memory_data = (IorD == 1'b0) ? mem_data_out : 32'b0;
    assign data_memory_read        = (IorD == 1'b1) ? mem_data_out : 32'b0;
    assign mdr_out = data_memory_read;

    // --- MUX de Write-Back para o Banco de Registradores ---
    assign wb_data = (WBDataSrc == 3'b001) ? mdr_out :
                     (WBDataSrc == 3'b010) ? hi_out :
                     (WBDataSrc == 3'b011) ? lo_out :
                     alu_out;

    // --- Unidades de Multiplicação e Divisão ---
    multiplier u_multiplier (
        .a(reg_a_out),
        .b(reg_b_out),
        .start(MultStart),
        .clk(clk),
        .reset(reset),
        .done(mult_done),
        .result(mult_result)
    );

    divider u_divider (
        .a(reg_a_out),
        .b(reg_b_out),
        .start(DivStart),
        .clk(clk),
        .reset(reset),
        .quotient(div_quotient),
        .remainder(div_remainder),
        .done(div_done),
        .div_by_zero(div_by_zero_flag)
    );
    
    // --- Registradores HI e LO ---
    hi_lo_registers u_hi_lo (
        .clk(clk),
        .reset(reset),
        .hi_in(HIWrite ? (MultStart ? mult_result[63:32] : div_remainder) : hi_out),
        .lo_in(LOWrite ? (MultStart ? mult_result[31:0]  : div_quotient) : lo_out),
        .hi_write(HIWrite),
        .lo_write(LOWrite),
        .hi_out(hi_out),
        .lo_out(lo_out)
    );

endmodule