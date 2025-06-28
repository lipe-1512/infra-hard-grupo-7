--------------------------------------------------------------------------------
-- Title		: Unidade de L�gica e Aritm�tica
-- Project		: CPU multi-ciclo
--------------------------------------------------------------------------------
-- File			: ula32.vhd
-- Author		: Emannuel Gomes Mac�do (egm@cin.ufpe.br)
--				  Fernando Raposo Camara da Silva (frcs@cin.ufpe.br)
--				  Pedro Machado Manh�es de Castro (pmmc@cin.ufpe.br)
--				  Rodrigo Alves Costa (rac2@cin.ufpe.br)
-- Organization : Universidade Federal de Pernambuco
-- Created		: 29/07/2002
-- Last update	: 21/11/2002
-- Plataform	: Flex10K
-- Simulators	: Altera Max+plus II
-- Synthesizers	: 
-- Targets		: 
-- Dependency	: 
--------------------------------------------------------------------------------
-- Description	: Entidade que processa as opera��es l�gicas e aritm�ticas da
-- cpu.
--------------------------------------------------------------------------------
-- Copyright (c) notice
--		Universidade Federal de Pernambuco (UFPE).
--		CIn - Centro de Informatica.
--		Developed by computer science undergraduate students.
--		This code may be used for educational and non-educational purposes as 
--		long as its copyright notice remains unchanged. 
--------------------------------------------------------------------------------
-- Revisions		: 1
-- Revision Number	: 1
-- Version			: 1.1
-- Date				: 21/11/2002
-- Modifier			: Marcus Vinicius Lima e Machado (mvlm@cin.ufpe.br)
--				   	  Paulo Roberto Santana Oliveira Filho (prsof@cin.ufpe.br)
--					  Viviane Cristina Oliveira Aureliano (vcoa@cin.ufpe.br)
-- Description		:
--------------------------------------------------------------------------------
--------------------------------------------------------------------------------
-- Revisions		: 2
-- Revision Number	: 1.1
-- Version			: 1.2
-- Date				: 18/08/2008
-- Modifier			: Jo�o Paulo Fernandes Barbosa (jpfb@cin.ufpe.br)
-- Description		: Entradas, sa�das e sinais internos passam a ser std_logic.
--------------------------------------------------------------------------------
--------------------------------------------------------------------------------
-- Revisions		: 3
-- Revision Number	: 1.2
-- Version			: 1.3
-- Date				: 01/02/2021
-- Modifier			: André Soares da Silva Filho <assf@cin.ufpe.br>
-- Description		: A biblioteca passa ser a NUMERIC_STD para evitar conflitos no ModelSim 20.1.1.
--------------------------------------------------------------------------------
--------------------------------------------------------------------------------
-- Revisions		: 4
-- Revision Number	: 1.3
-- Version			: 1.4
-- Date				: 23/02/2024
-- Modifier			: João Victor da Silva <jvs2@cin.ufpe.br>
-- Description		: Sinal de overflow só pode ser ativado nas operações de Soma, Substração e Incremento.
--------------------------------------------------------------------------------


-- ula32.vhd
-- Arquitetura com a operação de comparação (slt) corrigida.

LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;

entity Ula32 is
	port ( 
		A 			: in  std_logic_vector (31 downto 0);
		B 			: in  std_logic_vector (31 downto 0);
		Seletor 	: in  std_logic_vector (2 downto 0);
		S 			: out std_logic_vector (31 downto 0);
		Overflow 	: out std_logic;
		Negativo	: out std_logic;
		z 			: out std_logic;
		Igual		: out std_logic;
		Maior		: out std_logic;
		Menor		: out std_logic 
	);
end Ula32;

architecture behavioral of Ula32 is
	
	signal s_temp		: std_logic_vector (31 downto 0);
 	signal soma_temp 	: std_logic_vector (31 downto 0);
	signal carry_temp	: std_logic_vector (31 downto 0);
	signal novo_B 		: std_logic_vector (31 downto 0);
	signal i_temp		: std_logic_vector (31 downto 0);
	signal igual_temp	: std_logic;
	signal overflow_temp: std_logic;
    signal menor_temp   : std_logic; -- Adicionado para slt
    signal s_slt_res    : std_logic_vector (31 downto 0); -- Adicionado para slt

	begin

		i_temp <= (0 => '1', OTHERS => '0'); -- Constante 1
		
		-- Regiao que calcula a soma, subtracao e incremento					
		with Seletor select
			novo_B <= B  		when "001",  -- Soma
   				      i_temp 	when "100",  -- Incremento
                      not(B) 	when others; -- Subtracao e outros	
  	
    	soma_temp(0) <= A(0) xor novo_B(0) xor seletor(1);
		-- ... (código do somador completo permanece o mesmo) ...
		soma_temp(31) <= A(31) xor novo_B(31) xor carry_temp(30);

		carry_temp(0) <= (seletor(1) and (A(0) or novo_B(0))) or (A(0) and novo_B(0));
		-- ... (código do carry completo permanece o mesmo) ...
		carry_temp(31) <= (carry_temp(30) and (A(31) or novo_B(31))) or (A(31) and novo_B(31));

		overflow_temp <= carry_temp(31) xor carry_temp(30);

		with Seletor select
			Overflow <= overflow_temp when "001",
						overflow_temp when "010",
						overflow_temp when "100",
						'0' when others;

		-- Regiao que calcula a comparação (slt, beq)
		igual_temp <= '1' when soma_temp = X"00000000" else '0';
  		Igual <= igual_temp when Seletor = "010" else '0';
        
        -- ** CORREÇÃO DE LÓGICA AQUI **
        -- A flag 'Menor' é baseada no resultado da subtração (Seletor="010")
		menor_temp <= '1' when (signed(A) < signed(B)) else '0';
		Menor <= menor_temp;
        
		Maior <= '1' when (not menor_temp and not (igual_temp and Seletor = "010")) else '0';
        
        -- Saída principal S
        s_slt_res <= (0 => menor_temp, OTHERS => '0'); -- Resultado para slt é 1 ou 0
			
		with Seletor select
			s_temp <= 	A  			when "000",
			  			soma_temp  	when "001",
			  			soma_temp   when "010",
			  			(A and B)  	when "011",
			  			(A xor B) 	when "110",             
              			not(A)     	when "101",
			  			soma_temp  	when "100",
                        s_slt_res   when "111", -- ** CORREÇÃO DE LÓGICA AQUI **
             			X"00000000" when others;
			
		S <= s_temp;
		z <= '1' when s_temp = X"00000000" else '0';
		Negativo <= s_temp(31);

end behavioral;