LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE ieee.numeric_std.ALL;

ENTITY hrpwm IS
	GENERIC (
		VP : INTEGER;
		OFFSET : INTEGER;
		N : INTEGER);
	PORT (
		clk0, clk45, clk90, clk135, clk180, clk225, clk270, clk315 : IN STD_LOGIC;
		rst : IN STD_LOGIC;
		duty_cycle : IN INTEGER;
		trigger : OUT STD_LOGIC;
		pwm : OUT STD_LOGIC;
		hrpwm : OUT STD_LOGIC);
END hrpwm;

ARCHITECTURE rtl OF hrpwm IS

	COMPONENT SRLATCH IS
		PORT (
			S, R : IN STD_LOGIC;
			Q, QN : OUT STD_LOGIC
		);
	END COMPONENT SRLATCH;

	-- Sinais
	SIGNAL counter : INTEGER := 0;
	SIGNAL shadow : INTEGER := 0;
	SIGNAL dir : STD_LOGIC := '0';
	SIGNAL ADC_clk : STD_LOGIC := '0';

	SIGNAL SEL : STD_LOGIC_VECTOR(0 TO 2) := (OTHERS => '0');
	SIGNAL duty : STD_LOGIC_VECTOR(0 TO 9 - 3) := (OTHERS => '0');

	SIGNAL CLRD, SETD : STD_LOGIC := '0';
	SIGNAL S, R : STD_LOGIC := '0';
	SIGNAL CLR0, CLR45, CLR90, CLR135, CLR180, CLR225, CLR270, CLR315, CLR360 : STD_LOGIC := '0';
	SIGNAL SET0, SET45, SET90, SET135, SET180, SET225, SET270, SET315 : STD_LOGIC := '0';

BEGIN

	srlatch_pwm : SRLATCH
	PORT MAP(
		S => S,
		R => R,
		Q => hrpwm,
		QN => OPEN
	);

	trigger_process : PROCESS (clk0, rst)
	BEGIN
		IF rst = '1' THEN
			ADC_clk <= '0';
		ELSIF rising_edge(clk0) THEN
			IF counter = 0 OR counter = VP OR counter = VP/2 THEN
				ADC_clk <= NOT ADC_clk; --TOOGLE
			END IF;
		END IF;
	END PROCESS trigger_process;
	
	trigger <= ADC_clk;

	main : PROCESS (clk0, rst)
	BEGIN
		IF rst = '1' THEN
			counter <= 0;
			shadow <= 0;
			dir <= '0';
			SEL <= (OTHERS => '0');
		ELSIF rising_edge(clk0) THEN

			-- Atualiza registrador Shadow		
			IF counter = 0 OR counter = VP THEN -- Zero e pico da portadora
				shadow <= to_integer(to_signed(duty_cycle, 32)(9 DOWNTO 3));
				SEL <= STD_LOGIC_VECTOR(to_signed(duty_cycle, 32)(2 DOWNTO 0));
			END IF;

			--Contador da portadora 
			IF dir = '0' THEN
				counter <= counter + 1;
			ELSE
				counter <= counter - 1;
			END IF;

			--Define quando troca o sentido de contagem
			IF (counter = (vp - 1)) THEN
				dir <= '1';
			END IF;
			IF (counter = (0 + 1)) THEN
				dir <= '0';
			END IF;

			--Comparador para pwm de baixa resolução
			IF shadow > (counter + offset) THEN
				pwm <= '1';
			ELSE
				pwm <= '0';
			END IF;
		END IF;
	END PROCESS;

	-- Processos para pwm de alta resolução

	PROCESS (shadow, counter, dir)
	BEGIN
		-- Caso estiver incrementando e duty = counter
		IF (shadow = (counter + offset) AND dir = '1') THEN
			SETD <= '1';
		ELSE
			SETD <= '0';
		END IF;
		-- Caso estiver decrementando e duty = counter
		IF (shadow = (counter + offset) AND dir = '0') THEN
			CLRD <= '1';
		ELSE
			CLRD <= '0';
		END IF;
	END PROCESS;

	PROCESS (clk0)
	BEGIN
		IF rising_edge(clk0) THEN
			SET0 <= SETD;
			CLR0 <= CLRD;
			CLR360 <= CLR0;
		END IF;
	END PROCESS;

	PROCESS (clk45)
	BEGIN
		IF rising_edge(clk45) THEN
			CLR45 <= CLR0;
			SET45 <= SET0;
		END IF;
	END PROCESS;

	PROCESS (clk90)
	BEGIN
		IF rising_edge(clk90) THEN
			CLR90 <= CLR0;
			SET90 <= SET0;
		END IF;
	END PROCESS;

	PROCESS (clK135)
	BEGIN
		IF rising_edge(clk135) THEN
			CLR135 <= CLR0;
			SET135 <= SET0;
		END IF;
	END PROCESS;

	PROCESS (clK180)
	BEGIN
		IF rising_edge(clk180) THEN
			CLR180 <= CLR0;
			SET180 <= SET0;
		END IF;
	END PROCESS;

	PROCESS (clk225)
	BEGIN
		IF rising_edge(clk225) THEN
			CLR225 <= CLR0;
			SET225 <= SET0;
		END IF;
	END PROCESS;

	PROCESS (clk270)
	BEGIN
		IF rising_edge(clk270) THEN
			CLR270 <= CLR0;
			SET270 <= SET0;
		END IF;
	END PROCESS;

	PROCESS (clk315)
	BEGIN
		IF rising_edge(clk315) THEN
			CLR315 <= CLR0;
			SET315 <= SET0;
		END IF;
	END PROCESS;

	WITH SEL SELECT
		R <= CLR45 WHEN "111",
		CLR90  WHEN "110",
		CLR135 WHEN "101",
		CLR180 WHEN "100",
		CLR225 WHEN "011",
		CLR270 WHEN "010",
		CLR315 WHEN "001",
		CLR360 WHEN "000",
		CLR360 WHEN OTHERS;

	WITH SEL SELECT
		S <= SET0 WHEN "000",
		SET45  WHEN "001",
		SET90  WHEN "010",
		SET135 WHEN "011",
		SET180 WHEN "100",
		SET225 WHEN "101",
		SET270 WHEN "110",
		SET315 WHEN "111",
		SET0 WHEN OTHERS;

END rtl;