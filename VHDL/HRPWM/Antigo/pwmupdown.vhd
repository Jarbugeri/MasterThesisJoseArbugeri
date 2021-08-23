LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE ieee.numeric_std.ALL;

ENTITY pwmupdown IS
	GENERIC (
		VP : INTEGER;
		OFFSET : INTEGER
	);
	PORT (
		clk : IN STD_LOGIC;
		rst : IN STD_LOGIC;
		duty : IN INTEGER;
		pwm : OUT STD_LOGIC;
		trigger, diro : OUT STD_LOGIC);
END pwmupdown;

ARCHITECTURE ARCH OF pwmupdown IS

	-- Constantes

	-- Sinais
	SIGNAL counter : INTEGER := 0;
	SIGNAL shadow : INTEGER := 0;
	SIGNAL dir : STD_LOGIC := '0';

BEGIN

	PROCESS (clk, rst)
	BEGIN
		IF rst = '1' THEN

			counter <= 0;
			shadow <= 0;
			dir <= '0';

		ELSIF rising_edge(clk) THEN

			-- Atualiza registrador Shadow e seta trigger			
			IF  (counter = 0) OR (counter = VP) THEN --(counter = 0) OR
				shadow <= duty;
				trigger <= '1';
			ELSE
				trigger <= '0';
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

			--Comparador
			IF shadow >= (counter + offset) THEN
				pwm <= '1';
			ELSE
				pwm <= '0';
			END IF;
		END IF;
	END PROCESS;

	diro <= dir;

END ARCH;