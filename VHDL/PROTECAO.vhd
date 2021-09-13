LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE ieee.numeric_std.ALL;

ENTITY PROTECAO IS
	PORT (
		CLK : IN STD_LOGIC;
		RESET : IN STD_LOGIC;
		PWMAin : IN STD_LOGIC;
		PWMBin : IN STD_LOGIC;
		OverC : IN STD_LOGIC;
		CORRENTE : IN STD_LOGIC_VECTOR (11 DOWNTO 0);
		TENSAO : IN STD_LOGIC_VECTOR (11 DOWNTO 0);
		CORRENTE_MAX : OUT INTEGER;
		OVERTENSION : OUT STD_LOGIC;
		OverC_HARDWARE : OUT STD_LOGIC;
		OverC_SOFTWARE : OUT STD_LOGIC;
		ENABLEP : OUT STD_LOGIC;
		ENABLEN : OUT STD_LOGIC;
		PWMPout : OUT STD_LOGIC;
		PWMNout : OUT STD_LOGIC);
END PROTECAO;

ARCHITECTURE ARCH OF PROTECAO IS

	--REGISTER
	SIGNAL FLAG_CORRENTE : STD_LOGIC := '0';
	SIGNAL FLAG_TENSAO : STD_LOGIC := '0';
	SIGNAL FLAG_OVERC : STD_LOGIC := '0';
	CONSTANT LIMITE_CORRENTE_P : INTEGER := 15000; --vALOR EM mA 
	CONSTANT LIMITE_CORRENTE_N : INTEGER := - 15000; --vALOR EM mA 
	CONSTANT LIMITE_TENSAO : INTEGER := 200000; --vALOR EM mV 
	SIGNAL CORRENTE_VALUE : INTEGER := 0;
	SIGNAL Iin_mA, Vout_mV : INTEGER;
BEGIN

	OVERTENSION <= FLAG_TENSAO;
	OverC_HARDWARE <= FLAG_OVERC;
	OverC_SOFTWARE <= FLAG_CORRENTE;
	CORRENTE_MAX <= CORRENTE_VALUE;

	Vout_mV <= (TO_INTEGER(UNSIGNED('0' & TENSAO)) - 2 ** (12 - 1)) * 145;

	corrente_mA : PROCESS (CORRENTE)
	BEGIN
		IF SIGNED('0' & CORRENTE) >= TO_SIGNED(1902, 13) THEN
			Iin_mA <= (TO_INTEGER(UNSIGNED('0' & CORRENTE)) - 1902) * (-13); -- 11c
		ELSE
			Iin_mA <= (TO_INTEGER(UNSIGNED('0' & CORRENTE)) - 1902) * (-13); --18 , 13c , 11
		END IF;
	END PROCESS corrente_mA;

	PROCESS (CLK, RESET) BEGIN
		IF (RESET = '1') THEN
			FLAG_CORRENTE <= '0';
			FLAG_TENSAO   <= '0';
			FLAG_OVERC    <= '0';
			PWMPout <= '0';
			PWMNout <= '0';
			ENABLEP <= '0';
			ENABLEN <= '0';
			CORRENTE_VALUE <= 0;
		ELSIF RISING_EDGE(CLK) THEN

			IF Iin_mA > LIMITE_CORRENTE_P THEN
				FLAG_CORRENTE <= '1';
				CORRENTE_VALUE <= Iin_mA;
			END IF;

			IF Iin_mA < LIMITE_CORRENTE_N THEN
				FLAG_CORRENTE <= '1';
				CORRENTE_VALUE <= Iin_mA;
			END IF;

			IF Vout_mV >= LIMITE_TENSAO THEN
				FLAG_TENSAO <= '1';
			END IF;

			IF OverC = '0' THEN -- OverC == 1, Iin < 20 A , OverC == 0, Iin > 20 A
				FLAG_OVERC <= '1';
			END IF;

			IF FLAG_CORRENTE = '1' OR FLAG_TENSAO = '1' THEN -- OR FLAG_OVERC = '1' Removi pq ta com ruido
				PWMPout <= '0';
				PWMNout <= '0';
				ENABLEP <= '0';
				ENABLEN <= '0';
			ELSE
				PWMPout <= PWMAin;
				PWMNout <= PWMBin;
				ENABLEP <= '1';
				ENABLEN <= '1';
			END IF;
		END IF;
	END PROCESS;
END ARCH;