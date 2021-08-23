LIBRARY IEEE;
USE IEEE.std_logic_1164.ALL;
USE IEEE.numeric_std.ALL;	

ENTITY STARTKIT IS
	PORT (
		CLK 			: IN  STD_LOGIC;
		RESET     	: IN  STD_LOGIC;                  
		BT    		: IN  STD_LOGIC_VECTOR(1 DOWNTO 0);
		BT_PCB		: IN  STD_LOGIC_VECTOR(1 DOWNTO 0);
		LED 		   : OUT STD_LOGIC_VECTOR(3 DOWNTO 0);
		RGB 		   : OUT STD_LOGIC_VECTOR(2 DOWNTO 0); 
		BUZZER     	: OUT  STD_LOGIC      		
	);
END ENTITY;
  
ARCHITECTURE COMPORTAMENTO OF STARTKIT IS

	SIGNAL PORTADORA  :UNSIGNED (11 downto 0);
	SIGNAL PORTADORA_BZ  :UNSIGNED (13 downto 0);
	SIGNAL MODULADORA :UNSIGNED (11 downto 0);
	SIGNAL COUNTER 	:UNSIGNED (25 downto 0);
	SIGNAL LED_MASK 	:UNSIGNED (3 downto 0);
	SIGNAL PWM		 	:UNSIGNED (3 downto 0);
	SIGNAL PWM_B        : STD_LOGIC;
	
	
BEGIN

	PROCESS (CLK, RESET) BEGIN
		IF (RESET = '1') THEN
			COUNTER 		<= (others=>'0');
			LED_MASK		<= (others=>'0');
			PORTADORA	<= (others=>'0');
			RGB	<= (others=>'0');
		ELSIF (RISING_EDGE(CLK)) THEN
		
			IF(COUNTER = 0) THEN
				LED_MASK <= LED_MASK + 1;
			END IF;
			
			PORTADORA  	  <= PORTADORA + 1;
			PORTADORA_BZ  <= PORTADORA_BZ + 1;
			COUNTER    <= COUNTER + 1;
			MODULADORA <= COUNTER(25 DOWNTO 25-11);
			
			IF(BT_PCB(0) = '1') THEN
				BUZZER <= PWM_B;
			ELSE
				BUZZER <= '0';
			END IF;
			
			
			IF(MODULADORA >= PORTADORA) THEN
				PWM <= to_unsigned(15,4);
			ELSE
				PWM <= to_unsigned(0,4);
			END IF;
			
			IF(TO_UNSIGNED(10000,14) >= PORTADORA_BZ) THEN
				PWM_B <= '1';
			ELSE
				PWM_B <= '0';
			END IF;			
		END IF;
	END PROCESS;
	
	LED <= STD_LOGIC_VECTOR(PWM) AND STD_LOGIC_VECTOR(LED_MASK);

END ARCHITECTURE;