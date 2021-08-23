-- Title      : Modulador PWM PFC dissertação 
-- Project    : Comit
----------------------------------------------------------------------------------------------------------
-- File       : pwm.vhd
-- Author     : José Augusto Arbugeri (josearbugeri@gmail.com)
-- Company    : Instituto de eletrônica de Potência, Universidade Federal de Santa Catarina 
-----------------------------------------------------------------------------------------------------------
-- Modulador pwm portadara UPDOWN, utilizado na dissertação do José Augusto Arbugeri
--            Trigger pode ser usado para disparar conversão do AD
--            DIRo: direção da contagem da portadora
--            offset : valor de deslocamento da portadora para amplitude disposition
--            vp : valor de pico da portadora
-----------------------------------------------------------------------------------------------------------
-- Copyright (c) Instituto de eletrônica de Potência, Universidade Federal de Santa Catarina 
-----------------------------------------------------------------------------------------------------------
-- Revisions  :
-- Date        Version  Author   Description
-- 2020          1.0     José     Created: pwm up down 
-- 11/02/2021    2.0     José     Modificado para portadora com offset setavel, para pode utilizar com
--                                valor de modulação negativo
------------------------------------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;
USE ieee.math_real.ALL;

ENTITY pwm IS
    PORT (
        clk : IN STD_LOGIC;
        rst : IN STD_LOGIC;
        il : IN STD_LOGIC_VECTOR(11 DOWNTO 0);
        vin : IN STD_LOGIC_VECTOR(11 DOWNTO 0);
		  trigger : OUT STD_LOGIC;
        Sa : OUT STD_LOGIC;
        Sb : OUT STD_LOGIC
    );
END pwm;

ARCHITECTURE rtl OF pwm IS

    COMPONENT pwmupdown IS
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
    END COMPONENT pwmupdown;

    -- Constantes
    CONSTANT Ki : real := 1.0*0.0849; -- dasfsdds
    CONSTANT Qb : real := 2.0 ** 16;

    -- Conversão
    SIGNAL Ki_Q16 : INTEGER := INTEGER(Ki * Qb);

    -- Signals
    SIGNAL Vin_mV, Iin_mA : INTEGER := 0;
    SIGNAL counter : INTEGER;
    SIGNAL moduladora : INTEGER;
    SIGNAL pwmP, pwmN, Ep, En : STD_LOGIC := '0';
	 SIGNAL trig : STD_LOGIC;

BEGIN

    pwmA : pwmupdown
    GENERIC MAP(
        VP => 63,
        OFFSET => 0
    )
    PORT MAP(
        clk => clk,
        rst => rst,
        duty => moduladora,
        pwm => pwmP,
        trigger => trig,
        diro => OPEN
    );

    pwmB: pwmupdown
    GENERIC MAP(
        VP => 63,
        OFFSET => -63 -- -63
    )
    PORT MAP(
        clk => clk,
        rst => rst,
        duty => moduladora,
        pwm => pwmN,
        trigger => OPEN,
        diro => OPEN
    );

    modulador : PROCESS (clk, rst)
    BEGIN
        IF rst = '1' THEN

            Sa <= '0';
            Sb <= '0';
            counter <= 0;
            moduladora <= 0;

        ELSIF rising_edge(clk) THEN

            Vin_mV <= (TO_INTEGER(UNSIGNED('0' & vin)) - 2 ** (12 - 1)) * 219;

            IF SIGNED('0' & il) >= TO_SIGNED(1902, 13) THEN
                Iin_mA <= (TO_INTEGER(UNSIGNED('0' & il)) - 1902) * (-11); -- 11c
            ELSE
                Iin_mA <= (TO_INTEGER(UNSIGNED('0' & il)) - 1902) * (-13); --18 , 13c , 11
            END IF;

            -- Controle: Self control
            moduladora <= to_integer(shift_right(to_signed(Iin_mA * Ki_Q16, 32), 16 + 4 - 0 ));
				
				-- Valor fixo para CC-CC
				--moduladora <= 32; -- 50% (32/64)
				
            -- Comparador Enable P e N
            IF Vin_mV >= 2000 THEN
                Ep <= '1';
                En <= '0';
            ELSIF Vin_mV <= -2000 THEN
                Ep <= '0';
                En <= '1';
            ELSE 
                Ep <= '0';
                En <= '0'; 
            END IF;

            -- Habilita sinal Sa e Sb para cada SemiCilco
				
            --Sa <= NOT(pwmP) AND Ep;
            Sa <= NOT(pwmP);
				-- Sa <= '0'; -- Teste para habilitar so semiciclo negativo
            
				--Sb <= pwmN AND En;
				Sb <= pwmN ;
				
				-- Para CC-CC
				--Sa <= pwmP AND E;
				--Sb <= pwmN AND NOT(E);

        END IF;

    END PROCESS modulador;
	 
	 trigger <= trig;

END ARCHITECTURE;