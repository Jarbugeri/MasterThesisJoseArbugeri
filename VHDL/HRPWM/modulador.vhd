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
-- 01/03/2021    3.0     José     Adicionado PWM de alta resolução
------------------------------------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;
USE ieee.math_real.ALL;

ENTITY modulador IS
    PORT (
        clk0, clk45, clk90, clk135, clk180, clk225, clk270, clk315 : IN STD_LOGIC;
        rst : IN STD_LOGIC;
        moduladora : IN INTEGER;
		  il : IN STD_LOGIC_VECTOR (11 DOWNTO 0);
		  vout : IN STD_LOGIC_VECTOR (11 DOWNTO 0);
		  vin : IN STD_LOGIC_VECTOR (11 DOWNTO 0);
        trigger : OUT STD_LOGIC;
        Sa : OUT STD_LOGIC;
        Sb : OUT STD_LOGIC
    );
END modulador;

ARCHITECTURE rtl OF modulador IS

    COMPONENT hrpwm IS
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
    END COMPONENT hrpwm;

    -- Constantes
	CONSTANT Vp : INTEGER := 2 ** 6 - 1; -- 63

    -- Signals
    SIGNAL Vin_mV, Iin_mA : INTEGER := 0;
    SIGNAL pwmP, pwmN, Ep, En : STD_LOGIC := '0';

BEGIN

    pwmA : hrpwm
    GENERIC MAP(
        VP => Vp, -- 2*63
        OFFSET => 0,
        N => 3
    )
    PORT MAP(
        clk0 => clk0,
        clk45 => clk45,
        clk90 => clk90,
        clk135 => clk135,
        clk180 => clk180,
        clk225 => clk225,
        clk270 => clk270,
        clk315 => clk315,
        rst => rst,
        duty_cycle => moduladora,  -- moduladora
        trigger => trigger,
        pwm => OPEN,
        hrpwm => pwmP
    );

    pwmB : hrpwm
    GENERIC MAP(
        VP => Vp,
        OFFSET => - Vp, ---- 2*63
        N => 3
    )
    PORT MAP(
        clk0 => clk0,
        clk45 => clk45,
        clk90 => clk90,
        clk135 => clk135,
        clk180 => clk180,
        clk225 => clk225,
        clk270 => clk270,
        clk315 => clk315,
        rst => rst,
        duty_cycle => moduladora, --Moduladora
        trigger => OPEN,
        pwm => OPEN,
        hrpwm => pwmN
    );
		
	 Vin_mV <= (TO_INTEGER(UNSIGNED('0' & vin)) - 2 ** (12 - 1)) * 219;	
		
    modulador1 : PROCESS (clk0, rst)
    BEGIN
        IF rst = '1' THEN

            Sa <= '0';
            Sb <= '0';

        ELSIF rising_edge(clk0) THEN
		  
            -- Comparador Enable P e N (histerese passagem por 0)
            IF Vin_mV >= 0 THEN
                Ep <= '1';
            ELSE
                Ep <= '0';
            END IF;

			
            -- Auto controle
			    --Sa <= NOT(pwmP) AND     Ep;
			    --Sb <= (pwmN)    AND NOT(Ep);

			   -- Valores medios
                Sa <=  pwmP AND     Ep  ;
                Sb <=  pwmP AND NOT(Ep) ;

        END IF;

    END PROCESS modulador1;

END ARCHITECTURE;