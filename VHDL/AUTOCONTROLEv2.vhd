LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE ieee.numeric_std.ALL;
USE IEEE.MATH_REAL.ALL;

ENTITY AUTOCONTROLEv2 IS
    PORT (
        clk : IN STD_LOGIC;
        rst : IN STD_LOGIC;
        il : IN STD_LOGIC_VECTOR (11 DOWNTO 0);
        vout : IN STD_LOGIC_VECTOR (11 DOWNTO 0);
        vin : IN STD_LOGIC_VECTOR (11 DOWNTO 0);
        duty : OUT INTEGER
    );
END AUTOCONTROLEv2;

ARCHITECTURE ARCH OF AUTOCONTROLEv2 IS

    -- Sinais de divisão do clock     
    SIGNAL count : INTEGER := 0;
    CONSTANT DIV : INTEGER := 16;

    -- Auto Controle

    CONSTANT KiS : real := 0.08484298245614035087; -- Ki = Vp^2/(2*Po*Vo) = 311^2/(2*1500*380)
    CONSTANT Qr : real := 16.0;

    -- Conversão
    CONSTANT Q : INTEGER := 16;
    SIGNAL Ki_Q16 : INTEGER := INTEGER(KiS * (2.0 ** Qr));

    -- Contantes representação real
    CONSTANT KPr : real := 0.038418; -- 0.0074061
    CONSTANT KIrc : real := 1574.0; -- 222.3311
    CONSTANT KIr : real := KIrc * (REAL(DIV)/(750.0e3)); --1574.0

    CONSTANT Kvr : real := 0.030991735537190;
    CONSTANT KRr : real := SQRT(KIrc); -- SQRT(1574.0)

    -- Contantes na base Q
    CONSTANT KP : INTEGER := INTEGER(KPr * 2.0 ** Qr);
    CONSTANT KI : INTEGER := INTEGER(KIr * 2.0 ** Qr);
    CONSTANT Kv : INTEGER := INTEGER(Kvr * 2.0 ** Qr);
    CONSTANT Kr : INTEGER := INTEGER(KRr);

    SIGNAL Y, Y_sat, IREF, ERRO, INTEGRAL, PROPORCIONAL : INTEGER;
    SIGNAL Iin_mA, Vin_mV : INTEGER;

    --Anti windup
    CONSTANT maximo : INTEGER := INTEGER((2.0 ** Qr) * (510.0));
    CONSTANT minimo : INTEGER := INTEGER((2.0 ** Qr) * (0.0));
    SIGNAL e_sat, feed_0, feed_1 : INTEGER;

BEGIN

    Vin_mV <= (TO_INTEGER(UNSIGNED('0' & vin)) - 2 ** (12 - 1)) * 219;

    corrente : PROCESS (il)
    BEGIN
        IF SIGNED('0' & il) >= TO_SIGNED(1902, 13) THEN
            Iin_mA <= (TO_INTEGER(UNSIGNED('0' & il)) - 1902) * (-13); -- 11c
        ELSE
            Iin_mA <= (TO_INTEGER(UNSIGNED('0' & il)) - 1902) * (-13); --18 , 13c , 11
        END IF;
    END PROCESS corrente;

    -- Parte sem memoria do PI
    -- Referencia para malha de corrente
    IREF <= ABS((Vin_mV * Kv) / 2 ** 10);
    -- Erro da malha de corrente
    ERRO <= feed_0 + IREF - ABS((2 ** Q * Iin_mA) / 2 ** 10); -- Realimentação do windup atrasado de uma amostra(feed_1)
    --Termo proporcional malha corrente
    PROPORCIONAL <= (KP * ERRO) / 2 ** Q;
    Integrador : PROCESS (clk, rst)
    BEGIN
        IF rst = '1' THEN
            INTEGRAL <= 0;
            duty <= 0;
        ELSIF rising_edge(clk) THEN
            count <= count + 1;
            IF count = (DIV - 1) THEN
                count <= 0;
                -- Termo Integral
                INTEGRAL <= INTEGRAL + ((KI * ERRO) / 2 ** Q);
                --Feedback
                feed_0 <= e_sat * Kr;
                -- Valores Médios
                duty <= Y_sat / (2 ** (Q - 9));
                -- Auto Controle (OK)
                --duty <= to_integer(shift_right(to_signed(Iin_mA * Ki_Q16, 64), Q + 1));
            END IF;
        END IF;
    END PROCESS Integrador;

    Y <= PROPORCIONAL + INTEGRAL;

    e_sat <= Y_sat - Y;

    saturador : process(Y)
	BEGIN
		IF Y >= maximo THEN
			Y_sat <= maximo;
		ELSIF Y <=  minimo THEN
			Y_sat <= minimo;
		ELSE
			Y_sat <= Y;
		END IF;
	END PROCESS saturador;

END ARCHITECTURE;