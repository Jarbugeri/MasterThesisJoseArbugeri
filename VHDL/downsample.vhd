----------------------------------------------------------------------------------------------------------
-- Title        : DownSample ADC
-- Project      : Dissertação José Augusto Arbugeri
----------------------------------------------------------------------------------------------------------
-- File         : downsample.vhd
-- Author       : José Augusto Arbugeri (josearbugeri@gmail.com)
-- University   : Universidade Federal de Santa Catariana (UFSC)
-----------------------------------------------------------------------------------------------------------
-- Description  : Este program realiza o downsample das medidas do ADC.
-----------------------------------------------------------------------------------------------------------
-- Copyright (c) Centre for INEP, Universidade Federal de Santa Catariana
-----------------------------------------------------------------------------------------------------------
-- Revisions  :
-- Date        Version  Author   Description
-- 20/08/21      1.0     José      Created
------------------------------------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

ENTITY downsample IS
    GENERIC (
        CLOCK_FREQ : INTEGER := 48000000;
        ADC_FREQ : INTEGER := 750000;
        DOWNSAMPLE : INTEGER := 2
    );
    PORT (
        clk : IN STD_LOGIC;
        rst : IN STD_LOGIC;
        iin_in : IN STD_LOGIC_VECTOR(11 DOWNTO 0);
        vin_in : IN STD_LOGIC_VECTOR(11 DOWNTO 0);
        vout_in : IN STD_LOGIC_VECTOR(11 DOWNTO 0);
        iin_out : OUT STD_LOGIC_VECTOR(11 DOWNTO 0);
        vin_out : OUT STD_LOGIC_VECTOR(11 DOWNTO 0);
        vout_out : OUT STD_LOGIC_VECTOR(11 DOWNTO 0)
    );
END downsample;

ARCHITECTURE rtl OF downsample IS

    CONSTANT counte_val : INTEGER := DOWNSAMPLE*CLOCK_FREQ/ADC_FREQ - 1;
    SIGNAL counter : INTEGER := 0;
    SIGNAL vin_temp,vout_temp,iin_temp : STD_LOGIC_VECTOR(11 DOWNTO 0) := (others => '0');

BEGIN

    counter0 : PROCESS (clk, rst)
    BEGIN
        IF rst = '0' THEN
            counter <= 0;
        ELSIF rising_edge(clk) THEN
            counter <= counter + 1;
            IF counter = counte_val THEN
                counter <= 0;
                iin_temp <= iin_in;
                vin_temp <= vin_in;
                vout_temp <= vout_in;
            ELSE
                iin_temp <= iin_temp;
                vin_temp <= vin_temp;
                vout_temp <= vout_temp;
            END IF;

        END IF;
    END PROCESS counter0;

    -- Bypass

    iin_out <= iin_in;
    --vin_out <= vin_in;
    --vout_out <= vout_in;

    -- DownSample

    --iin_out <= iin_temp;
    vin_out <= vin_temp;
    vout_out <= vout_temp;

END ARCHITECTURE;