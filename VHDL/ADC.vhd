--------------------------------------------------------------------------------
--
--   FileName:         spi_master.vhd
--   Dependencies:     none
--   Design Software:  Quartus II Version 9.0 Build 132 SJ Full Version
--
--
--   Version History
--   Version 1.0 7/23/2010 Scott Larson
--     Initial Public Release
--   Version 1.1 4/11/2013 Scott Larson
--     Corrected ModelSim simulation error (explicitly reset clk_toggles signal)
--    
--------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.std_logic_arith.ALL;
USE ieee.std_logic_unsigned.ALL;

ENTITY ADC IS
  GENERIC (
    ADCs : INTEGER := 3; -- Number of ADC slaves
    ADC_width : INTEGER := 12); --data bus width
  PORT (
    clk : IN STD_LOGIC; --system clock
    rst : IN STD_LOGIC; --asynchronous reset
    clk_div : IN INTEGER; --system clock cycles per 1/2 period of sclk
    enable : IN STD_LOGIC; --initiate transaction
    -- Pins TO/FROM ADC's chips
    cdata : IN STD_LOGIC_VECTOR(ADCs - 1 DOWNTO 0); -- chip data
    cs : OUT STD_LOGIC_VECTOR(ADCs - 1 DOWNTO 0); -- chip select
    cclk : OUT STD_LOGIC; -- chip clk
    -- Data out FROM ADC's converter
    ADC0 : OUT UNSIGNED(ADC_width - 1 DOWNTO 0); --data received ADC0
    ADC1 : OUT UNSIGNED(ADC_width - 1 DOWNTO 0); --data received ADC1
    ADC2 : OUT UNSIGNED(ADC_width - 1 DOWNTO 0) --data received ADC2   
  ); --data received
END ADC;

ARCHITECTURE rtl OF ADC IS

  COMPONENT spi_controller IS
    GENERIC (
      d_width : INTEGER := 16 --data bus width
    );
    PORT (
      clk : IN STD_LOGIC; --system clk
      rst : IN STD_LOGIC; --asynchronous reset
      enable : IN STD_LOGIC; --initiate transaction
      cpol : IN STD_LOGIC; --spi clk polarity
      cpha : IN STD_LOGIC; --spi clk phase
      cont : IN STD_LOGIC; --continuous mode command
      clk_div : IN INTEGER; --system clk cycles per 1/2 period of sck
      tx_data : IN STD_LOGIC_VECTOR(d_width - 1 DOWNTO 0); --data to transmit
      sdi : IN STD_LOGIC; -- Serial data in
      sck : BUFFER STD_LOGIC; --spi clk
      cs : BUFFER STD_LOGIC; --chip select
      sdo : OUT STD_LOGIC; -- Serial data out
      busy : OUT STD_LOGIC; --busy / data ready signal
      rx_data : OUT STD_LOGIC_VECTOR(d_width - 1 DOWNTO 0) --data received
    );
  END COMPONENT spi_controller;

  CONSTANT CPOL : STD_LOGIC := '1';
  CONSTANT CPHA : STD_LOGIC := '0';
  CONSTANT CONT : STD_LOGIC := '0';
  CONSTANT d_width : INTEGER := 14;

  TYPE data_array IS ARRAY (ADCs - 1 DOWNTO 0) OF STD_LOGIC_VECTOR(d_width - 1 DOWNTO 0);
  TYPE clk_array IS ARRAY (ADCs - 1 DOWNTO 0) OF STD_LOGIC;
  
  SIGNAL data : data_array;
  SIGNAL clk_o : clk_array;
  SIGNAL busy_array : clk_array;  
  SIGNAL clk_sample : STD_LOGIC;
  SIGNAL enable_adc : STD_LOGIC;

BEGIN

  clk_sample <= enable;

  ADC_gen : FOR i IN 0 TO ADCs - 1 GENERATE
    ADC_x : spi_controller
    GENERIC MAP(
      d_width => d_width
    )
    PORT MAP(
      clk => clk,
      rst => rst,
      enable => enable_adc,
      cpol => CPOL,
      cpha => CPHA,
      cont => CONT,
      clk_div => clk_div,
      tx_data => (OTHERS => '0'),
      sdi => cdata(i),
      sck => clk_o(i),
      cs => cs(i),
      sdo => OPEN,
      busy => busy_array(i),
      rx_data => data(i)
    );
  END GENERATE ADC_gen;

  proc_name: process(clk_sample, rst, busy_array)
  begin
    if rst = '1' then
      enable_adc <= '0';
    elsif rising_edge(clk_sample) then
      enable_adc <= '1';
    end if;
    if (busy_array(0) AND busy_array(1) AND busy_array(2)) = '1' THEN
      enable_adc <= '0';
    end if;    
  end process proc_name;

  cclk <= clk_o(0);
  ADC0 <= UNSIGNED(data(0)(ADC_width - 1 DOWNTO 0));
  ADC1 <= UNSIGNED(data(1)(ADC_width - 1 DOWNTO 0));
  ADC2 <= UNSIGNED(data(2)(ADC_width - 1 DOWNTO 0));	
  

END ARCHITECTURE rtl;