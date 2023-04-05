--------------------------------------------------------------------------------
--
--   FileName:         spi_controller.vhd
--   Dependencies:     none
--   Design Software:  Quartus II Version 9.0 Build 132 SJ Full Version
--
--   HDL CODE IS PROVIDED "AS IS."  DIGI-KEY EXPRESSLY DISCLAIMS ANY
--   WARRANTY OF ANY KIND, WHETHER EXPRESS OR IMPLIED, INCLUDING BUT NOT
--   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
--   PARTICULAR PURPOSE, OR NON-INFRINGEMENT. IN NO EVENT SHALL DIGI-KEY
--   BE LIABLE FOR ANY INCIDENTAL, SPECIAL, INDIRECT OR CONSEQUENTIAL
--   DAMAGES, LOST PROFITS OR LOST DATA, HARM TO YOUR EQUIPMENT, COST OF
--   PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
--   BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF),
--   ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER SIMILAR COSTS.
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

ENTITY spi_controller IS
  GENERIC (
    d_width : INTEGER := 16
  ); --data bus width
  PORT (
    clk : IN STD_LOGIC; --system clk
    rst : IN STD_LOGIC; --asynchronous reset
    enable : IN STD_LOGIC; --initiate transaction
    cpol : IN STD_LOGIC; --spi clk polarity
    cpha : IN STD_LOGIC; --spi clk phase
    cont : IN STD_LOGIC; --continuous mode command
    clk_div : IN INTEGER; --system clk cycles per 1/2 period of sck
    tx_data : IN STD_LOGIC_VECTOR(d_width - 1 DOWNTO 0); --data to transmit
    sdi : IN STD_LOGIC; --master in, slave out
    sck : BUFFER STD_LOGIC; --spi clk
    cs : BUFFER STD_LOGIC; --chip select
    sdo : OUT STD_LOGIC; --master out, slave in
    busy : OUT STD_LOGIC; --busy / data ready signal
    rx_data : OUT STD_LOGIC_VECTOR(d_width - 1 DOWNTO 0)); --data received
END spi_controller;

ARCHITECTURE rtl OF spi_controller IS
  TYPE machine IS(ready, execute); --state machine data type
  SIGNAL state : machine; --current state
  SIGNAL clk_ratio : INTEGER; --current clk_div
  SIGNAL count : INTEGER; --counter to trigger sck from system clk
  SIGNAL clk_toggles : INTEGER RANGE 0 TO d_width * 2 + 1; --count spi clk toggles
  SIGNAL assert_data : STD_LOGIC; --'1' is tx sck toggle, '0' is rx sck toggle
  SIGNAL continue : STD_LOGIC; --flag to continue transaction
  SIGNAL rx_buffer : STD_LOGIC_VECTOR(d_width - 1 DOWNTO 0); --receive data buffer
  SIGNAL tx_buffer : STD_LOGIC_VECTOR(d_width - 1 DOWNTO 0); --transmit data buffer
  SIGNAL last_bit_rx : INTEGER RANGE 0 TO d_width * 2; --last rx data bit location
BEGIN
  PROCESS (clk, rst)
  BEGIN

    IF (rst = '1') THEN -- Se reset = 1
      busy <= '1'; -- set busy signal
      cs <= '1'; -- 
      sdo <= 'Z'; --set master out to high impedance
      rx_data <= (OTHERS => '0'); --clear receive data port
      state <= ready; --go to ready state when reset is exited

    ELSIF rising_edge(clk) THEN
      CASE state IS --state machine

        WHEN ready =>
          busy <= '0'; --clk out not busy signal
          cs <= '1'; --set all slave select outputs high
          sdo <= 'Z'; --set sdo output high impedance
          continue <= '0'; --clear continue flag

          --user input to initiate transaction
          IF enable = '1' THEN
            busy <= '1'; --set busy signal
            IF (clk_div = 0) THEN --check for valid spi speed
              clk_ratio <= 1; --set to maximum speed if zero
              count <= 1; --initiate system-to-spi clk counter
            ELSE
              clk_ratio <= clk_div; --set to input selection if valid
              count <= clk_div; --initiate system-to-spi clk counter
            END IF;
            sck <= cpol; --set spi clk polarity
            assert_data <= NOT cpha; --set spi clk phase
            tx_buffer <= tx_data; --clk in data for transmit into buffer
            clk_toggles <= 0; --initiate clk toggle counter
            last_bit_rx <= d_width * 2 + conv_integer(cpha) - 1; --set last rx data bit
            state <= execute; --proceed to execute state
          ELSE
            state <= ready; --remain in ready state
          END IF;

        WHEN execute =>
          busy <= '1'; --set busy signal
          cs <= '0'; --set proper slave select output

          --system clk to sck ratio is met
          IF count = clk_ratio THEN
            count <= 1; --reset system-to-spi clk counter
            assert_data <= NOT assert_data; --switch transmit/receive indicator
            IF clk_toggles = d_width * 2 + 1 THEN
              clk_toggles <= 0; --reset spi clk toggles counter
            ELSE
              clk_toggles <= clk_toggles + 1; --increment spi clk toggles counter
            END IF;

            --spi clk toggle needed
            IF clk_toggles <= d_width * 2 AND cs = '0' THEN
              sck <= NOT sck; --toggle spi clk
            END IF;

            --receive spi clk toggle
            IF assert_data = '0' AND clk_toggles < last_bit_rx + 1 AND cs = '0' THEN
              rx_buffer <= rx_buffer(d_width - 2 DOWNTO 0) & sdi; --shift in received bit
            END IF;

            --transmit spi clk toggle
            IF assert_data = '1' AND clk_toggles < last_bit_rx THEN
              sdo <= tx_buffer(d_width - 1); --clk out data bit
              tx_buffer <= tx_buffer(d_width - 2 DOWNTO 0) & '0'; --shift data transmit buffer
            END IF;

            --last data receive, but continue
            IF clk_toggles = last_bit_rx AND cont = '1' THEN
              tx_buffer <= tx_data; --reload transmit buffer
              clk_toggles <= last_bit_rx - d_width * 2 + 1; --reset spi clk toggle counter
              continue <= '1'; --set continue flag
            END IF;

            --normal end of transaction, but continue
            IF continue = '1' THEN
              continue <= '0'; --clear continue flag
              busy <= '0'; --clk out signal that first receive data is ready
              rx_data <= rx_buffer; --clk out received data to output port    
            END IF;

            --end of transaction
            IF (clk_toggles = d_width * 2 + 1) AND cont = '0' THEN
              busy <= '0'; --clk out not busy signal
              cs <= '1'; --set all slave selects high
              sdo <= 'Z'; --set sdo output high impedance
              rx_data <= rx_buffer; --clk out received data to output port
              state <= ready; --return to ready state
            ELSE --not end of transaction
              state <= execute; --remain in execute state
            END IF;

          ELSE --system clk to sck ratio not met
            count <= count + 1; --increment counter
            state <= execute; --remain in execute state
          END IF;

      END CASE;
    END IF;
  END PROCESS;
END rtl;