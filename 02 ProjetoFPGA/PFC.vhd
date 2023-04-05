----------------------------------------------------------------------------------------------------------
-- Title        : Conversor PFC
-- Project      : Dissertação José Augusto Arbugeri
----------------------------------------------------------------------------------------------------------
-- File         : FGPALaunch.vhd
-- Author       : José Augusto Arbugeri (josearbugeri@gmail.com)
-- University   : Universidade Federal de Santa Catariana (UFSC)
-----------------------------------------------------------------------------------------------------------
-- Description  : Este program realiza o controle de um conversor retificado PFC.
-----------------------------------------------------------------------------------------------------------
-- Copyright (c) Centre for INEP, Universidade Federal de Santa Catariana
-----------------------------------------------------------------------------------------------------------
-- Revisions  :
-- Date        Version  Author   Description
-- 2020          1.0     José      Created
-- 23/08/2021    2.0     José      New Project
--               2.1    José       Add downsample
--      1.3     José      Major revision
------------------------------------------------------------------------------------------------------------

LIBRARY IEEE;
USE IEEE.std_logic_1164.ALL;
USE IEEE.numeric_std.ALL;
USE IEEE.MATH_REAL.ALL;

ENTITY PFC IS
	PORT (
		--	Clock 50 MHz
		CLOCK : IN STD_LOGIC;
		--	Leds LD0-LD3
		LED : OUT STD_LOGIC_VECTOR(3 DOWNTO 0);
		--	Buttons BT1-BT2
		BT : IN STD_LOGIC_VECTOR(1 DOWNTO 0);
		--	ADCs AD0-AD9 signals (SPI)
		CS : OUT STD_LOGIC_VECTOR(9 DOWNTO 0);
		DATA : IN STD_LOGIC_VECTOR(9 DOWNTO 0);
		ADCLK : OUT STD_LOGIC_VECTOR(1 DOWNTO 0);
		--	General Use Outputs
		GPIO : INOUT STD_LOGIC_VECTOR(24 DOWNTO 0);
		SAIDA : OUT STD_LOGIC_VECTOR(23 DOWNTO 0)
	);
END ENTITY PFC;

ARCHITECTURE COMPORTAMENTO OF PFC IS

	--DECLARAÇÃO COMPONETES

	COMPONENT PLL IS
		PORT (
			areset : IN STD_LOGIC := '0';
			inclk0 : IN STD_LOGIC := '0';
			c0 : OUT STD_LOGIC;
			c1 : OUT STD_LOGIC;
			c2 : OUT STD_LOGIC;
			c3 : OUT STD_LOGIC;
			locked : OUT STD_LOGIC
		);
	END COMPONENT PLL;

	COMPONENT STARTKIT IS
		PORT (
			CLK : IN STD_LOGIC;
			RESET  : IN STD_LOGIC;
			BT     : IN  STD_LOGIC_VECTOR(1 DOWNTO 0);
			BT_PCB : IN  STD_LOGIC_VECTOR(1 DOWNTO 0);
			LED    : OUT STD_LOGIC_VECTOR(3 DOWNTO 0);
			RGB    : OUT STD_LOGIC_VECTOR(2 DOWNTO 0);
			BUZZER : OUT STD_LOGIC
		);
	END COMPONENT STARTKIT;

	COMPONENT modulador IS
		PORT (
			clk0, clk45, clk90, clk135, clk180, clk225, clk270, clk315 : IN STD_LOGIC;
			rst : IN STD_LOGIC;
			moduladora : IN INTEGER;
			il   : IN STD_LOGIC_VECTOR (11 DOWNTO 0);
			vout : IN STD_LOGIC_VECTOR (11 DOWNTO 0);
			vin  : IN STD_LOGIC_VECTOR (11 DOWNTO 0);
			trigger : OUT STD_LOGIC;
			Sa : OUT STD_LOGIC;
			Sb : OUT STD_LOGIC
		);
	END COMPONENT modulador;

	COMPONENT moving_average IS
		GENERIC (
			G_NBIT        : INTEGER := 13;
			G_AVG_LEN_LOG : INTEGER :=  4
		);
		PORT (
			i_clk  : IN STD_LOGIC;
			i_rstb : IN STD_LOGIC;
			i_data : IN STD_LOGIC_VECTOR(G_NBIT - 1 DOWNTO 0);
			o_data : OUT STD_LOGIC_VECTOR(G_NBIT - 1 DOWNTO 0)
		);
	END COMPONENT moving_average;

	COMPONENT PROTECAO IS
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
	END COMPONENT PROTECAO;

	COMPONENT AUTOCONTROLEv2 IS
		PORT (
			clk : IN STD_LOGIC;
			rst : IN STD_LOGIC;
			il : IN STD_LOGIC_VECTOR (11 DOWNTO 0);
			vout : IN STD_LOGIC_VECTOR (11 DOWNTO 0);
			vin : IN STD_LOGIC_VECTOR (11 DOWNTO 0);
			duty : OUT INTEGER
		);
	END COMPONENT;

	COMPONENT ADC IS
		GENERIC (
			ADCs : INTEGER;
			ADC_width : INTEGER
		);
		PORT (
			clk : IN STD_LOGIC;
			rst : IN STD_LOGIC;
			clk_div : IN INTEGER;
			enable : IN STD_LOGIC;
			-- Pins TO/FROM ADC's chips
			cdata : IN STD_LOGIC_VECTOR(ADCs - 1 DOWNTO 0);
			cs : OUT STD_LOGIC_VECTOR(ADCs - 1 DOWNTO 0);
			cclk : OUT STD_LOGIC;
			-- Data out FROM ADC's converter
			ADC0 : OUT UNSIGNED(ADC_width - 1 DOWNTO 0);
			ADC1 : OUT UNSIGNED(ADC_width - 1 DOWNTO 0);
			ADC2 : OUT UNSIGNED(ADC_width - 1 DOWNTO 0)
		);
	END COMPONENT ADC;

	COMPONENT downsample IS
		GENERIC (
			CLOCK_FREQ : INTEGER := 48e6;
			ADC_FREQ   : INTEGER := 750e3;
			DOWNSAMPLE : INTEGER := 2
		);
		PORT (
			clk : IN STD_LOGIC;
			rst : IN STD_LOGIC;
			iin_in  : IN STD_LOGIC_VECTOR(11 DOWNTO 0);
			vin_in  : IN STD_LOGIC_VECTOR(11 DOWNTO 0);
			vout_in : IN STD_LOGIC_VECTOR(11 DOWNTO 0);
			iin_out  : OUT STD_LOGIC_VECTOR(11 DOWNTO 0);
			vin_out  : OUT STD_LOGIC_VECTOR(11 DOWNTO 0);
			vout_out : OUT STD_LOGIC_VECTOR(11 DOWNTO 0)
		);
 	END COMPONENT downsample;

	--DECLARAÇÃO SINAIS

	SIGNAL BT_FLAG : STD_LOGIC_VECTOR(1 DOWNTO 0);
	SIGNAL LEDS : STD_LOGIC_VECTOR(3 DOWNTO 0);
	SIGNAL CLOCK50Mhz : STD_LOGIC;
	SIGNAL CLOCK48M_0, CLOCK48M_45, CLOCK48M_90, CLOCK48M_135 : STD_LOGIC;
	SIGNAL CLOCK48M_180, CLOCK48M_225, CLOCK48M_270, CLOCK48M_315 : STD_LOGIC;
	SIGNAL OverCurrent : STD_LOGIC;

	-- Saida ADC e Downsample e Media movel
	SIGNAL ADC_VIN, ADC_IIN, ADC_VOUT          : UNSIGNED(11 DOWNTO 0);
	SIGNAL ADC_VIN_DS, ADC_IIN_DS, ADC_VOUT_DS : STD_LOGIC_VECTOR(11 DOWNTO 0);
	SIGNAL VIN_Mean, IIN_Mean, VOUT_Mean       : STD_LOGIC_VECTOR(12 DOWNTO 0);

	SIGNAL PWM_SCP, PWM_SCN : STD_LOGIC;
	SIGNAL PWMA, PWMB : STD_LOGIC;
	SIGNAL ENABLE_SCP, ENABLE_SCN : STD_LOGIC;
	SIGNAL Vo_mV : UNSIGNED(23 DOWNTO 0);
	SIGNAL CORRENTE_MAX : INTEGER;
	SIGNAL START_ADC : STD_LOGIC;
	SIGNAL PUSH1, PUSH2 : STD_LOGIC;
	SIGNAL OVER_CH, OVER_CS, OVER_VS : STD_LOGIC;

	SIGNAL DUTY_CYCLE : STD_LOGIC_VECTOR(9 DOWNTO 0);
	SIGNAL pwmEnable : STD_LOGIC_VECTOR(24 DOWNTO 0);
	SIGNAL duty : INTEGER;
	SIGNAL ADC_CLOCK_OUT : STD_LOGIC;
	SIGNAL ADC_DATA_OUT_1 : STD_LOGIC_VECTOR(13 DOWNTO 0);

BEGIN

	PLL_1 : PLL
	PORT MAP(
		areset => '0',
		inclk0 => CLOCK50Mhz,
		C0 => CLOCK48M_0,
		C1 => CLOCK48M_45,
		C2 => CLOCK48M_90,
		c3 => CLOCK48M_135,
		locked => OPEN
	);

	CLOCK48M_180 <= NOT(CLOCK48M_0);
	CLOCK48M_225 <= NOT(CLOCK48M_45);
	CLOCK48M_270 <= NOT(CLOCK48M_90);
	CLOCK48M_315 <= NOT(CLOCK48M_135);

	COMP1 : STARTKIT PORT MAP(
		CLK   => CLOCK48M_0,
		RESET => BT_FLAG(0),
		BT    => BT_FLAG,
		BT_PCB => PUSH2 & PUSH1,
		LED    => LEDS,
		RGB    => OPEN,
		BUZZER => OPEN
	);

	PWM1 : modulador 
	PORT MAP(
		clk0   => CLOCK48M_0,
		clk45  => CLOCK48M_45,
		clk90  => CLOCK48M_90,
		clk135 => CLOCK48M_135,
		clk180 => CLOCK48M_180,
		clk225 => CLOCK48M_225,
		clk270 => CLOCK48M_270,
		clk315 => CLOCK48M_315,
		rst  => '0',
		il   => STD_LOGIC_VECTOR(IIN_Mean)(11 DOWNTO 0),
		vout => STD_LOGIC_VECTOR(VOUT_Mean)(11 DOWNTO 0),
		vin  => STD_LOGIC_VECTOR(VIN_Mean)(11 DOWNTO 0),
		moduladora => duty,
		trigger => START_ADC,
		Sa => PWMA,
		Sb => PWMB
	);

	ADC_kit : ADC
	GENERIC MAP(
		ADCs => 3,
		ADC_width => 12
	)
	PORT MAP(
		clk => CLOCK48M_0,
		rst => '0',
		clk_div => 2,
		enable  => START_ADC,
		-- Pins TO/FROM ADC's chips
		cdata => DATA(3 DOWNTO 1),
		cs    => CS(3 DOWNTO 1),
		cclk  => ADC_CLOCK_OUT,
		-- Data out FROM ADC's converter
		ADC0 => ADC_VOUT,
		ADC1 => ADC_IIN,
		ADC2 => ADC_VIN
	);

	down_sample : downsample 
	GENERIC MAP(
		CLOCK_FREQ => 48000000,
		ADC_FREQ   => 750000,
		DOWNSAMPLE => 8
	)
	PORT MAP(
		clk => CLOCK48M_0,
		rst => '0',
		iin_in   => STD_LOGIC_VECTOR(ADC_IIN),
		vin_in   => STD_LOGIC_VECTOR(ADC_VIN),
		vout_in  => STD_LOGIC_VECTOR(ADC_VOUT),
		iin_out  => ADC_IIN_DS, --,
		vin_out  => ADC_VIN_DS, --,
		vout_out => ADC_VOUT_DS --
	);


	maIin : moving_average
	GENERIC MAP(
		G_NBIT => 13,
		G_AVG_LEN_LOG => 4
	)
	PORT MAP(
		i_clk => START_ADC,
		i_rstb => '0',
		i_data => '0' & STD_LOGIC_VECTOR(ADC_IIN),
		o_data => IIN_Mean --
	);
	maVout : moving_average
	GENERIC MAP(
		G_NBIT => 13,
		G_AVG_LEN_LOG => 4
	)
	PORT MAP(
		i_clk => START_ADC,
		i_rstb => '0',
		i_data => '0' & STD_LOGIC_VECTOR(ADC_VOUT),
		o_data => VOUT_Mean --
	);

	maVin : moving_average
	GENERIC MAP(
		G_NBIT => 13,
		G_AVG_LEN_LOG => 4
	)
	PORT MAP(
		i_clk => START_ADC,
		i_rstb => '0',
		i_data => '0' & STD_LOGIC_VECTOR(ADC_VIN),
		o_data => VIN_Mean --
	);

	PROTECAO_KIT : PROTECAO
	PORT MAP(
		CLK    => CLOCK48M_0,
		RESET  => BT_FLAG(0),
		PWMAin => PWMA,
		PWMBin => PWMB,
		OverC          => OverCurrent,
		CORRENTE    => STD_LOGIC_VECTOR(IIN_Mean)(11 DOWNTO 0),
		TENSAO      => STD_LOGIC_VECTOR(VOUT_Mean)(11 DOWNTO 0),
		OVERTENSION    => OVER_VS,
		OverC_HARDWARE => OVER_CH,
		OverC_SOFTWARE => OVER_CS,
		CORRENTE_MAX   => CORRENTE_MAX,
		ENABLEP => ENABLE_SCP,
		ENABLEN => ENABLE_SCN,
		PWMPout => PWM_SCP,
		PWMNout => PWM_SCN
	);

	CONTROLE : AUTOCONTROLEv2
	PORT MAP(
		clk  => START_ADC,
		rst  => BT_FLAG(0),
		il   => STD_LOGIC_VECTOR(IIN_Mean)(11 DOWNTO 0),
		vout => STD_LOGIC_VECTOR(VOUT_Mean)(11 DOWNTO 0),
		vin  => STD_LOGIC_VECTOR(VIN_Mean)(11 DOWNTO 0),
		duty => duty
	);

	-------------------------------------------------------------------------------
	-- TRISTATE : Define as entradas do vetor GPIO como alta impedancia
	-------------------------------------------------------------------------------

	GPIO(6 DOWNTO 0)	<= (others => '0');	--: OUT STD_LOGIC_VECTOR(6  DOWNTO 0);
	GPIO(7)             <= 'Z';             --: IN  STD_LOGIC_VECTOR(7  DOWNTO 7);
	GPIO(8)             <= '0'; 
	GPIO(16 DOWNTO 10)  <= (others => '0');	--: OUT STD_LOGIC_VECTOR(20 DOWNTO 8);
	GPIO(22 DOWNTO 21)  <= "ZZ";            --: IN  STD_LOGIC_VECTOR(22 DOWNTO 21);
	
	-------------------------------------------------------------------------------
	-- Entradas : Entradas referente ao conversor projetado e não o launchBruxilis
	-------------------------------------------------------------------------------

	CLOCK50Mhz <= CLOCK;

	-----PushButton----------------------------------------------------------------
	-----OverCurrent----------------------------------------------------------------

	PROCESS (START_ADC)
	BEGIN
		IF (RISING_EDGE(START_ADC)) THEN
			OverCurrent <= GPIO(7);
			PUSH1 <= NOT(GPIO(21));
			PUSH2 <= NOT(GPIO(22));
			BT_FLAG <= NOT BT; --Botão solto = 1, Botão pressionado = 0
		END IF;
	END PROCESS;

	-------------------------------------------------------------------------------
	-- Saídas : Saídas referente ao conversor projetado e não o launchBruxilis
	-------------------------------------------------------------------------------
	-----CLOCK ADC-----------------------------------------------------------------

	ADCLK(0) <= ADC_CLOCK_OUT;
	ADCLK(1) <= '0';

	-- Chips selects não utilizados são setados para '0'
	CS(0) <= '0';
	CS(9 DOWNTO 4) <= (others => '0');

	-----Led bruxilis--------------------------------------------------------------

	LED <= LEDS; 

	-----Buzzar--------------------------------------------------------------------	

	GPIO(15) <= '0';

	-----LEd's RGB - 20 B, 19 G, 18 R----------------------------------------------		

	GPIO(20 DOWNTO 18) <= (OVER_CH & OVER_VS & OVER_CS) AND "111";

	-----PWMN----------------------------------------------------------------------			

	GPIO(9) <= PWM_SCN; --PWM SCN
	GPIO(17) <= ENABLE_SCN; --ENABLE SCN

	-----PWMP----------------------------------------------------------------------

	GPIO(24) <= PWM_SCP; --PWM SCP
	GPIO(23) <= ENABLE_SCP; --ENABLE SCP

END ARCHITECTURE COMPORTAMENTO;