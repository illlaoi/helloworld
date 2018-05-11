----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    18:50:23 03/26/2018 
-- Design Name: 
-- Module Name:    I2C - Behavioral 
-- Project Name: 
-- Target Devices: 
-- Tool versions: 
-- Description: 
--
-- Dependencies: 
--
-- Revision: 
-- Revision 0.01 - File Created
-- Additional Comments: 
--
----------------------------------------------------------------------------------



---- Uncomment the following library declaration if instantiating
---- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;


LIBRARY IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

ENTITY I2C IS 
	PORT (RESET 		: IN 	STD_LOGIC;
			CLK_40M		: IN	STD_LOGIC;

		READ_H			: IN 	STD_LOGIC;
		WRITE_H			: IN 	STD_LOGIC;

			RD_DOWN		: OUT		STD_LOGIC;
			WR_DOWN		: OUT		STD_LOGIC;
			ERROR_O		: OUT 		STD_LOGIC;

			SCL,SDATA 	: INOUT STD_LOGIC;

			ZX          : OUT 	STD_LOGIC_VECTOR(6 DOWNTO 0);--CHOOSE THE WORD
			WX 			: OUT 	STD_LOGIC_VECTOR(7 downto 0));--CHOOSE THE ROOM
END I2C;

ARCHITECTURE structural OF I2C IS
component simple_i2c is
	port (
		clk : in std_logic;
		ena : in std_logic;
		nReset : in std_logic;
		-- input signals
		start,
		stop,
		read,
		write,
		ack_in:IN std_logic;
		Din : in std_logic_vector(7 downto 0);

		-- output signals
		cmd_ack : out std_logic;
		ack_out : out std_logic;
		Dout : out std_logic_vector(7 downto 0);
		simple_STATE_POINT:OUT STD_LOGIC_VECTOR(3 DOWNTO 0);
				core_state:out std_logic_vector(2 downto 0);
		-- i2c signals
		SCL : inout std_logic;
		SDA : inout std_logic
	);
end component simple_i2c;

CONSTANT DEVICE_ADDRESS_WRITE			: STD_LOGIC_VECTOR (7 DOWNTO 0):="10100000";
CONSTANT DEVICE_ADDRESS_READ			: STD_LOGIC_VECTOR (7 DOWNTO 0):="10100001";
CONSTANT WORD_REG_ADDRESS				: STD_LOGIC_VECTOR (7 DOWNTO 0):="00000001";
CONSTANT	DATA_CHANGED					: STD_LOGIC_VECTOR(3 DOWNTO 0):="0110";
CONSTANT ENA_TOP							: STD_LOGIC:='1';

SIGNAL CLK_100K 				: STD_LOGIC;
SIGNAL CLK_1K					: STD_LOGIC;
SIGNAL ACK_CHECK_IN				: STD_LOGIC;
SIGNAL CMD_START				: STD_LOGIC;
SIGNAL CMD_READ					: STD_LOGIC;
SIGNAL CMD_WRITE				: STD_LOGIC;
SIGNAL CMD_STOP					: STD_LOGIC;
SIGNAL q						: STD_LOGIC_VECTOR(3 downto 0);
SIGNAL irxack					: STD_LOGIC;
SIGNAL ack_feedback 			: STD_LOGIC;


SIGNAL simple_STATE 			: STD_LOGIC_VECTOR(3 DOWNTO 0);
SIGNAL core_state				: STD_LOGIC_VECTOR(2 DOWNTO 0);
SIGNAL TOP_STATE 				: STD_LOGIC_VECTOR(3 DOWNTO 0);
SIGNAL top_Dout 				: std_logic_vector(7 downto 0);--(读取出来的8位)
SIGNAL DATA_READ 				: std_logic_vector(7 downto 0);--(读取出来的8位)
SIGNAL DATA_REALWRITE			: STD_LOGIC_VECTOR(7 DOWNTO 0);--（写进サ?位）


BEGIN

u0: simple_i2c port map (clk 	=>CLK_100K,
						ena 	=>ENA_TOP,
						nReset 	=>RESET,
						start 	=>CMD_START,
						stop 	=>CMD_STOP,
						read 	=>CMD_READ,
						write 	=>CMD_WRITE,
						ack_in 	=>ack_feedback, 
						ack_out	=>irxack,
						Din 	=>DATA_REALWRITE,
						cmd_ack =>ACK_CHECK_IN,
						Dout 	=>top_Dout,
						simple_STATE_POINT=>simple_STATE,
						core_state=>		core_state,
						SCL 	=>SCL,
						SDA 	=>SDATA);



PROCESS(CLK_40M) --从40M时钟产生100K,100K和50K时钟信号
VARIABLE COUNT_1K		: INTEGER RANGE 0 TO 25E3;
BEGIN
	IF (CLK_40M'EVENT AND CLK_40M='1') THEN
		IF(COUNT_1K=25E3) THEN 
			COUNT_1K:=0;
			CLK_1K <= NOT CLK_1K;
		ELSE COUNT_1K :=COUNT_1K+1;
		END IF;
	END IF;
END PROCESS;


PROCESS(CLK_40M,RESET) 
VARIABLE COUNT_100K		: INTEGER RANGE 0 TO 2E2;
BEGIN
	IF (RESET='0') THEN COUNT_100K :=0;CLK_100K<='0';
	  ELSIF (CLK_40M'EVENT AND CLK_40M='1') THEN
	  	IF(COUNT_100K=2E6) THEN 
	  		COUNT_100K:=0; 
	  		CLK_100K<= NOT CLK_100K;
	  		ELSE COUNT_100K:=COUNT_100K +1;
		END IF;
	END IF;
END PROCESS;

Display_zx: process(q)
begin
   CASE  q  IS
   when "0000" =>  	zx <= "1000000" ; --0 g-a LOW ACTIVE
   when "0001"=>  	zx <= "1111001" ; --1
   when "0010" =>  	zx <= "0100100" ;
   when "0011" =>  	zx <= "0110000" ;
   when "0100" =>  	zx <= "0011001" ;
   when "0101" =>  	zx <= "0010010" ;
   when "0110" =>  	zx <= "0000010" ;
   when "0111" =>  	zx <= "1111000" ;
   when "1000" =>  	zx <= "0000000" ;
   when "1001" =>  	zx <= "0010000" ;
   when "1010" =>  	zx <= "0001000" ;
   when "1011" =>  	zx <= "0000011" ;
   when "1100" =>  	zx <= "1000110" ;
   when "1101" =>  	zx <= "0100001" ;
   when "1110" =>  	zx <= "0000110" ;
   when "1111" =>  	zx <= "0001110" ;
   when OTHERS =>  	zx <= "1111111" ; --NULL
   END CASE ;
end process;

Display_wx: process(CLK_1K,RESET,top_Dout)
   variable count1  : std_logic_vector(2 downto 0);
   variable qTEMP	: std_logic_vector(3 downto 0);
   variable wxTEMP	: STD_LOGIC_VECTOR(7 DOWNTO 0);
begin

   qTEMP:="0000";
   wxTEMP:="11111111";
   CASE  count1  IS

   when "000" =>  wxTEMP := "11111110" ; qTEMP:=DATA_READ(3 DOWNTO 0) ;
   when "001" =>  wxTEMP := "11111101" ; qTEMP:=DATA_READ(7 DOWNTO 4) ;
   when "010" =>  wxTEMP := "11111011" ; qTEMP:=DATA_REALWRITE(3 DOWNTO 0) ;
   when "011" =>  wxTEMP := "11110111" ; qTEMP:=DATA_REALWRITE(7 DOWNTO 4) ;
   when "100" =>  wxTEMP := "11111111" ; 
   when "101" =>  wxTEMP := "11011111" ; qTEMP:='0'&core_state;
   when "110" =>  wxTEMP := "10111111" ; qTEMP:=simple_STATE(3 DOWNTO 0) ;
   when "111" =>  wxTEMP := "01111111" ; qTEMP:=TOP_STATE(3 DOWNTO 0) ;
   --when "100" =>  wxTEMP := "11101111" ; qTEMP:=simple_STATE(0);
   --when "101" =>  wxTEMP := "11011111" ; qTEMP:=simple_STATE(1);
   --when "110" =>  wxTEMP := "10111111" ; qTEMP:=STATE_POINT(0) ;
   --when "111" =>  wxTEMP := "01111111" ; qTEMP:=STATE_POINT(1) ;
   when OTHERS => wxTEMP := "11111111" ; 
	END CASE ;
	IF (RESET ='0') THEN 
		wx<="11111111";
		q<="0000";



	ELSIF CLK_1K'event and CLK_1K='1' then
      count1 := count1+1;
      q<=qTEMP;
      wx<=wxTEMP;

   end if;
end process;

topstatemachine: block
TYPE STATETYPE IS (	IDLE,
				TOP_WR_DEV_ADDR,
				TOP_WR_ERR,
				TOP_WR_REG_ADDR,
				TOP_WR_DATA,
				TOP_WR_STOP,
				TOP_RD_DEV_ADDR,
				TOP_RD_REG_ADDR,
				TOP_RD_DEV_ADDR1,
				TOP_RD_DATA,
				TOP_RD_STOP,
				TOP_RD_ACK,
				TOP_WR_ACK,
				TOP_WAIT);		
SIGNAL I2CCONTROLSTATE 		: STATETYPE;

BEGIN

PROCESS(CLK_100K,RESET)
BEGIN
	IF (CLK_100K'EVENT AND CLK_100K='1') THEN
	IF (RESET= '0') THEN
			ERROR_O<='1';
	ELSIF (I2CCONTROLSTATE =IDLE) THEN
			ERROR_O<='1';
	ELSIF (I2CCONTROLSTATE = TOP_WR_ERR) THEN
			ERROR_O <='0';
	END IF;
END IF;	
END PROCESS;


PROCESS(CLK_100K,RESET)
BEGIN
	IF (CLK_100K'EVENT AND CLK_100K='1') THEN
	IF (RESET= '0') THEN
			RD_DOWN<='1';
			WR_DOWN<='1';
	ELSIF (I2CCONTROLSTATE =TOP_RD_ACK) THEN
			RD_DOWN<='0';
	ELSIF (I2CCONTROLSTATE = TOP_WR_ACK) THEN
			WR_DOWN<='0';
	ELSE
		RD_DOWN<='1';
		WR_DOWN<='1';
	END IF;
END IF;	
END PROCESS;

PROCESS(CLK_100K,RESET)
BEGIN
	IF (CLK_100K'EVENT AND CLK_100K='1') THEN
	IF (RESET= '0') THEN
			CMD_START<='0';
	ELSIF (ACK_CHECK_IN='1') THEN
			CMD_START<='0';
	ELSIF (I2CCONTROLSTATE = TOP_WR_DEV_ADDR OR  I2CCONTROLSTATE =TOP_RD_DEV_ADDR OR I2CCONTROLSTATE =TOP_RD_DEV_ADDR1) THEN
			CMD_START <='1';
	END IF;
END IF;	
END PROCESS;

PROCESS(CLK_100K,RESET)
BEGIN
	IF (CLK_100K'EVENT AND CLK_100K='1') THEN
	IF (RESET= '0') THEN
			CMD_STOP<='0';
	ELSIF (ACK_CHECK_IN='1') THEN
			CMD_STOP<='0';
	ELSIF (I2CCONTROLSTATE = TOP_WR_STOP OR I2CCONTROLSTATE =TOP_RD_STOP) THEN
			CMD_STOP <='1';
	END IF;
END IF;	
END PROCESS;

PROCESS(CLK_100K,RESET)
BEGIN
	IF (CLK_100K'EVENT AND CLK_100K='1') THEN
	IF (RESET= '0') THEN
			ack_feedback<='0';
	ELSE
			ack_feedback <='1';
	END IF;
END IF;	
END PROCESS;

PROCESS(CLK_100K,RESET)
BEGIN
	IF (CLK_100K'EVENT AND CLK_100K='1') THEN
	IF (RESET= '0') THEN
			CMD_WRITE<='0';
	ELSIF (ACK_CHECK_IN='1') THEN
			CMD_WRITE<='0';
	ELSIF (I2CCONTROLSTATE =TOP_WR_DEV_ADDR OR I2CCONTROLSTATE =TOP_WR_REG_ADDR OR I2CCONTROLSTATE =TOP_WR_DATA OR I2CCONTROLSTATE =TOP_RD_DEV_ADDR OR I2CCONTROLSTATE =TOP_RD_DEV_ADDR1 OR I2CCONTROLSTATE =TOP_RD_REG_ADDR) THEN
			CMD_WRITE <='1';
	END IF;
END IF;	
END PROCESS;

PROCESS(CLK_100K,RESET)
BEGIN
	IF (CLK_100K'EVENT AND CLK_100K='1') THEN
	IF (RESET= '0') THEN
			CMD_READ<='0';
	ELSIF (ACK_CHECK_IN='1') THEN
			CMD_READ<='0';
	ELSIF (I2CCONTROLSTATE = TOP_RD_DATA) THEN
			CMD_READ <='1';
	END IF;
END IF;	
END PROCESS;

PROCESS(CLK_100K,RESET)
BEGIN
	IF (CLK_100K'EVENT AND CLK_100K='1') THEN
	IF (RESET= '0') THEN
			DATA_READ<="00000000";
	ELSIF (I2CCONTROLSTATE = TOP_RD_DATA AND ACK_CHECK_IN='1') THEN
			DATA_READ <=top_Dout;
	else
		DATA_READ<="00000000";
	END IF;
END IF;	
END PROCESS;

PROCESS(CLK_100K,RESET)
BEGIN
	IF (CLK_100K'EVENT AND CLK_100K='1') THEN
	IF (RESET= '0') THEN
			DATA_REALWRITE<=(OTHERS=>'0');
	ELSE
		CASE I2CCONTROLSTATE IS 
			WHEN TOP_WR_DEV_ADDR  =>
				DATA_REALWRITE<= DEVICE_ADDRESS_WRITE;
			WHEN TOP_RD_DEV_ADDR =>
				DATA_REALWRITE<= DEVICE_ADDRESS_WRITE;
			WHEN TOP_WR_REG_ADDR =>
				DATA_REALWRITE<= WORD_REG_ADDRESS;
			WHEN TOP_RD_REG_ADDR =>
				DATA_REALWRITE<= WORD_REG_ADDRESS;
			WHEN TOP_WR_DATA =>
				DATA_REALWRITE<= "0000"&DATA_CHANGED;
			WHEN OTHERS =>
				DATA_REALWRITE<="11111111";
		END CASE;
	END IF;
END IF;	
END PROCESS;





TOP_nxt_state: PROCESS(CLK_100K,RESET,ack_check_in,I2CCONTROLSTATE)
	variable  nxt_state 		: STATETYPE;

BEGIN
		nxt_state 	:= I2CCONTROLSTATE;
		CASE nxt_state IS
			
			WHEN IDLE			=>
			TOP_STATE<="0000";

				IF (WRITE_H = '1') THEN
					nxt_state 	:= TOP_WR_DEV_ADDR;
				ELSIF (READ_H = '1') THEN
					nxt_state 	:= TOP_RD_DEV_ADDR;
				ELSE 
					nxt_state := IDLE;
				END IF;

			WHEN TOP_WR_DEV_ADDR =>
				--iCMD_START :='1';
				--iDATA_REALWRITE:= DEVICE_ADDRESS_WRITE;
				TOP_STATE<="0001";
				IF(ACK_CHECK_IN='1' AND irxack ='1') THEN
					nxt_state 	:= TOP_WR_ERR;
				ELSIF (ACK_CHECK_IN='1') THEN
					nxt_state 	:= TOP_WR_REG_ADDR;
				ELSE
				 	nxt_state 	:= TOP_WR_DEV_ADDR;
				END IF;
			
			WHEN TOP_WR_REG_ADDR =>
				--iCMD_WRITE:='1';
				--iDATA_REALWRITE:=WORD_REG_ADDRESS ;
				TOP_STATE<="0010";
				IF (ACK_CHECK_IN='1') THEN
					nxt_state 	:= TOP_WR_DATA;
				ELSE
				 	nxt_state 	:= TOP_WR_REG_ADDR;
				END IF;

			WHEN TOP_WR_DATA =>
				--iCMD_WRITE:='1';
				--iDATA_REALWRITE:="0000"&DATA_CHANGED ;
				TOP_STATE<="0011";
				IF (ACK_CHECK_IN='1') THEN
					nxt_state 	:= TOP_WR_STOP;
				ELSE
				 	nxt_state 	:= TOP_WR_DATA;
				END IF;

			WHEN TOP_WR_STOP =>
			TOP_STATE<="0100";
				--iCMD_STOP:='1';
				IF (ACK_CHECK_IN='1') THEN
					nxt_state 	:=	TOP_WR_ACK;
				ELSE 
					nxt_state 	:=	TOP_WR_STOP;
				END IF;

			WHEN TOP_RD_DEV_ADDR =>
				--iCMD_START:='1';
				--iDATA_REALWRITE:=DEVICE_ADDRESS_WRITE;
				TOP_STATE<="0101";
				IF(ACK_CHECK_IN='1' AND irxack ='1') THEN
					nxt_state 	:= TOP_WR_ERR;
				ELSIF (ACK_CHECK_IN='1') THEN
					nxt_state 	:= TOP_RD_REG_ADDR;
				ELSE
				 	nxt_state 	:= TOP_RD_DEV_ADDR;
				END IF;



		    WHEN TOP_RD_REG_ADDR =>
		    TOP_STATE<="0110";
				--iCMD_WRITE:='1';
				--iDATA_REALWRITE:=WORD_REG_ADDRESS ;
				
				IF (ACK_CHECK_IN='1') THEN
					nxt_state 	:= TOP_RD_DEV_ADDR1;
				ELSE
				 	nxt_state 	:= TOP_RD_REG_ADDR;
				END IF;

			 WHEN TOP_RD_DEV_ADDR1 =>
			 TOP_STATE<="0111";
				--iCMD_WRITE:='1';
				--iDATA_REALWRITE:=DEVICE_ADDRESS_READ ;
				
				IF (ACK_CHECK_IN='1') THEN
					nxt_state 	:= TOP_RD_DATA;
				ELSE
				 	nxt_state 	:= TOP_RD_DEV_ADDR1;
				END IF;

			WHEN TOP_RD_DATA =>
			TOP_STATE<="1000";
				--iCMD_READ:='1';
				IF (ACK_CHECK_IN='1') THEN
					--iDATA_READ 	:= top_Dout;
					nxt_state 	:= TOP_RD_STOP;
				ELSE
				 	nxt_state 	:= TOP_RD_DATA;
				END IF;

			WHEN TOP_RD_STOP =>
			TOP_STATE<="1001";
				----iCMD_STOP:='1';
				IF (ACK_CHECK_IN='1') THEN
					nxt_state 	:= TOP_RD_ACK;
				ELSE
				 	nxt_state 	:= TOP_RD_STOP;
				END IF;

			WHEN TOP_RD_ACK =>
			TOP_STATE<="1010";
				--RD_DOWN<='1';

					nxt_state 	:= TOP_WAIT;



			WHEN TOP_WR_ACK =>
			TOP_STATE<="1011";
					--WR_DOWN<='1';

					nxt_state 	:= TOP_WAIT;


			WHEN TOP_WR_ERR =>
			TOP_STATE<="1100";
				nxt_state 	:= TOP_WR_STOP;

			WHEN TOP_WAIT =>
			TOP_STATE<="1101";	
				nxt_state 	:= IDLE;
			WHEN OTHERS =>
			TOP_STATE<="1111";
				nxt_state 	:=IDLE;
		END CASE;	

	IF (RESET='0') THEN 
		I2CCONTROLSTATE<=IDLE;
		TOP_STATE<="0000";

	ELSIF (CLK_100K'EVENT AND CLK_100K= '1') THEN
		I2CCONTROLSTATE <= nxt_state;
	
	END IF;

	END PROCESS TOP_nxt_state;

end block topstatemachine;

END ARCHITECTURE structural;

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;

entity simple_i2c is
	port (
		clk : in std_logic;
		ena : in std_logic;
		nReset : in std_logic;



		-- input signals
		start,
		stop,
		read,
		write,
		ack_in : std_logic;
		Din : in std_logic_vector(7 downto 0);

		-- output signals
		cmd_ack : out std_logic;
		ack_out : out std_logic;
		Dout : out std_logic_vector(7 downto 0);
		simple_STATE_POINT :OUT STD_LOGIC_VECTOR (3 DOWNTO 0);
		core_state:out std_logic_vector(2 downto 0);

		-- i2c signals
		SCL : inout std_logic;
		SDA : inout std_logic
	);
end entity simple_i2c;

architecture structural of simple_i2c is
	component i2c_core is
	port (
		clk : in std_logic;
		nReset : in std_logic;

		cmd : in std_logic_vector(2 downto 0);
		cmd_ack : out std_logic;

		Din : in std_logic;
		Dout : out std_logic;


		SCL : inout std_logic;
		SDA : inout std_logic
	);
	end component i2c_core;

	-- commands for i2c_core
	constant CMD_NOP	: std_logic_vector(2 downto 0) := "000";
	constant CMD_START	: std_logic_vector(2 downto 0) := "010";
	constant CMD_STOP	: std_logic_vector(2 downto 0) := "011";
	constant CMD_READ	: std_logic_vector(2 downto 0) := "100";
	constant CMD_WRITE	: std_logic_vector(2 downto 0) := "101";

	-- signals for i2c_core
	signal core_cmd : std_logic_vector(2 downto 0);
	signal core_ack, core_txd, core_rxd : std_logic;

	-- signals for shift register
	signal sr : std_logic_vector(7 downto 0); -- 8bit shift register

	signal shift, ld : std_logic;

	-- signals for state machine
	signal go, host_ack : std_logic;
begin
		core_state<=core_cmd;
	-- hookup i2c core
	u1: i2c_core port map (clk=>clk, 
						nReset=>nReset, 
						cmd=>core_cmd, 
						cmd_ack=>core_ack, 
						Din=>core_txd, 
						Dout=>core_rxd, 
						SCL=>SCL, 
						SDA=>SDA);

	-- generate host-command-acknowledge
	cmd_ack <= host_ack;
	
	-- generate go-signal
	go <= (read or write OR STOP ) and not host_ack;

	-- assign Dout output to shift-register
	Dout <= sr;

	-- assign ack_out output to core_rxd (contains last received bit)
	ack_out <= core_rxd;

	-- generate shift register
	shift_register: process(clk)
	begin
		if (clk'event and clk = '1') then
			if (nReset = '0') then
				sr <="00000000";
			ELSIF (ld = '1') then
				sr <= din;
			elsif (shift = '1') then
				sr <= (sr(6 downto 0) & core_rxd);
			end if;
		end if;
	end process shift_register;




	-- state machine
	--
	statemachine : block
		type states is (st_idle, st_start, st_read, st_write, st_ack, st_stop);
		signal state : states;
		signal dcnt : unsigned(2 downto 0);
	begin
		--
		-- command interpreter, translate complex commands into simpler I2C commands

			shift_counter: process(clk)
	begin
		if (clk'event and clk='1') then
			if (nReset = '0') then
				dcnt <= "000";
			ELSIF (ld = '1') then
				dcnt <= "111";
			elsif (shift = '1') then
				dcnt <= dcnt -1;
			end if;
		end if;
	end process shift_counter;

		nxt_state_decoder: process(clk, nReset, state)
			variable nxt_state : states;

			variable ihost_ack : std_logic;
			variable icore_cmd : std_logic_vector(2 downto 0);
			variable icore_txd : std_logic;
			variable ishift, iload : std_logic;
			VARIABLE iSTATE_POINT	:STD_LOGIC_VECTOR(3 DOWNTO 0);
			VARIABLE iDOUT		:STD_LOGIC_VECTOR(7 DOWNTO 0);
		begin
			-- 8 databits (1byte) of data to shift-in/out


			-- no acknowledge (until command complete)
			ihost_ack := '0';

			icore_txd := core_txd;

			-- keep current command to i2c_core
			icore_cmd := core_cmd;

			-- no shifting or loading of shift-register
			ishift := '0';
			iload  := '0';
			iSTATE_POINT :="0000";

			-- keep current display;
--			iDout  :=Dout_temp;

			-- keep current state;
			nxt_state := state;
			case state is
				when st_idle =>
				iSTATE_POINT :="0000";
					if (go = '1') then
						if (start = '1') then
							nxt_state := st_start;	
							icore_cmd := CMD_START;
						elsif (read = '1') then
							nxt_state := st_read;
							icore_cmd := CMD_READ;

						elsif(write='1') then
							nxt_state := st_write;
							icore_cmd := CMD_WRITE;

							
						else
							nxt_state := st_stop;
							icore_cmd := CMD_STOP;
						end if;

						iload := '1';
					end if;

				when st_start =>
					iSTATE_POINT :="0001";

					if (core_ack = '1') then
						if (read = '1') then
							nxt_state := st_read;
							icore_cmd := CMD_READ;
						else
							nxt_state := st_write;
							icore_cmd := CMD_WRITE;
						end if;

						iload := '1';
					end if;

				when st_write =>
				iSTATE_POINT :="0010";
					if (core_ack = '1') then

						if (dcnt = 0) then
							nxt_state := st_ack;
							icore_cmd := CMD_READ;
						else
							nxt_state :=st_write;
							icore_cmd := CMD_WRITE;
							ishift := '1';
--							
						end if;
						

					end if;			

				when st_read =>
					iSTATE_POINT :="0011";
					if (core_ack = '1') then				
						if (dcnt = 0) then
							
							nxt_state := st_ack;
							icore_cmd := CMD_WRITE;
							
						else
							nxt_state := st_read;
							icore_cmd := CMD_READ;
						end if;
						ishift := '1';
						icore_txd := ack_in;
					end if;			

				when st_ack =>
				iSTATE_POINT :="0100";
					if (core_ack = '1') then
						if (stop = '1') then
							nxt_state := st_stop;
							icore_cmd := CMD_STOP;
						else
							nxt_state := st_idle;
							icore_cmd := CMD_NOP;
							ihost_ack := '1';
						end if;
						icore_txd := ack_in;
					end if;

				when st_stop =>
				iSTATE_POINT :="1000";
					if (core_ack = '1') then
						ihost_ack := '1';-- generate command acknowledge signal
						nxt_state := st_idle;
						icore_cmd := CMD_NOP;
					end if;

				when others => -- illegal states
				iSTATE_POINT :="1111";
					nxt_state := st_idle;
					icore_cmd := CMD_NOP;
			end case;

			-- generate registers
			if (nReset = '0') then
				core_cmd <= CMD_NOP;
				core_txd <= '0';
--				Dout_temp<="00000000";
				shift <= '0';
				ld <= '0';
				simple_STATE_POINT<="0000";

				host_ack <= '0';

				state <= st_idle;
			elsif (clk'event and clk = '1') then
				if (ena = '1') then
					state <= nxt_state;
--					Dout_temp <=iDout;
--					Dout<=Dout_temp;

					shift <= ishift;
					ld <= iload;

					core_cmd <= icore_cmd;
					core_txd <= icore_txd;
					simple_STATE_POINT<=iSTATE_POINT;
					host_ack <= ihost_ack;
				end if;
			end if;
		end process nxt_state_decoder;

	end block statemachine;

end architecture structural;


--
--
-- I2C Core
--
-- Translate simple commands into SCL/SDA transitions
-- Each command has 5 states, A/B/C/D/idle
--
-- start:	SCL	~~~~~~~~~~\____
--	SDA	~~~~~~~~\______
--		 x | A | B | C | D | i
--
-- repstart	SCL	____/~~~~\___
--	SDA	__/~~~\______
--		 x | A | B | C | D | i
--
-- stop	SCL	____/~~~~~~~~
--	SDA	==\____/~~~~~
--		 x | A | B | C | D | i
--
--- write	SCL	____/~~~~\____
--	SDA	==X=========X=
--		 x | A | B | C | D | i
--
--- read	SCL	____/~~~~\____
--	SDA	XXXX=====XXXX
--		 x | A | B | C | D | i
--

-- Timing:		Normal mode	Fast mode
-----------------------------------------------------------------
-- Fscl		100KHz		400KHz
-- Th_scl		4.0us		0.6us	High period of SCL
-- Tl_scl		4.7us		1.3us	Low period of SCL
-- Tsu:sta		4.7us		0.6us	setup time for a repeated start condition
-- Tsu:sto		4.0us		0.6us	setup time for a stop conditon
-- Tbuf		4.7us		1.3us	Bus free time between a stop and start condition
--

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_UNSIGNED.all;

entity i2c_core is
	port (
		clk : in std_logic;
		nReset : in std_logic;


		cmd : in std_logic_vector(2 downto 0);
		cmd_ack : out std_logic;


		Din : in std_logic;
		Dout : out std_logic;

		SCL : inout std_logic;
		SDA : inout std_logic
	);
end entity i2c_core;

architecture structural of i2c_core is
	constant CMD_NOP	: std_logic_vector(2 downto 0) := "000";
	constant CMD_START	: std_logic_vector(2 downto 0) := "010";
	constant CMD_STOP	: std_logic_vector(2 downto 0) := "011";
	constant CMD_READ	: std_logic_vector(2 downto 0) := "100";
	constant CMD_WRITE	: std_logic_vector(2 downto 0) := "101";

	type cmds is (idle, start_a, start_b, start_c, start_d, stop_a, stop_b, stop_c, rd_a, rd_b, rd_c, rd_d, wr_a, wr_b, wr_c, wr_d);
	signal state : cmds;
	signal SDAo, SCLo : std_logic;

	signal clk_en, slave_wait :std_logic;
	signal cnt : unsigned(7 downto 0) := "00000000";
begin
	-- whenever the slave is not ready it can delay the cycle by pulling SCL low
	slave_wait <= '1' when ((SCLo = '1') and (SCL = '0')) else '0';

	-- generate clk enable signal
	gen_clken: process(clk, nReset)
	begin
		if (nReset = '0') then
			cnt <= (others => '0');
			clk_en <= '1'; --'0';
		elsif (clk'event and clk = '1') then
			if (cnt = 0) then
				clk_en <= '1';
				cnt <= "00000000";

			else
				if (slave_wait = '0') then
					cnt <= cnt -1;
				end if;
				clk_en <= '0';
			end if;
		end if;
	end process gen_clken;

	-- generate statemachine
	nxt_state_decoder : process (clk ,nReset, state, cmd, SDA)
		variable nxt_state : cmds;
		variable icmd_ack, store_sda : std_logic;

	begin

		nxt_state := state;

		icmd_ack := '0'; -- default no acknowledge

		store_sda := '0';



		case (state) is
			-- idle
			when idle =>
				case cmd is
					when CMD_START =>
						nxt_state := start_a;
						icmd_ack := '1'; -- command completed

					when CMD_STOP =>
						nxt_state := stop_a;
						icmd_ack := '1'; -- command completed

					when CMD_WRITE =>
						nxt_state := wr_a;
						icmd_ack := '1'; -- command completed


					when CMD_READ =>
						nxt_state := rd_a;
						icmd_ack := '1'; -- command completed

					when others =>
						nxt_state := idle;
-- don't acknowledge NOP command						icmd_ack := '1'; -- command completed

				end case;

			-- start
			when start_a =>
				nxt_state := start_b;

			when start_b =>
				nxt_state := start_c;

			when start_c =>
				nxt_state := start_d;

			when start_d =>
				nxt_state := idle;



			-- stop
			when stop_a =>
				nxt_state := stop_b;

			when stop_b =>
				nxt_state := stop_c;

			when stop_c =>
--				nxt_state := stop_d;

--			when stop_d =>
				nxt_state := idle;


			-- read
			when rd_a =>
				nxt_state := rd_b;

			when rd_b =>
				nxt_state := rd_c;

			when rd_c =>
				nxt_state := rd_d;
				store_sda := '1';

			when rd_d =>
				nxt_state := idle;


			-- write
			when wr_a =>
				nxt_state := wr_b;

			when wr_b =>
				nxt_state := wr_c;

			when wr_c =>
				nxt_state := wr_d;

			when wr_d =>
				nxt_state := idle;


		end case;

		-- generate regs
		if (nReset = '0') then
			state <= idle;
			cmd_ack <= '0';

			Dout <= '0';
		elsif (clk'event and clk = '1') then
			if (clk_en = '1') then
				state <= nxt_state;



				if (store_sda = '1') then
					Dout <= SDA;
				end if;
			end if;

			cmd_ack <= icmd_ack and clk_en;
		end if;
	end process nxt_state_decoder;

	--
	-- convert states to SCL and SDA signals
	--
	output_decoder: process (clk, nReset, state)
		variable iscl, isda : std_logic;
	begin
		case (state) is
			when idle =>
				iscl := SCLo; -- keep SCL in same state
				isda := SDA; -- keep SDA in same state

			-- start
			when start_a =>
				iscl := SCLo; -- keep SCL in same state (for repeated start)
				isda := '1'; -- set SDA high

			when start_b =>
				iscl := '1';	-- set SCL high
				isda := '1'; -- keep SDA high

			when start_c =>
				iscl := '1';	-- keep SCL high
				isda := '0'; -- sel SDA low

			when start_d =>
				iscl := '0'; -- set SCL low
				isda := '0'; -- keep SDA low

			-- stop
			when stop_a =>
				iscl := '0'; -- keep SCL disabled
				isda := '0'; -- set SDA low

			when stop_b =>
				iscl := '1'; -- set SCL high
				isda := '0'; -- keep SDA low

			when stop_c =>
				iscl := '1'; -- keep SCL high
				isda := '1'; -- set SDA high

			-- write
			when wr_a =>
				iscl := '0';	-- keep SCL low

				isda := Din;

			when wr_b =>
				iscl := '1';	-- set SCL high

				isda := Din;

			when wr_c =>
				iscl := '1';	-- keep SCL high
--				isda := txd; -- set SDA
				isda := Din;

			when wr_d =>
				iscl := '0'; -- set SCL low
--				isda := txd; -- set SDA
				isda := Din;

			-- read
			when rd_a =>
				iscl := '0'; -- keep SCL low
				isda := '1'; -- tri-state SDA

			when rd_b =>
				iscl := '1'; -- set SCL high
				isda := '1'; -- tri-state SDA

			when rd_c =>
				iscl := '1'; -- keep SCL high
				isda := '1'; -- tri-state SDA

			when rd_d =>
				iscl := '0'; -- set SCL low
				isda := '1'; -- tri-state SDA
		end case;

		-- generate registers
		if (nReset = '0') then
			SCLo <= '1';
			SDAo <= '1';
		elsif (clk'event and clk = '1') then
			if (clk_en = '1') then
				SCLo <= iscl;
				SDAo <= isda;
			end if;
		end if;
	end process output_decoder;

	SCL <= '0' when (SCLo = '0') else 'Z'; -- since SCL is externally pulled-up convert a '1' to a 'Z'(tri-state)
	SDA <= '0' when (SDAo = '0') else 'Z'; -- since SDA is externally pulled-up convert a '1' to a 'Z'(tri-state)
--	SCL <= SCLo;
--	SDA <= SDAo;

end architecture structural;
