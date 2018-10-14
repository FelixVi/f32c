--
-- Copyright (c) 2018 Felix Vietmeyer
-- All rights reserved.
--
-- Redistribution and use in source and binary forms, with or without
-- modification, are permitted provided that the following conditions
-- are met:
-- 1. Redistributions of source code must retain the above copyright
--    notice, this list of conditions and the following disclaimer.
-- 2. Redistributions in binary form must reproduce the above copyright
--    notice, this list of conditions and the following disclaimer in the
--    documentation and/or other materials provided with the distribution.
--
-- THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
-- ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
-- IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
-- ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
-- FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
-- DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
-- OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
-- HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
-- LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
-- OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
-- SUCH DAMAGE.

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use ieee.numeric_std.all;

use work.sram_pack.all;

entity bridge_spi_sdram is
  port
  (
    clk         : in  std_logic;
    reset       : in  std_logic;

    -- SPI interface
    spi_sck     : in std_logic;
    spi_mosi    : in std_logic;
    spi_cen     : in std_logic;
    spi_miso    : out std_logic;

    rfd         : out std_logic;
    data_rdy    : out std_logic;

    -- SDRAM port connection
    sdram_data_out       : in std_logic_vector(31 downto 0);
    sdram_ready_out      : in std_logic;
    sdram_bus_in         : out sram_port_type
  );
end bridge_spi_sdram;

-- Here, we are doing transaction that are either 32 or 64-bit long
-- The format is the following:
-- XXXX YYYY YYYY YYYY YYYY YYYY YYYY YYYY
-- where X is an instruction for the memory controller and
--       Y is an address (in our first test case, actual length is 21 bit)
-- If X indicates a write operation, data is provided by the following bits
-- ZZZZ ZZZZ ZZZZ ZZZZ ZZZZ ZZZZ ZZZZ ZZZZ
-- Where Z is the data to be transferred
-- We are doing all transactions with 32-bit granularity, no byte select
-- signals are provided
--
-- If a read transaction is carried out, this controller issues the data_rdy
-- signal once data has been retrieved from SDRAM. It can then be read out.
--
-- Note that reads and writes are concurrent, so read data from SDRAM will
-- be transferred on the miso line even when a write command is issued
-- via mosi.
--
-- Instructions for the controller:
-- 0000 NOP (use when reading only)
-- 0001 READ
-- 0010 WRITE
-- further bits may be used later to implement burst reads and writes

architecture Behavioral of bridge_spi_sdram is
    --SPI signals
    signal R_spi_sck_dly0       : std_logic;    -- previous spi_sck state
    signal R_spi_mosi_dly0      : std_logic;    -- previous spi_mosi state
    signal spi_sck_rise         : std_logic;    -- spi_sck rising edge
    signal spi_sck_fall         : std_logic;    -- spi_sck falling edge

    --SPI data handling
    signal R_bit_cnt            : std_logic_vector( 5 downto 0) := (others => '0');
    signal R_inst               : std_logic_vector(31 downto 0);
    signal R_data               : std_logic_vector(31 downto 0);
    signal R_data_from_mem      : std_logic_vector(31 downto 0);
    signal R_data_flag          : std_logic := '0'; --1 when data is coming in
    
    signal R_data_trans         : boolean := false;
    signal R_word_received      : boolean := false;
    signal R_data_rdy           : std_logic := '0';
    
    --memory FSM
    type t_mem_state is (
        ST_IDLE,
        ST_READ,
        ST_DATA,
        ST_WRITE,
        ST_WWAIT,
        ST_RWAIT
    );
    signal pr_mem_state, nx_mem_state: t_mem_state;
    
begin

    --FSM state register
    process(clk, reset)
    begin
        if reset = '1' then
            pr_mem_state <= ST_IDLE;
        elsif rising_edge(clk) then
            pr_mem_state <= nx_mem_state;
        end if;
    end process;

    -- register spi_sck for edge detection and spi_mosi for read
    spi_edge : process (clk)
    begin
        if rising_edge(clk) then
            if reset = '1' then
                R_spi_sck_dly0  <= '0';
                R_spi_mosi_dly0 <= '0';
            else
                R_spi_sck_dly0  <= spi_sck;
                R_spi_mosi_dly0 <= spi_mosi;
            end if;
        end if;
    end process;

    --flage indicating rising/falling edges on sck
    spi_sck_rise <= not spi_sck and R_spi_sck_dly0;
    spi_sck_fall <= spi_sck and not R_spi_sck_dly0;
    
    --data output
    spi_miso <= R_data_from_mem(31);
    
    -- reading data coming in and placing it into instruction and data registers
    mosi_in : process (clk, spi_cen, reset)
    begin
        if spi_cen = '1' or reset = '1' then
            R_bit_cnt           <= (others => '0');
        elsif rising_edge(clk) then
            R_word_received <= false;
            if R_bit_cnt(5) = '1' then --32-bits received
                R_bit_cnt <= (others => '0');
                R_word_received <= true;
            end if;
            
            if spi_sck_rise = '1' then
                if pr_mem_state = ST_IDLE then
                    R_inst <= R_inst(30 downto 0) & R_spi_mosi_dly0;
                    R_data_from_mem <= R_data_from_mem(30 downto 0) & '-';
                elsif pr_mem_state = ST_DATA then
                    R_data <= R_data(30 downto 0) & R_spi_mosi_dly0;
                end if;
            end if;
            
            if spi_sck_fall = '1' then
                R_bit_cnt <= R_bit_cnt + 1;
            end if;
            
            --update data from memory
            if pr_mem_state = ST_RWAIT or pr_mem_state = ST_WWAIT then
                if sdram_ready_out = '1' then
                    R_data_from_mem <= sdram_data_out;
                end if;
            end if;
        end if;
    end process;
    
    --memory FSM state logic
    --FSM combinational logic
    process(nx_mem_state, pr_mem_state, R_word_received, sdram_ready_out, R_inst, R_data_trans)
    begin
        nx_mem_state <= pr_mem_state;
        case pr_mem_state is
            when ST_IDLE =>
                if R_word_received then
                    case R_inst(31 downto 28) is
                        when "0001" => --read
                            nx_mem_state <= ST_READ;
                        when "0010" => --write
                            nx_mem_state <= ST_DATA;
                        when others =>
                            null;
                    end case;
                end if;
            when ST_READ =>
                nx_mem_state <= ST_RWAIT;
            when ST_DATA =>
                if R_word_received and R_data_trans then
                    nx_mem_state <= ST_WRITE;
                end if;
            when ST_WRITE =>
                nx_mem_state <= ST_WWAIT;
            when ST_RWAIT | ST_WWAIT =>
                if sdram_ready_out = '1' then
                    nx_mem_state <= ST_IDLE;
                end if;
            when others =>
                nx_mem_state <= ST_IDLE;
        end case;
    end process;
    
    -- SDRAM synchronous
    mem_sync : process (clk)
    begin
        if rising_edge(clk) then
            if reset = '1' then
                R_data_rdy <= '0';
            end if;
            
            if pr_mem_state = ST_IDLE then
                R_data_trans <= false;
            elsif pr_mem_state = ST_DATA and R_bit_cnt(5) = '0' then
                R_data_trans <= true;
            end if;
            if pr_mem_state = ST_RWAIT and sdram_ready_out = '1' then
                R_data_rdy     <= '1';
            end if;
            if pr_mem_state = ST_DATA or pr_mem_state = ST_READ then
                R_data_rdy     <= '0';
            end if;
        end if;
    end process;

    -- SDRAM asynchronous
    mem_async : process (pr_mem_state)
    begin
        sdram_bus_in.write       <= '0';
        sdram_bus_in.addr_strobe <= '0';
        
        if pr_mem_state = ST_WRITE then
            sdram_bus_in.write       <= '1';
        end if;
        
        if pr_mem_state = ST_READ or pr_mem_state = ST_WRITE or pr_mem_state = ST_RWAIT or pr_mem_state = ST_WWAIT then
            --hold strobe until transaction is done
            sdram_bus_in.addr_strobe <= '1';
        end if;
        
    end process;
    
    --                                          12-bit       2-bit    7-bit  => 21-bit address space for 32-bit words
    --                          "0000000"        ROW         BANK      COL
    sdram_bus_in.addr        <= "0000000" & R_inst(20 downto 0);
    --                                COL is one bit less as we write in 2 16-bit bursts to transfer one 32-bit word
    sdram_bus_in.data_in     <= R_data;
    sdram_bus_in.byte_sel    <= (others => '1');
    
    --ready for data signal
    process(pr_mem_state)
    begin
        rfd <= '0';
        if pr_mem_state = ST_IDLE then
            rfd <= '1';
        end if;
    end process;
    
    data_rdy <= R_data_rdy;
end Behavioral;
