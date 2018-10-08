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
--

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
use IEEE.MATH_REAL.ALL;

library unisim;
use unisim.vcomponents.all;

use work.f32c_pack.all;

entity f32_papilio_main is
  Port(
    CLK_CRYSTAL     : in    std_logic;
    RX              : in    std_logic;
    TX              : out   std_logic;

    --A( 7 downto  0)
    SIMPLE_IN       : in    std_logic_vector(7 downto 0);
    --A(15 downto  8)
    SIMPLE_OUT      : out   std_logic_vector(7 downto 0);

    B               : inout std_logic_vector(15 downto 0);
    C               : inout std_logic_vector(11 downto 0);

    --C(15 downto 12 - SCK, SS, MOSI, MISO)
    SPI_SCK         : out   std_logic;
    SPI_SS          : out   std_logic;
    SPI_MOSI        : out   std_logic;
    SPI_MISO        : in    std_logic;

    FLASH_CS        : out   std_logic;
    FLASH_CK        : out   std_logic;
    FLASH_SI        : out   std_logic;
    FLASH_SO        : in    std_logic;

    SDRAM_ADDR      : out std_logic_vector(12 downto 0);
    SDRAM_DATA      : inout std_logic_vector(15 downto 0);
    SDRAM_DQM       : out std_logic_vector(1 downto 0);
    SDRAM_BA        : out std_logic_vector(1 downto 0);
    SDRAM_nWE       : out std_logic;
    SDRAM_nCAS      : out std_logic;
    SDRAM_nRAS      : out std_logic;
    SDRAM_CS        : out std_logic;
    SDRAM_CLK       : out std_logic;
    SDRAM_CKE       : out std_logic;

    LED1            : out   std_logic
  );
end f32_papilio_main;

architecture Behavioral of f32_papilio_main is

component papilio_xram_bram is
  generic
  (
    -- ISA: either ARCH_MI32 or ARCH_RV32
    C_arch                      : integer := ARCH_MI32;
    C_debug                     : boolean := false;

    -- Main clock: 100
    C_clk_freq                  : integer := 100;
    C_vendor_specific_startup   : boolean := false; -- false: disabled (xilinx startup doesn't work reliable on this board - check this)
    -- SoC configuration options
    C_bram_size                 : integer := 4 -- bootloader area
  );
  port
  (
    clk_32MHz                   : in    std_logic;

    rs232_tx                    : out   std_logic;
    rs232_rx                    : in    std_logic;

    spi_sck                     : out   std_logic_vector(2 downto 0);
    spi_ss                      : out   std_logic_vector(2 downto 0);
    spi_mosi                    : out   std_logic_vector(2 downto 0);
    spi_miso                    : in    std_logic_vector(2 downto 0);
    --user LED on Papilio
    led                         : out   std_logic;
    --7 simple in for Papilio pins
    simple_in                   : in    std_logic_vector(7 downto 0);
    --7 simple out for Papilio pins
    simple_out                  : out   std_logic_vector(7 downto 0);
    --GPIO
    gpio                        : inout std_logic_vector(27 downto 0);
    --SDRAM
    sdram_addr                  : out   std_logic_vector(12 downto 0);
    sdram_data                  : inout std_logic_vector(15 downto 0);
    sdram_ba                    : out   std_logic_vector(1 downto 0);
    sdram_dqm                   : out   std_logic_vector(1 downto 0);
    sdram_ras                   : out   std_logic;
    sdram_cas                   : out   std_logic;
    sdram_clk                   : out   std_logic;
    sdram_cke                   : out   std_logic;
    sdram_we                    : out   std_logic;
    sdram_cs                    : out   std_logic
  );
end component;

begin

inst_f32c : papilio_xram_bram
  port map(
    clk_32MHz           => CLK_CRYSTAL,

    rs232_tx            => TX,
    rs232_rx            => RX,

    simple_in           => SIMPLE_IN,
    simple_out          => SIMPLE_OUT,

    -- 0    flash memory SPI interface
    -- 1    no SD card connected
    -- 2    user SPI channel
    spi_sck(0)          => FLASH_CK,
    spi_sck(1)          => open,
    spi_sck(2)          => SPI_SCK,

    spi_ss(0)           => FLASH_CS,
    spi_ss(1)           => open,
    spi_ss(2)           => SPI_SS,

    spi_mosi(0)         => FLASH_SI,
    spi_mosi(1)         => open,
    spi_mosi(2)         => SPI_MOSI,

    spi_miso(0)         => FLASH_SO,
    spi_miso(1)         => '0',
    spi_miso(2)         => SPI_MISO,

    LED                 => LED1,
    GPIO(15 downto 0)   => B,
    GPIO(27 downto 16)  => C,

    sdram_addr          => SDRAM_ADDR,
    sdram_data          => SDRAM_DATA,
    sdram_ba            => SDRAM_BA,
    sdram_dqm           => SDRAM_DQM,
    sdram_ras           => SDRAM_nRAS,
    sdram_cas           => SDRAM_nCAS,
    sdram_clk           => SDRAM_CLK,
    sdram_cke           => SDRAM_CKE,
    sdram_we            => SDRAM_nWE,
    sdram_cs            => SDRAM_CS
  );

end Behavioral;

