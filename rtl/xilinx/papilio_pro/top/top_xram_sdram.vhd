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
use IEEE.MATH_REAL.ALL; -- floor(), log2()

library unisim;
use unisim.vcomponents.all;

use work.f32c_pack.all;

entity papilio_xram_bram is
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
end papilio_xram_bram;

architecture Behavioral of papilio_xram_bram is
  signal cpu_clk     : std_logic;
  signal sdram_clk_p : std_logic;
  signal sdram_clk_n : std_logic;
  signal rs232_break : std_logic;
begin
  -- clock synthesizer: Xilinx Spartan-6 specific
  clk100: if C_clk_freq = 100 generate
    clkgen: entity work.xil_pll
      generic map(
        clk_in_period_ns    => 31.250,  --32 MHz
        clk_mult            => 25,      --fVCO = 800 MHz
        clk_diva            => 8,
        clk_phasea          => 0.0,
        clk_divb            => 8,
        clk_phaseb          => 180.0,
        clk_divc            => 8,
        clk_phasec          => 0.0
      )
      port map(
        clk_in              => clk_32MHz,
        clk_outa            => sdram_clk_p,
        clk_outb            => sdram_clk_n,
        clk_outc            => cpu_clk
      );
  end generate;

  G_vendor_specific_startup: if C_vendor_specific_startup generate
  -- reset hard-block: Xilinx Spartan-6 specific
  reset: startup_spartan6
    port map
    (
      clk       => cpu_clk,
      gsr       => rs232_break,
      gts       => rs232_break,
      keyclearb => '0'
    );
  end generate; -- G_vendor_specific_startup

  -- generic XRAM glue, listing options for clarity
  glue_xram: entity work.glue_xram
  generic map (
    --options configured in top
    C_arch                      => C_arch,
    C_debug                     => C_debug,
    C_clk_freq                  => C_clk_freq,
    C_bram_size                 => C_bram_size,
    --parameters we fix for this example
    C_sio                       => 1,
    C_spi                       => 3,
    C_gpio                      => 28,

    C_sdram                     => true,
    C_sdram_clock_range         => 2,
    C_sdram_address_width       => 22,
    C_sdram_column_bits         => 8,
    C_sdram_startup_cycles      => 10100,
    C_sdram_cycles_per_refresh  => (64000*100)/4096-1,
    --these settings reflext default settings in glue_xram
    -- ISA options
    C_big_endian                => false,
    C_mult_enable               => true,
    C_mul_acc                   => false,
    C_mul_reg                   => false,
    C_branch_likely             => true,
    C_sign_extend               => true,
    C_ll_sc                     => false,
    C_PC_mask                   => x"ffffffff",
    C_exceptions                => true,

    -- COP0 options
    C_cop0_count                => true,
    C_cop0_compare              => true,
    C_cop0_config               => true,

    -- CPU core configuration options
    C_branch_prediction         => true,
    C_full_shifter              => true,
    C_result_forwarding         => true,
    C_load_aligner              => true,
    C_regfile_synchronous_read  => false,
    -- Negatively influences timing closure, hence disabled
    C_movn_movz                 => false,

    -- SoC configuration options
    C_bram_const_init           => true,
    C_boot_write_protect        => true,
    C_boot_spi                  => false,
    C_icache_size               => 0,
    C_dcache_size               => 0,
    C_xram_base                 => X"8",
    C_cached_addr_bits          => 20,

    C_sio_init_baudrate         => 115200,
    C_sio_fixed_baudrate        => false,
    C_sio_break_detect          => true,

    C_spi_turbo_mode            => "0000",
    C_spi_fixed_speed           => "1111",

    C_simple_in                 => 32,
    C_simple_out                => 32,

    C_timer                     => true,
    C_timer_ocp_mux             => true,
    C_timer_ocps                => 2,
    C_timer_icps                => 2,
    --these settings turn off unused features in glue_xram
    C_xdma                      => false,
    C_sram                      => false,
    C_acram                     => false,
    C_axiram                    => false,
    C_tv                        => false,
    C_vgahdmi                   => false,
    C_ledstrip                  => false,
    C_vgatext                   => false,
    C_pcm                       => false,
    C_synth                     => false,
    C_spdif                     => false,
    C_cw_simple_out             => -1,
    C_fmrds                     => false,
    C_gpio_adc                  => 0,
    C_pids                      => 0,
    C_vector                    => false
  )
  port map(
    clk                         => cpu_clk,
    clk_sdram                   => sdram_clk_p,
    sio_txd(0)                  => rs232_tx,
    sio_rxd(0)                  => rs232_rx,
    sio_break(0)                => rs232_break,
    spi_sck                     => spi_sck,
    spi_ss                      => spi_ss,
    spi_mosi                    => spi_mosi,
    spi_miso                    => spi_miso,
    simple_in(31 downto 8)      => (others => '0'),
    simple_in( 7 downto 0)      => simple_in,
    simple_out(31 downto 17)    => open,
    simple_out(16)              => led,
    simple_out(15 downto  8)    => open,
    simple_out( 7 downto  0)    => simple_out,
    gpio(127 downto 28)         => open,
    gpio(27 downto 0)           => gpio(27 downto 0),
    -- sdram
    sdram_addr                  => sdram_addr,
    sdram_data                  => sdram_data,
    sdram_ba                    => sdram_ba,
    sdram_dqm                   => sdram_dqm,
    sdram_ras                   => sdram_ras,
    sdram_cas                   => sdram_cas,
    sdram_cke                   => sdram_cke,
    sdram_clk                   => open,
    sdram_we                    => sdram_we,
    sdram_cs                    => sdram_cs
  );

  --clock out for SDRAM
  oddr2_SDRAM_clk : ODDR2
  generic map(
    DDR_ALIGNMENT   => "NONE",
    INIT            => '0',
    SRTYPE          => "SYNC"
  )
  port map(
    Q   => sdram_clk,
    C0  => sdram_clk_p,
    C1  => sdram_clk_n,
    CE  => '1',
    D0  => '0',
    D1  => '1',
    R   => '0',
    S   => '0'
  );

end Behavioral;
