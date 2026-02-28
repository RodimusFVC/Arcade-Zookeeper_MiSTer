--===========================================================================--
--
-- S Y N T H E Z I A B L E CPU68 C O R E
--
-- www.OpenCores.Org - December 2002
-- This core adheres to the GNU public license 
--
-- File name : cpu68.vhd
--
-- Purpose : Implements a 6800 compatible CPU core with some
-- additional instructions found in the 6801
-- 
-- Dependencies : ieee.Std_Logic_1164
-- ieee.std_logic_unsigned
--
-- Author : John E. Kent
-- Extended by : Rodimus 
--
--===========================================================================----
--
-- Revision History:
--
-- Date: Revision Author
-- 22 Sep 2002 0.1 John Kent
--
-- 30 Oct 2002 0.2 John Kent
-- made NMI edge triggered
--
-- 30 Oct 2002 0.3 John Kent
-- more corrections to NMI
-- added wai_wait_state to prevent stack overflow on wai.
--
-- 1 Nov 2002 0.4 John Kent
-- removed WAI states and integrated WAI with the interrupt service routine
-- replace Data out (do) and Data in (di) register with a single Memory Data (md) reg.
-- Added Multiply instruction states.
-- run ALU and CC out of CPU module for timing measurements.
--
-- 3 Nov 2002 0.5 John Kent
-- Memory Data Register was not loaded on Store instructions
-- SEV and CLV were not defined in the ALU
-- Overflow Flag on NEG was incorrect
--
-- 16th Feb 2003 0.6 John Kent
-- Rearranged the execution cycle for dual operand instructions
-- so that occurs during the following fetch cycle.
-- This allows the reduction of one clock cycle from dual operand
-- instruction. Note that this also necessitated re-arranging the
-- program counter so that it is no longer incremented in the ALU.
-- The effective address has also been re-arranged to include a
-- separate added. The STD (store accd) now sets the condition codes.
--
-- 28th Jun 2003 0.7 John Kent
-- Added Hold and Halt signals. Hold is used to steal cycles from the
-- CPU or add wait states. Halt puts the CPU in the inactive state
-- and is only honoured in the fetch cycle. Both signals are active high.
--
-- 9th Jan 2004 0.8 John Kent
-- Clear instruction did an alu_ld8 rather than an alu_clr, so
-- the carry bit was not cleared correctly.
-- This error was picked up by Michael Hassenfratz.
--
-- 2019 Jared Boone
-- Stall1_State1 & Stall2|_State added for better cycle accuracy.
--
-- March 2020 Gyorgy Szombathelyi
-- Runned through VHDLFormatter
-- Set I bit at reset
-- Set I bit on NMI
-- Fixes many Irem sound board issues, where NMI is constantly
-- issued by the ADPCM chip, and interrupted by normal IRQ
--
-- February 2026 Rodimus
-- Updated to support M6802 with internal 128-byte RAM.

library ieee;
use IEEE.STD_LOGIC_1164.all;
use IEEE.STD_LOGIC_ARITH.all;
use IEEE.STD_LOGIC_UNSIGNED.all;

entity cpu68 is
	port (
		clk : in std_logic;
		rst : in std_logic;
		rw : out std_logic;
		vma : out std_logic;
		address : out std_logic_vector(15 downto 0);
		data_in : in std_logic_vector(7 downto 0);
		data_out : out std_logic_vector(7 downto 0);
		hold : in std_logic;
		halt : in std_logic;
		irq : in std_logic;
		nmi : in std_logic;
		test_alu : out std_logic_vector(15 downto 0);
		test_cc : out std_logic_vector(7 downto 0)
	);
end;

architecture CPU_ARCH of cpu68 is

	constant SBIT : integer := 7;
	constant XBIT : integer := 6;
	constant HBIT : integer := 5;
	constant IBIT : integer := 4;
	constant NBIT : integer := 3;
	constant ZBIT : integer := 2;
	constant VBIT : integer := 1;
	constant CBIT : integer := 0;

	type state_type is (reset_state, fetch_state, decode_state, 
	extended_state, indexed_state, read8_state, read16_state, immediate16_state, 
	write8_state, write16_state, 
	execute_state, halt_state, error_state, 
	mul_state, mulea_state, muld_state, 
	mul0_state, mul1_state, mul2_state, mul3_state, 
	mul4_state, mul5_state, mul6_state, mul7_state, 
	jmp_state, jsr_state, jsr1_state, 
	branch_state, bsr_state, bsr1_state, 
	rts_hi_state, rts_lo_state, 
	int_pcl_state, int_pch_state, 
	int_ixl_state, int_ixh_state, 
	int_cc_state, int_acca_state, int_accb_state, 
	int_wai_state, int_mask_state, 
	rti_state, rti_cc_state, rti_acca_state, rti_accb_state, 
	rti_ixl_state, rti_ixh_state, 
	rti_pcl_state, rti_pch_state, 
	pula_state, psha_state, pulb_state, pshb_state, 
	pulx_lo_state, pulx_hi_state, pshx_lo_state, pshx_hi_state, 
	vect_lo_state, vect_hi_state, 
	stall1_state, stall2_state,
	stall1_write_state, stall2_write_state,
	stall1_write16_state, stall2_write16_state,
	stall1_idx_read_state,
	stall1_jsr_state, stall2_jsr_state, stall3_jsr_state,
	stall1_bsr_state, stall2_bsr_state);
	type addr_type is (idle_ad, fetch_ad, read_ad, write_ad, push_ad, pull_ad, int_hi_ad, int_lo_ad);
	type dout_type is (md_lo_dout, md_hi_dout, acca_dout, accb_dout, ix_lo_dout, ix_hi_dout, cc_dout, pc_lo_dout, pc_hi_dout);
	type op_type is (reset_op, fetch_op, latch_op);
	type acca_type is (reset_acca, load_acca, load_hi_acca, pull_acca, latch_acca);
	type accb_type is (reset_accb, load_accb, pull_accb, latch_accb);
	type cc_type is (reset_cc, load_cc, pull_cc, latch_cc);
	type ix_type is (reset_ix, load_ix, pull_lo_ix, pull_hi_ix, latch_ix);
	type sp_type is (reset_sp, latch_sp, load_sp);
	type pc_type is (reset_pc, latch_pc, load_ea_pc, add_ea_pc, pull_lo_pc, pull_hi_pc, inc_pc, load_ext_pc);
	type md_type is (reset_md, latch_md, load_md, fetch_first_md, fetch_next_md, shiftl_md);
	type ea_type is (reset_ea, latch_ea, add_ix_ea, load_accb_ea, inc_ea, fetch_first_ea, fetch_next_ea);
	type iv_type is (reset_iv, latch_iv, swi_iv, nmi_iv, irq_iv);
	type nmi_type is (reset_nmi, set_nmi, latch_nmi);
	type left_type is (acca_left, accb_left, accd_left, md_left, ix_left, sp_left);
	type right_type is (md_right, zero_right, plus_one_right, accb_right);
	type alu_type is (alu_add8, alu_sub8, alu_add16, alu_sub16, alu_adc, alu_sbc, 
	alu_and, alu_ora, alu_eor, 
	alu_tst, alu_inc, alu_dec, alu_clr, alu_neg, alu_com, 
	alu_inx, alu_dex, alu_cpx, 
	alu_lsr16, alu_lsl16, 
	alu_ror8, alu_rol8, 
	alu_asr8, alu_asl8, alu_lsr8, 
	alu_sei, alu_cli, alu_sec, alu_clc, alu_sev, alu_clv, alu_tpa, alu_tap, 
	alu_ld8, alu_st8, alu_ld16, alu_st16, alu_nop, alu_daa);

	signal op_code : std_logic_vector(7 downto 0);
	signal acca : std_logic_vector(7 downto 0);
	signal accb : std_logic_vector(7 downto 0);
	signal cc : std_logic_vector(7 downto 0);
	signal cc_out : std_logic_vector(7 downto 0);
	signal xreg : std_logic_vector(15 downto 0);
	signal sp : std_logic_vector(15 downto 0);
	signal ea : std_logic_vector(15 downto 0);
	signal pc : std_logic_vector(15 downto 0);
	signal md : std_logic_vector(15 downto 0);
	signal left : std_logic_vector(15 downto 0);
	signal right : std_logic_vector(15 downto 0);
	signal out_alu : std_logic_vector(15 downto 0);
	signal iv : std_logic_vector(1 downto 0);
	signal nmi_req : std_logic;
	signal nmi_ack : std_logic;

	signal state : state_type;
	signal next_state : state_type;
	signal pc_ctrl : pc_type;
	signal ea_ctrl : ea_type;
	signal op_ctrl : op_type;
	signal md_ctrl : md_type;
	signal acca_ctrl : acca_type;
	signal accb_ctrl : accb_type;
	signal ix_ctrl : ix_type;
	signal cc_ctrl : cc_type;
	signal sp_ctrl : sp_type;
	signal iv_ctrl : iv_type;
	signal left_ctrl : left_type;
	signal right_ctrl : right_type;
	signal alu_ctrl : alu_type;
	signal addr_ctrl : addr_type;
	signal dout_ctrl : dout_type;
	signal nmi_ctrl : nmi_type;

	type ram_type is array (0 to 127) of std_logic_vector(7 downto 0);
	signal internal_ram: ram_type;
	signal is_ram_access: std_logic;
	signal memory_data_in: std_logic_vector(7 downto 0);

begin
	----------------------------------
	--
	-- Internal RAM access detection
	--
	----------------------------------
	ram_access: process(addr_ctrl, pc, ea, sp)
	begin
		case addr_ctrl is
			when fetch_ad =>
				if pc(15 downto 7) = "000000000" then
					is_ram_access <= '1';
				else
					is_ram_access <= '0';
				end if;
			when read_ad =>
				if ea(15 downto 7) = "000000000" then
					is_ram_access <= '1';
				else
					is_ram_access <= '0';
				end if;
			when write_ad =>
				if ea(15 downto 7) = "000000000" then
					is_ram_access <= '1';
				else
					is_ram_access <= '0';
				end if;
			when push_ad =>
				if sp(15 downto 7) = "000000000" then
					is_ram_access <= '1';
				else
					is_ram_access <= '0';
				end if;
			when pull_ad =>
				if sp(15 downto 7) = "000000000" then
					is_ram_access <= '1';
				else
					is_ram_access <= '0';
				end if;
			when others =>
				is_ram_access <= '0';
		end case;
	end process;

	----------------------------------
	--
	-- Address bus multiplexer
	--
	----------------------------------

	addr_mux : process(addr_ctrl, pc, ea, sp, iv, is_ram_access)
	begin
		case addr_ctrl is
			when idle_ad => 
				address <= "1111111111111111";
				vma <= '0';
				rw <= '1';
			when fetch_ad => 
				address <= pc;
				if is_ram_access = '1' then
					vma <= '0';
				else
					vma <= '1';
				end if;
				rw <= '1';
			when read_ad => 
				address <= ea;
				if is_ram_access = '1' then
					vma <= '0';
				else
					vma <= '1';
				end if;
				rw <= '1';
			when write_ad => 
				address <= ea;
				if is_ram_access = '1' then
					vma <= '0';
					rw <= '1'; -- idle for internal
				else
					vma <= '1';
					rw <= '0';
				end if;
			when push_ad => 
				address <= sp;
				if is_ram_access = '1' then
					vma <= '0';
					rw <= '1'; -- idle for internal
				else
					vma <= '1';
					rw <= '0';
				end if;
			when pull_ad => 
				address <= sp;
				if is_ram_access = '1' then
					vma <= '0';
				else
					vma <= '1';
				end if;
				rw <= '1';
			when int_hi_ad => 
				address <= "1111111111111" & iv & "0";
				vma <= '1';
				rw <= '1';
			when int_lo_ad => 
				address <= "1111111111111" & iv & "1";
				vma <= '1';
				rw <= '1';
			when others => 
				address <= "1111111111111111";
				vma <= '0';
				rw <= '1';
		end case;
	end process;

	----------------------------------
	--
	-- Internal RAM write
	--
	----------------------------------
	ram_write: process(clk, addr_ctrl, is_ram_access, address, data_out)
	begin
		if clk'event and clk = '0' then
			if is_ram_access = '1' and (addr_ctrl = write_ad or addr_ctrl = push_ad) then
				internal_ram(to_integer(unsigned(address(6 downto 0)))) <= data_out;
			end if;
		end if;
	end process;

	----------------------------------
	--
	-- Memory data in mux (external or internal RAM)
	--
	----------------------------------
	memory_data_in <= internal_ram(to_integer(unsigned(address(6 downto 0)))) when 
	                  (is_ram_access = '1' and (addr_ctrl = fetch_ad or addr_ctrl = read_ad or addr_ctrl = pull_ad)) 
	                  else data_in;

	--------------------------------
	--
	-- Data Bus output
	--
	--------------------------------
	dout_mux : process (clk, dout_ctrl, md, acca, accb, xreg, pc, cc)
	begin
		case dout_ctrl is
			when md_hi_dout => -- alu output
				data_out <= md(15 downto 8);
			when md_lo_dout => 
				data_out <= md(7 downto 0);
			when acca_dout => -- accumulator a
				data_out <= acca;
			when accb_dout => -- accumulator b
				data_out <= accb;
			when ix_lo_dout => -- index reg
				data_out <= xreg(7 downto 0);
			when ix_hi_dout => -- index reg
				data_out <= xreg(15 downto 8);
			when cc_dout => -- condition codes
				data_out <= cc;
			when pc_lo_dout => -- low order pc
				data_out <= pc(7 downto 0);
			when pc_hi_dout => -- high order pc
				data_out <= pc(15 downto 8);
			when others => 
				data_out <= "00000000";
		end case;
	end process;
	----------------------------------
	--
	-- Program Counter Control
	--
	----------------------------------

	pc_mux : process (clk, pc_ctrl, pc, out_alu, memory_data_in, ea, hold)
		variable tempof : std_logic_vector(15 downto 0);
		variable temppc : std_logic_vector(15 downto 0);
	begin
		case pc_ctrl is
			when add_ea_pc => 
				if ea(7) = '0' then
					tempof := "00000000" & ea(7 downto 0);
				else
					tempof := "11111111" & ea(7 downto 0);
				end if;
			when inc_pc => 
				tempof := "0000000000000001";
			when others => 
				tempof := "0000000000000000";
		end case;

		case pc_ctrl is
			when reset_pc => 
				temppc := "1111111111111110";
			when load_ea_pc => 
				temppc := ea;
			when load_ext_pc =>
				temppc(15 downto 8) := ea(7 downto 0);
				temppc(7 downto 0) := memory_data_in;
			when pull_lo_pc => 
				temppc(7 downto 0) := memory_data_in;
				temppc(15 downto 8) := pc(15 downto 8);
			when pull_hi_pc => 
				temppc(7 downto 0) := pc(7 downto 0);
				temppc(15 downto 8) := memory_data_in;
			when others => 
				temppc := pc;
		end case;

		if clk'event and clk = '0' then
			if hold = '1' then
				pc <= pc;
			else
				pc <= temppc + tempof;
			end if;
		end if;
	end process;

	----------------------------------
	--
	-- Effective Address Control
	--
	----------------------------------

	ea_mux : process (clk, ea_ctrl, ea, out_alu, memory_data_in, accb, xreg, hold)
		variable tempind : std_logic_vector(15 downto 0);
		variable tempea : std_logic_vector(15 downto 0);
	begin
		case ea_ctrl is
			when add_ix_ea => 
				tempind := "00000000" & ea(7 downto 0);
			when inc_ea => 
				tempind := "0000000000000001";
			when others => 
				tempind := "0000000000000000";
		end case;

		case ea_ctrl is
			when reset_ea => 
				tempea := "0000000000000000";
			when load_accb_ea => 
				tempea := "00000000" & accb(7 downto 0);
			when add_ix_ea => 
				tempea := xreg;
			when fetch_first_ea => 
				tempea(7 downto 0) := memory_data_in;
				tempea(15 downto 8) := "00000000";
			when fetch_next_ea => 
				tempea(7 downto 0) := memory_data_in;
				tempea(15 downto 8) := ea(7 downto 0);
			when others => 
				tempea := ea;
		end case;

		if clk'event and clk = '0' then
			if hold = '1' then
				ea <= ea;
			else
				ea <= tempea + tempind;
			end if;
		end if;
	end process;

	--------------------------------
	--
	-- Accumulator A
	--
	--------------------------------
	acca_mux : process (clk, acca_ctrl, out_alu, acca, memory_data_in, hold)
	begin
		if clk'event and clk = '0' then
			if hold = '1' then
				acca <= acca;
			else
				case acca_ctrl is
					when reset_acca => 
						acca <= "00000000";
					when load_acca => 
						acca <= out_alu(7 downto 0);
					when load_hi_acca => 
						acca <= out_alu(15 downto 8);
					when pull_acca => 
						acca <= memory_data_in;
					when others => 
						-- when latch_acca =>
						acca <= acca;
				end case;
			end if;
		end if;
	end process;

	--------------------------------
	--
	-- Accumulator B
	--
	--------------------------------
	accb_mux : process (clk, accb_ctrl, out_alu, accb, memory_data_in, hold)
	begin
		if clk'event and clk = '0' then
			if hold = '1' then
				accb <= accb;
			else
				case accb_ctrl is
					when reset_accb => 
						accb <= "00000000";
					when load_accb => 
						accb <= out_alu(7 downto 0);
					when pull_accb => 
						accb <= memory_data_in;
					when others => 
						-- when latch_accb =>
						accb <= accb;
				end case;
			end if;
		end if;
	end process;

	--------------------------------
	--
	-- X Index register
	--
	--------------------------------
	ix_mux : process (clk, ix_ctrl, out_alu, xreg, memory_data_in, hold)
	begin
		if clk'event and clk = '0' then
			if hold = '1' then
				xreg <= xreg;
			else
				case ix_ctrl is
					when reset_ix => 
						xreg <= "0000000000000000";
					when load_ix => 
						xreg <= out_alu(15 downto 0);
					when pull_hi_ix => 
						xreg(15 downto 8) <= memory_data_in;
					when pull_lo_ix => 
						xreg(7 downto 0) <= memory_data_in;
					when others => 
						-- when latch_ix =>
						xreg <= xreg;
				end case;
			end if;
		end if;
	end process;

	--------------------------------
	--
	-- stack pointer
	--
	--------------------------------
	sp_mux : process (clk, sp_ctrl, out_alu, hold)
	begin
		if clk'event and clk = '0' then
			if hold = '1' then
				sp <= sp;
			else
				case sp_ctrl is
					when reset_sp => 
						sp <= "0000000000000000";
					when load_sp => 
						sp <= out_alu(15 downto 0);
					when others => 
						-- when latch_sp =>
						sp <= sp;
				end case;
			end if;
		end if;
	end process;

	--------------------------------
	--
	-- Memory Data
	--
	--------------------------------
	md_mux : process (clk, md_ctrl, out_alu, memory_data_in, md, hold)
	begin
		if clk'event and clk = '0' then
			if hold = '1' then
				md <= md;
			else
				case md_ctrl is
					when reset_md => 
						md <= "0000000000000000";
					when load_md => 
						md <= out_alu(15 downto 0);
					when fetch_first_md => 
						md(15 downto 8) <= "00000000";
						md(7 downto 0) <= memory_data_in;
					when fetch_next_md => 
						md(15 downto 8) <= md(7 downto 0);
						md(7 downto 0) <= memory_data_in;
					when shiftl_md => 
						md(15 downto 1) <= md(14 downto 0);
						md(0) <= '0';
					when others => 
						-- when latch_md =>
						md <= md;
				end case;
			end if;
		end if;
	end process;
	----------------------------------
	--
	-- Condition Codes
	--
	----------------------------------

	cc_mux : process (clk, cc_ctrl, cc_out, cc, memory_data_in, hold)
	begin
		if clk'event and clk = '0' then
			if hold = '1' then
				cc <= cc;
			else
				case cc_ctrl is
					when reset_cc => 
						cc <= "11010000";
					when load_cc => 
						cc <= cc_out;
					when pull_cc => 
						cc <= memory_data_in;
					when others => 
						-- when latch_cc =>
						cc <= cc;
				end case;
			end if;
		end if;
	end process;

	----------------------------------
	--
	-- interrupt vector
	--
	----------------------------------

	iv_mux : process (clk, iv_ctrl, hold)
	begin
		if clk'event and clk = '0' then
			if hold = '1' then
				iv <= iv;
			else
				case iv_ctrl is
					when reset_iv => 
						iv <= "11";
					when nmi_iv => 
						iv <= "10";
					when swi_iv => 
						iv <= "01";
					when irq_iv => 
						iv <= "00";
					when others => 
						iv <= iv;
				end case;
			end if;
		end if;
	end process;

	----------------------------------
	--
	-- op code fetch
	--
	----------------------------------

	op_fetch : process (clk, memory_data_in, op_ctrl, op_code, hold)
	begin
		if clk'event and clk = '0' then
			if hold = '1' then
				op_code <= op_code;
			else
				case op_ctrl is
					when reset_op => 
						op_code <= "00000001"; -- nop
					when fetch_op => 
						op_code <= memory_data_in;
					when others => 
						-- when latch_op =>
						op_code <= op_code;
				end case;
			end if;
		end if;
	end process;

	----------------------------------
	--
	-- Left Mux
	--
	----------------------------------

	left_mux : process (left_ctrl, acca, accb, xreg, sp, pc, ea, md)
	begin
		case left_ctrl is
			when acca_left => 
				left(15 downto 8) <= "00000000";
				left(7 downto 0) <= acca;
			when accb_left => 
				left(15 downto 8) <= "00000000";
				left(7 downto 0) <= accb;
			when accd_left => 
				left(15 downto 8) <= acca;
				left(7 downto 0) <= accb;
			when ix_left => 
				left <= xreg;
			when sp_left => 
				left <= sp;
			when others => 
				-- when md_left =>
				left <= md;
		end case;
	end process;
	----------------------------------
	--
	-- Right Mux
	--
	----------------------------------

	right_mux : process (right_ctrl, memory_data_in, md, accb, ea)
	begin
		case right_ctrl is
			when zero_right => 
				right <= "0000000000000000";
			when plus_one_right => 
				right <= "0000000000000001";
			when accb_right => 
				right <= "00000000" & accb;
			when others => 
				-- when md_right =>
				right <= md;
		end case;
	end process;

	----------------------------------
	--
	-- Arithmetic Logic Unit
	--
	----------------------------------

	mux_alu : process (alu_ctrl, cc, left, right, out_alu, cc_out)
		variable valid_lo, valid_hi : boolean;
		variable carry_in : std_logic;
		variable daa_reg : std_logic_vector(7 downto 0);
	begin
		case alu_ctrl is
			when alu_adc | alu_sbc | 
				alu_rol8 | alu_ror8 => 
				carry_in := cc(CBIT);
			when others => 
				carry_in := '0';
		end case;

		valid_lo := left(3 downto 0) <= 9;
		valid_hi := left(7 downto 4) <= 9;

		if (cc(CBIT) = '0') then
			if (cc(HBIT) = '1') then
				if valid_hi then
					daa_reg := "00000110";
				else
					daa_reg := "01100110";
				end if;
			else
				if valid_lo then
					if valid_hi then
						daa_reg := "00000000";
					else
						daa_reg := "01100000";
					end if;
				else
					if (left(7 downto 4) <= 8) then
						daa_reg := "00000110";
					else
						daa_reg := "01100110";
					end if;
				end if;
			end if;
		else
			if (cc(HBIT) = '1') then
				daa_reg := "01100110";
			else
				if valid_lo then
					daa_reg := "01100000";
				else
					daa_reg := "01100110";
				end if;
			end if;
		end if;

		case alu_ctrl is
			when alu_add8 | alu_inc | 
				alu_add16 | alu_inx | 
				alu_adc => 
				out_alu <= left + right + ("000000000000000" & carry_in);
			when alu_sub8 | alu_dec | 
				alu_sub16 | alu_dex | 
				alu_sbc | alu_cpx => 
				out_alu <= left - right - ("000000000000000" & carry_in);
			when alu_and => 
				out_alu <= left and right; -- and/bit
			when alu_ora => 
				out_alu <= left or right; -- or
			when alu_eor => 
				out_alu <= left xor right; -- eor/xor
			when alu_lsl16 | alu_asl8 | alu_rol8 => 
				out_alu <= left(14 downto 0) & carry_in; -- rol8/asl8/lsl16
			when alu_lsr16 | alu_lsr8 => 
				out_alu <= carry_in & left(15 downto 1); -- lsr
			when alu_ror8 => 
				out_alu <= "00000000" & carry_in & left(7 downto 1); -- ror
			when alu_asr8 => 
				out_alu <= "00000000" & left(7) & left(7 downto 1); -- asr
			when alu_neg => 
				out_alu <= right - left; -- neg (right=0)
			when alu_com => 
				out_alu <= not left;
			when alu_clr | alu_ld8 | alu_ld16 => 
				out_alu <= right; -- clr, ld
			when alu_st8 | alu_st16 => 
				out_alu <= left;
			when alu_daa => 
				out_alu <= left + ("00000000" & daa_reg);
			when alu_tpa => 
				out_alu <= "00000000" & cc;
			when others => 
				out_alu <= left; -- nop
		end case;

		--
		-- carry bit
		--
		case alu_ctrl is
			when alu_add8 | alu_adc => 
				cc_out(CBIT) <= (left(7) and right(7)) or
					(left(7) and not out_alu(7)) or
					(right(7) and not out_alu(7));
			when alu_sub8 | alu_sbc => 
				cc_out(CBIT) <= ((not left(7)) and right(7)) or
					((not left(7)) and out_alu(7)) or
					(right(7) and out_alu(7));
			when alu_add16 => 
				cc_out(CBIT) <= (left(15) and right(15)) or
					(left(15) and not out_alu(15)) or
					(right(15) and not out_alu(15));
			when alu_sub16 => 
				cc_out(CBIT) <= ((not left(15)) and right(15)) or
					((not left(15)) and out_alu(15)) or
					(right(15) and out_alu(15));
			when alu_ror8 | alu_lsr16 | alu_lsr8 | alu_asr8 => 
				cc_out(CBIT) <= left(0);
			when alu_rol8 | alu_asl8 => 
				cc_out(CBIT) <= left(7);
			when alu_lsl16 => 
				cc_out(CBIT) <= left(15);
			when alu_com => 
				cc_out(CBIT) <= '1';
			when alu_neg | alu_clr => 
				cc_out(CBIT) <= out_alu(7) or out_alu(6) or out_alu(5) or out_alu(4) or
					out_alu(3) or out_alu(2) or out_alu(1) or out_alu(0);
			when alu_daa => 
				if (daa_reg(7 downto 4) = "0110") then
					cc_out(CBIT) <= '1';
				else
					cc_out(CBIT) <= '0';
				end if;
			when alu_sec => 
				cc_out(CBIT) <= '1';
			when alu_clc => 
				cc_out(CBIT) <= '0';
			when alu_tap => 
				cc_out(CBIT) <= left(CBIT);
			when others => -- carry is not affected by cpx
				cc_out(CBIT) <= cc(CBIT);
		end case;
		--
		-- Zero flag
		--
		case alu_ctrl is
			when alu_add8 | alu_sub8 | 
				alu_adc | alu_sbc | 
				alu_and | alu_ora | alu_eor | 
				alu_inc | alu_dec | 
				alu_neg | alu_com | alu_clr | 
				alu_rol8 | alu_ror8 | alu_asr8 | alu_asl8 | alu_lsr8 | 
				alu_ld8 | alu_st8 | alu_tst => 
				cc_out(ZBIT) <= '1' when out_alu(7 downto 0) = "00000000" else '0';
			when alu_add16 | 
				alu_sub16 | alu_lsl16 | alu_lsr16 | 
				alu_inx | alu_dex | 
				alu_ld16 | alu_st16 | alu_cpx => 
				cc_out(ZBIT) <= '1' when out_alu(15 downto 0) = "0000000000000000" else '0';
			when alu_tap => 
				cc_out(ZBIT) <= left(ZBIT);
			when others => 
				cc_out(ZBIT) <= cc(ZBIT);
		end case;
		--
		-- Negative flag
		--
		case alu_ctrl is
			when alu_add8 | 
				alu_sub8 | 
				alu_adc | alu_sbc | 
				alu_and | alu_ora | alu_eor | 
				alu_rol8 | alu_ror8 | alu_asr8 | alu_asl8 | alu_lsr8 | 
				alu_inc | alu_dec | alu_neg | alu_com | alu_clr | alu_ld8 | alu_st8 | alu_tst => 
				cc_out(NBIT) <= out_alu(7);
			when alu_add16 | alu_sub16 | alu_ld16 | alu_st16 | alu_cpx => 
				cc_out(NBIT) <= out_alu(15);
			when alu_tap => 
				cc_out(NBIT) <= left(NBIT);
			when others => 
				cc_out(NBIT) <= cc(NBIT);
		end case;
		--
		-- Interrupt mask flag
		--
		case alu_ctrl is
			when alu_sei => 
				cc_out(IBIT) <= '1'; -- set interrupt mask
			when alu_cli => 
				cc_out(IBIT) <= '0'; -- clear interrupt mask
			when alu_tap => 
				cc_out(IBIT) <= left(IBIT);
			when others => 
				cc_out(IBIT) <= cc(IBIT); -- interrupt mask
		end case;
		--
		-- Half Carry flag
		--
		case alu_ctrl is
			when alu_add8 | alu_adc => 
				cc_out(HBIT) <= (left(3) and right(3)) or
					(right(3) and not out_alu(3)) or 
					(left(3) and not out_alu(3));
			when alu_tap => 
				cc_out(HBIT) <= left(HBIT);
			when others => 
				cc_out(HBIT) <= cc(HBIT);
		end case;
		--
		-- Overflow flag
		--
		case alu_ctrl is
			when alu_add8 | alu_adc => 
				cc_out(VBIT) <= (left(7) and right(7) and not out_alu(7)) or
					((not left(7)) and (not right(7)) and out_alu(7));
			when alu_sub8 | alu_sbc => 
				cc_out(VBIT) <= (left(7) and (not right(7)) and (not out_alu(7))) or
					((not left(7)) and right(7) and out_alu(7));
			when alu_add16 => 
				cc_out(VBIT) <= (left(15) and right(15) and not out_alu(15)) or
					((not left(15)) and (not right(15)) and out_alu(15));
			when alu_sub16 => 
				cc_out(VBIT) <= (left(15) and (not right(15)) and (not out_alu(15))) or
					((not left(15)) and right(15) and out_alu(15));
			when alu_inc => 
				cc_out(VBIT) <= ((not left(7)) and left(6) and left(5) and left(4) and
					left(3) and left(2) and left(1) and left(0));
			when alu_dec | alu_neg => 
				cc_out(VBIT) <= (left(7) and (not left(6)) and (not left(5)) and (not left(4)) and
					(not left(3)) and (not left(2)) and (not left(1)) and (not left(0)));
			when alu_asr8 | alu_lsr8 | alu_lsl16 => 
				cc_out(VBIT) <= left(0) xor cc(NBIT);
			when alu_ror8 => 
				cc_out(VBIT) <= left(0) xor left(7);
			when alu_rol8 => 
				cc_out(VBIT) <= left(7) xor cc(CBIT);
			when alu_asl8 => 
				cc_out(VBIT) <= left(7) xor left(6);
			when alu_tst => 
				cc_out(VBIT) <= '0';
			when alu_clv => 
				cc_out(VBIT) <= '0';
			when alu_sev => 
				cc_out(VBIT) <= '1';
			when alu_tap => 
				cc_out(VBIT) <= left(VBIT);
			when alu_ld8 | alu_st8 | alu_ld16 | alu_st16 => 
				cc_out(VBIT) <= '0';
			when others => 
				cc_out(VBIT) <= cc(VBIT);
		end case;

		--
		-- Xbit
		--
		cc_out(XBIT) <= '1'; -- Always set

		--
		-- Sbit
		--
		cc_out(SBIT) <= '1'; -- Always set

	end process;

	------------------------------------
	--
	-- NMI edge detect
	--
	------------------------------------

	nmi_mux : process (clk, rst, nmi, nmi_ctrl, nmi_ack, nmi_req, hold)
	begin
		if clk'event and clk = '0' then
			if (hold = '1') then
				nmi_req <= nmi_req;
				nmi_ack <= nmi_ack;
			else
				case nmi_ctrl is
					when set_nmi => 
						nmi_ack <= '1';
						nmi_req <= nmi_req;
					when reset_nmi => 
						nmi_ack <= '0';
						nmi_req <= nmi_req;
					when others => 
						-- when latch_nmi =>
						nmi_ack <= nmi_ack;
						if (nmi_ack = '0') and (nmi = '1') then
							nmi_req <= '1';
						elsif (nmi_ack = '1') and (nmi = '0') then
							nmi_req <= '0';
						else
							nmi_req <= nmi_req;
						end if;
				end case;
			end if;
		elsif rst = '1' then
			nmi_req <= '0';
			nmi_ack <= '0';
		end if;
	end process;

	------------------------------------
	--
	-- state sequencer
	--
	------------------------------------
	state_decode : process (state, op_code, cc, ea, irq, nmi_req, nmi_ack, halt, hold)
	begin
		case state is
			when reset_state => 
				-- default
				acca_ctrl <= reset_acca;
				accb_ctrl <= reset_accb;
				ix_ctrl <= reset_ix;
				sp_ctrl <= reset_sp;
				pc_ctrl <= reset_pc;
				ea_ctrl <= reset_ea;
				md_ctrl <= reset_md;
				iv_ctrl <= reset_iv;
				op_ctrl <= reset_op;
				nmi_ctrl <= reset_nmi;
				-- idle ALU
				left_ctrl <= acca_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_nop;
				cc_ctrl <= reset_cc;
				-- idle bus
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= vect_hi_state;

				--
				-- Jump via extended address
				-- EA holds extended address
				--
			when jmp_state => 
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- jump to extended address
				left_ctrl <= acca_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_nop;
				cc_ctrl <= latch_cc;
				pc_ctrl <= load_ea_pc;
				-- idle bus
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= fetch_state;

				--
				-- single operand via direct page
				-- fetch operand address
				--
			when indexed_state => 
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				pc_ctrl <= inc_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				-- calculate the effective address using indexing
				left_ctrl <= ix_left;
				right_ctrl <= md_right;
				alu_ctrl <= alu_add16;
				cc_ctrl <= latch_cc;
				ea_ctrl <= add_ix_ea;
				-- idle the bus
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				case op_code(3 downto 0) is
					when "0000" | -- suba
						"0001" | -- cmpa
						"0010" | -- sbca
						"0011" | -- subd
						"0100" | -- anda
						"0101" | -- bita
						"0110" | -- ldaa
						"0111" | -- staa
						"1000" | -- eora
						"1001" | -- adca
						"1010" | -- oraa
						"1011" | -- adda
						"1100" | -- cpxa
						"1101" | -- bsr / jsr
						"1110" | -- lds
						"1111" => -- sts
						next_state <= stall1_idx_read_state;
					when others => 
						next_state <= fetch_state;
				end case;

				--
				-- Indirect addressing
				-- read first byte of indirect address
				--
			when read8_state => 
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				pc_ctrl <= latch_pc;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				-- read first byte from ea
				md_ctrl <= fetch_first_md;
				left_ctrl <= acca_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_nop;
				cc_ctrl <= latch_cc;
				ea_ctrl <= inc_ea;
				-- read memory
				addr_ctrl <= read_ad;
				dout_ctrl <= md_lo_dout;
				case op_code(3 downto 0) is
					when "0111" | -- staa
						"1111" => -- sts
						next_state <= stall1_write_state;
					when others => 
						next_state <= stall1_state;
				end case;

				--
				-- Indirect addressing
				-- read second byte of indirect address
				--
			when immediate16_state => 
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				-- increment pc
				left_ctrl <= pc_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_add16;
				cc_ctrl <= latch_cc;
				pc_ctrl <= add_ea_pc;
				-- read second byte from ea
				ea_ctrl <= fetch_next_ea;
				addr_ctrl <= read_ad;
				dout_ctrl <= md_lo_dout;
				case op_code(3 downto 2) is
					when "00" => -- suba, cmpa, sbca, cpx, anda, bita, ldaa,eora, adca, oraa, adda,
						next_state <= stall1_state;
					when "01" => -- subd
						next_state <= stall2_state;
					when "10" => -- lds
						next_state <= stall2_state;
					when others => 
						next_state <= fetch_state;
				end case;

				--
				-- single operand via direct page
				-- fetch operand address
				--
			when direct_state => 
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				-- increment pc
				left_ctrl <= pc_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_add16;
				cc_ctrl <= latch_cc;
				pc_ctrl <= inc_pc;
				-- ea holds 8 bit zero page address operand
				ea_ctrl <= fetch_first_ea;
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				case op_code(3 downto 0) is
					when "1101" | -- bsr / jsr
						"1110" | -- lds
						"1111" => -- sts
						next_state <= immediate16_state;
					when others => 
						next_state <= read8_state;
				end case;

				--
				-- Single operand indexed
				-- read 8 bit offset
				--
			when extended_state => 
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				-- increment pc
				pc_ctrl <= inc_pc;
				-- ea holds hi byte
				left_ctrl <= acca_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_nop;
				cc_ctrl <= latch_cc;
				ea_ctrl <= fetch_first_ea;
				-- idle bus
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				case op_code(3 downto 0) is
					when "0000" | -- suba
						"0001" | -- cmpa
						"0010" | -- sbca
						"0011" | -- subd
						"0100" | -- anda
						"0101" | -- bita
						"0110" | -- ldaa
						"0111" | -- staa
						"1000" | -- eora
						"1001" | -- adca
						"1010" | -- oraa
						"1011" | -- adda
						"1100" | -- cpxa
						"1101" | -- bsr / jsr
						"1110" | -- lds
						"1111" => -- sts
						next_state <= immediate16_state;
					when others => 
						next_state <= fetch_state;
				end case;

				--
				-- 16 bit operand
				-- ea holds high byte of extended address
				-- read low byte of extended address
				--
			when read16_state => 
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				pc_ctrl <= latch_pc;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				-- read low byte from ea
				md_ctrl <= fetch_next_md;
				left_ctrl <= acca_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_nop;
				cc_ctrl <= latch_cc;
				ea_ctrl <= inc_ea;
				-- read memory into md
				addr_ctrl <= read_ad;
				dout_ctrl <= md_lo_dout;
				case op_code(3 downto 1) is
					when "001" | -- subd
						"110" | -- lds
						"111" => -- sts
						next_state <= stall1_state;
					when others => 
						next_state <= stall2_state;
				end case;

				--
				-- decrement sp
				-- write low byte
				--
			when write8_state => 
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_sub16;
				cc_ctrl <= latch_cc;
				ea_ctrl <= latch_ea;
				-- write low byte of md
				addr_ctrl <= write_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= fetch_state;

				--
				-- decrement sp
				-- write high byte
				--
			when write16_state => 
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_sub16;
				cc_ctrl <= latch_cc;
				ea_ctrl <= latch_ea;
				-- write high byte of md
				addr_ctrl <= write_ad;
				dout_ctrl <= md_hi_dout;
				next_state <= stall1_write_state;

				--
				-- bit manipulation
				--
			when execute_state => -- execute
				-- defaults
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				-- idle the bus
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				case op_code is
					when "00000000" => -- nega
						-- default
						accb_ctrl <= latch_accb;
						md_ctrl <= latch_md;
						pc_ctrl <= latch_pc;
						ea_ctrl <= latch_ea;
						-- neg
						left_ctrl <= acca_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_neg;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_acca;
						next_state <= fetch_state;
					when "00000011" => -- coma
						-- default
						accb_ctrl <= latch_accb;
						md_ctrl <= latch_md;
						pc_ctrl <= latch_pc;
						ea_ctrl <= latch_ea;
						-- com
						left_ctrl <= acca_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_com;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_acca;
						next_state <= fetch_state;
					when "00000100" => -- lsra
						-- default
						accb_ctrl <= latch_accb;
						md_ctrl <= latch_md;
						pc_ctrl <= latch_pc;
						ea_ctrl <= latch_ea;
						-- lsr
						left_ctrl <= acca_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_lsr8;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_acca;
						next_state <= fetch_state;
					when "00000110" => -- rora
						-- default
						accb_ctrl <= latch_accb;
						md_ctrl <= latch_md;
						pc_ctrl <= latch_pc;
						ea_ctrl <= latch_ea;
						-- ror
						left_ctrl <= acca_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_ror8;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_acca;
						next_state <= fetch_state;
					when "00000111" => -- asra
						-- default
						accb_ctrl <= latch_accb;
						md_ctrl <= latch_md;
						pc_ctrl <= latch_pc;
						ea_ctrl <= latch_ea;
						-- asr
						left_ctrl <= acca_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_asr8;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_acca;
						next_state <= fetch_state;
					when "00001000" => -- asla
						-- default
						accb_ctrl <= latch_accb;
						md_ctrl <= latch_md;
						pc_ctrl <= latch_pc;
						ea_ctrl <= latch_ea;
						-- asl
						left_ctrl <= acca_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_asl8;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_acca;
						next_state <= fetch_state;
					when "00001001" => -- rola
						-- default
						accb_ctrl <= latch_accb;
						md_ctrl <= latch_md;
						pc_ctrl <= latch_pc;
						ea_ctrl <= latch_ea;
						-- rol
						left_ctrl <= acca_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_rol8;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_acca;
						next_state <= fetch_state;
					when "00001010" => -- deca
						-- default
						accb_ctrl <= latch_accb;
						md_ctrl <= latch_md;
						pc_ctrl <= latch_pc;
						ea_ctrl <= latch_ea;
						-- dec
						left_ctrl <= acca_left;
						right_ctrl <= plus_one_right;
						alu_ctrl <= alu_dec;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_acca;
						next_state <= fetch_state;
					when "00001011" => -- clra
						-- default
						accb_ctrl <= latch_accb;
						md_ctrl <= latch_md;
						pc_ctrl <= latch_pc;
						ea_ctrl <= latch_ea;
						-- clr
						left_ctrl <= acca_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_clr;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_acca;
						next_state <= fetch_state;
					when "00001100" => -- inca
						-- default
						accb_ctrl <= latch_accb;
						md_ctrl <= latch_md;
						pc_ctrl <= latch_pc;
						ea_ctrl <= latch_ea;
						-- inc
						left_ctrl <= acca_left;
						right_ctrl <= plus_one_right;
						alu_ctrl <= alu_inc;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_acca;
						next_state <= fetch_state;
					when "00001101" => -- tsta
						-- default
						accb_ctrl <= latch_accb;
						md_ctrl <= latch_md;
						pc_ctrl <= latch_pc;
						ea_ctrl <= latch_ea;
						-- tst
						left_ctrl <= acca_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_tst;
						cc_ctrl <= load_cc;
						acca_ctrl <= latch_acca;
						next_state <= fetch_state;
					when "00001111" => -- clra
						-- default
						accb_ctrl <= latch_accb;
						md_ctrl <= latch_md;
						pc_ctrl <= latch_pc;
						ea_ctrl <= latch_ea;
						-- clr
						left_ctrl <= acca_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_clr;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_acca;
						next_state <= fetch_state;
					when "01000000" => -- negb
						-- default
						acca_ctrl <= latch_acca;
						md_ctrl <= latch_md;
						pc_ctrl <= latch_pc;
						ea_ctrl <= latch_ea;
						-- neg
						left_ctrl <= accb_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_neg;
						cc_ctrl <= load_cc;
						accb_ctrl <= load_accb;
						next_state <= fetch_state;
					when "01000011" => -- comb
						-- default
						acca_ctrl <= latch_acca;
						md_ctrl <= latch_md;
						pc_ctrl <= latch_pc;
						ea_ctrl <= latch_ea;
						-- com
						left_ctrl <= accb_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_com;
						cc_ctrl <= load_cc;
						accb_ctrl <= load_accb;
						next_state <= fetch_state;
					when "01000100" => -- lsrb
						-- default
						acca_ctrl <= latch_acca;
						md_ctrl <= latch_md;
						pc_ctrl <= latch_pc;
						ea_ctrl <= latch_ea;
						-- lsr
						left_ctrl <= accb_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_lsr8;
						cc_ctrl <= load_cc;
						accb_ctrl <= load_accb;
						next_state <= fetch_state;
					when "01000110" => -- rorb
						-- default
						acca_ctrl <= latch_acca;
						md_ctrl <= latch_md;
						pc_ctrl <= latch_pc;
						ea_ctrl <= latch_ea;
						-- ror
						left_ctrl <= accb_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_ror8;
						cc_ctrl <= load_cc;
						accb_ctrl <= load_accb;
						next_state <= fetch_state;
					when "01000111" => -- asrb
						-- default
						acca_ctrl <= latch_acca;
						md_ctrl <= latch_md;
						pc_ctrl <= latch_pc;
						ea_ctrl <= latch_ea;
						-- asr
						left_ctrl <= accb_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_asr8;
						cc_ctrl <= load_cc;
						accb_ctrl <= load_accb;
						next_state <= fetch_state;
					when "01001000" => -- aslb
						-- default
						acca_ctrl <= latch_acca;
						md_ctrl <= latch_md;
						pc_ctrl <= latch_pc;
						ea_ctrl <= latch_ea;
						-- asl
						left_ctrl <= accb_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_asl8;
						cc_ctrl <= load_cc;
						accb_ctrl <= load_accb;
						next_state <= fetch_state;
					when "01001001" => -- rolb
						-- default
						acca_ctrl <= latch_acca;
						md_ctrl <= latch_md;
						pc_ctrl <= latch_pc;
						ea_ctrl <= latch_ea;
						-- rol
						left_ctrl <= accb_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_rol8;
						cc_ctrl <= load_cc;
						accb_ctrl <= load_accb;
						next_state <= fetch_state;
					when "01001010" => -- decb
						-- default
						acca_ctrl <= latch_acca;
						md_ctrl <= latch_md;
						pc_ctrl <= latch_pc;
						ea_ctrl <= latch_ea;
						-- dec
						left_ctrl <= accb_left;
						right_ctrl <= plus_one_right;
						alu_ctrl <= alu_dec;
						cc_ctrl <= load_cc;
						accb_ctrl <= load_accb;
						next_state <= fetch_state;
					when "01001011" => -- clrb
						-- default
						acca_ctrl <= latch_acca;
						md_ctrl <= latch_md;
						pc_ctrl <= latch_pc;
						ea_ctrl <= latch_ea;
						-- clr
						left_ctrl <= accb_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_clr;
						cc_ctrl <= load_cc;
						accb_ctrl <= load_accb;
						next_state <= fetch_state;
					when "01001100" => -- incb
						-- default
						acca_ctrl <= latch_acca;
						md_ctrl <= latch_md;
						pc_ctrl <= latch_pc;
						ea_ctrl <= latch_ea;
						-- inc
						left_ctrl <= accb_left;
						right_ctrl <= plus_one_right;
						alu_ctrl <= alu_inc;
						cc_ctrl <= load_cc;
						accb_ctrl <= load_accb;
						next_state <= fetch_state;
					when "01001101" => -- tstb
						-- default
						acca_ctrl <= latch_acca;
						md_ctrl <= latch_md;
						pc_ctrl <= latch_pc;
						ea_ctrl <= latch_ea;
						-- tst
						left_ctrl <= accb_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_tst;
						cc_ctrl <= load_cc;
						accb_ctrl <= latch_accb;
						next_state <= fetch_state;
					when "01001111" => -- clrb
						-- default
						acca_ctrl <= latch_acca;
						md_ctrl <= latch_md;
						pc_ctrl <= latch_pc;
						ea_ctrl <= latch_ea;
						-- clr
						left_ctrl <= accb_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_clr;
						cc_ctrl <= load_cc;
						accb_ctrl <= load_accb;
						next_state <= fetch_state;
					when "01010000" => -- neg ix
						-- default
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						pc_ctrl <= latch_pc;
						-- neg
						left_ctrl <= md_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_neg;
						cc_ctrl <= load_cc;
						md_ctrl <= load_md;
						ea_ctrl <= latch_ea;
						next_state <= stall1_write_state;
					when "01010011" => -- com ix
						-- default
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						pc_ctrl <= latch_pc;
						-- com
						left_ctrl <= md_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_com;
						cc_ctrl <= load_cc;
						md_ctrl <= load_md;
						ea_ctrl <= latch_ea;
						next_state <= stall1_write_state;
					when "01010100" => -- lsr ix
						-- default
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						pc_ctrl <= latch_pc;
						-- lsr
						left_ctrl <= md_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_lsr8;
						cc_ctrl <= load_cc;
						md_ctrl <= load_md;
						ea_ctrl <= latch_ea;
						next_state <= stall1_write_state;
					when "01010110" => -- ror ix
						-- default
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						pc_ctrl <= latch_pc;
						-- ror
						left_ctrl <= md_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_ror8;
						cc_ctrl <= load_cc;
						md_ctrl <= load_md;
						ea_ctrl <= latch_ea;
						next_state <= stall1_write_state;
					when "01010111" => -- asr ix
						-- default
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						pc_ctrl <= latch_pc;
						-- asr
						left_ctrl <= md_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_asr8;
						cc_ctrl <= load_cc;
						md_ctrl <= load_md;
						ea_ctrl <= latch_ea;
						next_state <= stall1_write_state;
					when "01011000" => -- asl ix
						-- default
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						pc_ctrl <= latch_pc;
						-- asl
						left_ctrl <= md_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_asl8;
						cc_ctrl <= load_cc;
						md_ctrl <= load_md;
						ea_ctrl <= latch_ea;
						next_state <= stall1_write_state;
					when "01011001" => -- rol ix
						-- default
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						pc_ctrl <= latch_pc;
						-- rol
						left_ctrl <= md_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_rol8;
						cc_ctrl <= load_cc;
						md_ctrl <= load_md;
						ea_ctrl <= latch_ea;
						next_state <= stall1_write_state;
					when "01011010" => -- dec ix
						-- default
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						pc_ctrl <= latch_pc;
						-- dec
						left_ctrl <= md_left;
						right_ctrl <= plus_one_right;
						alu_ctrl <= alu_dec;
						cc_ctrl <= load_cc;
						md_ctrl <= load_md;
						ea_ctrl <= latch_ea;
						next_state <= stall1_write_state;
					when "01011011" => -- clr ix ??? (undocumented)
						-- default
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						pc_ctrl <= latch_pc;
						-- clr
						left_ctrl <= md_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_clr;
						cc_ctrl <= load_cc;
						md_ctrl <= load_md;
						ea_ctrl <= latch_ea;
						next_state <= stall1_write_state;
					when "01011100" => -- inc ix
						-- default
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						pc_ctrl <= latch_pc;
						-- inc
						left_ctrl <= md_left;
						right_ctrl <= plus_one_right;
						alu_ctrl <= alu_inc;
						cc_ctrl <= load_cc;
						md_ctrl <= load_md;
						ea_ctrl <= latch_ea;
						next_state <= stall1_write_state;
					when "01011101" => -- tst ix
						-- default
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						md_ctrl <= latch_md;
						pc_ctrl <= latch_pc;
						ea_ctrl <= latch_ea;
						-- tst
						left_ctrl <= md_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_tst;
						cc_ctrl <= load_cc;
						next_state <= fetch_state;
					when "01011110" => -- jmp ix
						-- default
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						md_ctrl <= latch_md;
						pc_ctrl <= load_ea_pc;
						ea_ctrl <= latch_ea;
						-- jmp
						left_ctrl <= md_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_nop;
						cc_ctrl <= latch_cc;
						next_state <= fetch_state;
					when "01011111" => -- clr ix
						-- default
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						pc_ctrl <= latch_pc;
						-- clr
						left_ctrl <= md_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_clr;
						cc_ctrl <= load_cc;
						md_ctrl <= load_md;
						ea_ctrl <= latch_ea;
						next_state <= stall1_write_state;
					when "01100000" => -- neg ex
						-- default
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						pc_ctrl <= latch_pc;
						-- neg
						left_ctrl <= md_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_neg;
						cc_ctrl <= load_cc;
						md_ctrl <= load_md;
						ea_ctrl <= latch_ea;
						next_state <= stall1_write_state;
					when "01100011" => -- com ex
						-- default
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						pc_ctrl <= latch_pc;
						-- com
						left_ctrl <= md_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_com;
						cc_ctrl <= load_cc;
						md_ctrl <= load_md;
						ea_ctrl <= latch_ea;
						next_state <= stall1_write_state;
					when "01100100" => -- lsr ex
						-- default
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						pc_ctrl <= latch_pc;
						-- lsr
						left_ctrl <= md_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_lsr8;
						cc_ctrl <= load_cc;
						md_ctrl <= load_md;
						ea_ctrl <= latch_ea;
						next_state <= stall1_write_state;
					when "01100110" => -- ror ex
						-- default
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						pc_ctrl <= latch_pc;
						-- ror
						left_ctrl <= md_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_ror8;
						cc_ctrl <= load_cc;
						md_ctrl <= load_md;
						ea_ctrl <= latch_ea;
						next_state <= stall1_write_state;
					when "01100111" => -- asr ex
						-- default
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						pc_ctrl <= latch_pc;
						-- asr
						left_ctrl <= md_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_asr8;
						cc_ctrl <= load_cc;
						md_ctrl <= load_md;
						ea_ctrl <= latch_ea;
						next_state <= stall1_write_state;
					when "01101000" => -- asl ex
						-- default
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						pc_ctrl <= latch_pc;
						-- asl
						left_ctrl <= md_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_asl8;
						cc_ctrl <= load_cc;
						md_ctrl <= load_md;
						ea_ctrl <= latch_ea;
						next_state <= stall1_write_state;
					when "01101001" => -- rol ex
						-- default
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						pc_ctrl <= latch_pc;
						-- rol
						left_ctrl <= md_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_rol8;
						cc_ctrl <= load_cc;
						md_ctrl <= load_md;
						ea_ctrl <= latch_ea;
						next_state <= stall1_write_state;
					when "01101010" => -- dec ex
						-- default
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						pc_ctrl <= latch_pc;
						-- dec
						left_ctrl <= md_left;
						right_ctrl <= plus_one_right;
						alu_ctrl <= alu_dec;
						cc_ctrl <= load_cc;
						md_ctrl <= load_md;
						ea_ctrl <= latch_ea;
						next_state <= stall1_write_state;
					when "01101011" => -- clr ex ??? (undocumented ?)
						-- default
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						pc_ctrl <= latch_pc;
						-- clr
						left_ctrl <= md_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_clr;
						cc_ctrl <= load_cc;
						md_ctrl <= load_md;
						ea_ctrl <= latch_ea;
						next_state <= stall1_write_state;
					when "01101100" => -- inc ex
						-- default
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						pc_ctrl <= latch_pc;
						-- inc
						left_ctrl <= md_left;
						right_ctrl <= plus_one_right;
						alu_ctrl <= alu_inc;
						cc_ctrl <= load_cc;
						md_ctrl <= load_md;
						ea_ctrl <= latch_ea;
						next_state <= stall1_write_state;
					when "01101101" => -- tst ex
						-- default
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						md_ctrl <= latch_md;
						pc_ctrl <= latch_pc;
						ea_ctrl <= latch_ea;
						-- tst
						left_ctrl <= md_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_tst;
						cc_ctrl <= load_cc;
						next_state <= fetch_state;
					when "01101110" => -- jmp ex
						-- default
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						md_ctrl <= latch_md;
						pc_ctrl <= load_ea_pc;
						ea_ctrl <= latch_ea;
						-- jmp
						left_ctrl <= md_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_nop;
						cc_ctrl <= latch_cc;
						next_state <= fetch_state;
					when "01101111" => -- clr ex
						-- default
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						pc_ctrl <= latch_pc;
						-- clr
						left_ctrl <= md_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_clr;
						cc_ctrl <= load_cc;
						md_ctrl <= load_md;
						ea_ctrl <= latch_ea;
						next_state <= stall1_write_state;
					when "10000000" | -- suba im
						"11000000" => -- subb im
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= inc_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- sub
						left_ctrl <= accd_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_sub8;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_hi_acca;
						accb_ctrl <= load_accb;
						next_state <= stall2_state;
					when "10000001" | -- cmpa im
						"11000001" => -- cmpb im
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= inc_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- cmp
						left_ctrl <= accd_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_sub8;
						cc_ctrl <= load_cc;
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						next_state <= stall2_state;
					when "10000010" | -- sbca im
						"11000010" => -- sbcb im
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= inc_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- sbc
						left_ctrl <= accd_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_sbc;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_hi_acca;
						accb_ctrl <= load_accb;
						next_state <= stall2_state;
					when "10000011" | -- subd im
						"11010010" | -- addd im
						"11110010" => -- ldD im
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- add
						left_ctrl <= accd_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_sub16;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_hi_acca;
						accb_ctrl <= load_accb;
						next_state <= stall2_state;
					when "10000100" | -- anda im
						"11000100" => -- andb im
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= inc_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- and
						left_ctrl <= accd_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_and;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_hi_acca;
						accb_ctrl <= load_accb;
						next_state <= stall2_state;
					when "10000101" | -- bita im
						"11000101" => -- bitb im
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= inc_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- bit
						left_ctrl <= accd_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_and;
						cc_ctrl <= load_cc;
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						next_state <= stall2_state;
					when "10000110" | -- ldaa im
						"11000110" => -- ldab im
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= inc_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- lda
						left_ctrl <= acca_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_ld8;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_hi_acca;
						accb_ctrl <= load_accb;
						next_state <= stall2_state;
					when "10001000" | -- eora im
						"11001000" => -- eorb im
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= inc_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- eor
						left_ctrl <= accd_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_eor;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_hi_acca;
						accb_ctrl <= load_accb;
						next_state <= stall2_state;
					when "10001001" | -- adca im
						"11001001" => -- adcb im
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= inc_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- adc
						left_ctrl <= accd_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_adc;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_hi_acca;
						accb_ctrl <= load_accb;
						next_state <= stall2_state;
					when "10001010" | -- oraa im
						"11001010" => -- orab im
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= inc_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- or
						left_ctrl <= accd_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_ora;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_hi_acca;
						accb_ctrl <= load_accb;
						next_state <= stall2_state;
					when "10001011" | -- adda im
						"11001011" => -- addb im
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= inc_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- add
						left_ctrl <= accd_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_add8;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_hi_acca;
						accb_ctrl <= load_accb;
						next_state <= stall2_state;
					when "10001100" | -- cpx im
						"10011100" | -- cpx im
						"10101100" => -- cpx im
						-- default
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- cpx
						left_ctrl <= ix_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_cpx;
						cc_ctrl <= load_cc;
						ix_ctrl <= load_ix;
						next_state <= stall2_state;
					when "10001101" => -- bsr
						-- default
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						pc_ctrl <= inc_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- bsr
						left_ctrl <= acca_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_nop;
						cc_ctrl <= latch_cc;
						next_state <= stall1_bsr_state;
					when "10001110" | -- lds im
						"11001110" => -- lds im
						-- default
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						ix_ctrl <= latch_ix;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- lds
						left_ctrl <= md_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_ld16;
						cc_ctrl <= load_cc;
						sp_ctrl <= load_sp;
						next_state <= stall2_state;
					when "10100000" | -- suba di
						"11100000" => -- subb di
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- sub
						left_ctrl <= accd_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_sub8;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_hi_acca;
						accb_ctrl <= load_accb;
						next_state <= stall2_state;
					when "10100001" | -- cmpa di
						"11100001" => -- cmpb di
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- cmp
						left_ctrl <= accd_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_sub8;
						cc_ctrl <= load_cc;
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						next_state <= stall2_state;
					when "10100010" | -- sbca di
						"11100010" => -- sbcb di
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- sbc
						left_ctrl <= accd_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_sbc;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_hi_acca;
						accb_ctrl <= load_accb;
						next_state <= stall2_state;
					when "10100011" => -- subd di
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- subd
						left_ctrl <= accd_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_sub16;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_hi_acca;
						accb_ctrl <= load_accb;
						next_state <= stall2_state;
					when "10100100" | -- anda di
						"11100100" => -- andb di
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- anda
						left_ctrl <= accd_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_and;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_hi_acca;
						accb_ctrl <= load_accb;
						next_state <= stall2_state;
					when "10100101" | -- bita di
						"11100101" => -- bitb di
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- bita
						left_ctrl <= accd_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_and;
						cc_ctrl <= load_cc;
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						next_state <= stall2_state;
					when "10100110" | -- ldaa di
						"11100110" => -- ldab di
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- lda
						left_ctrl <= acca_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_ld8;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_hi_acca;
						accb_ctrl <= load_accb;
						next_state <= stall2_state;
					when "10100111" | -- staa di
						"11100111" => -- stab di
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- staa
						left_ctrl <= accd_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_st8;
						cc_ctrl <= load_cc;
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						next_state <= stall1_write_state;
					when "10101000" | -- eora di
						"11101000" => -- eorb di
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- eora
						left_ctrl <= accd_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_eor;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_hi_acca;
						accb_ctrl <= load_accb;
						next_state <= stall2_state;
					when "10101001" | -- adca di
						"11101001" => -- adcb di
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- adca
						left_ctrl <= accd_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_adc;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_hi_acca;
						accb_ctrl <= load_accb;
						next_state <= stall2_state;
					when "10101010" | -- oraa di
						"11101010" => -- orab di
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- oraa
						left_ctrl <= accd_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_ora;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_hi_acca;
						accb_ctrl <= load_accb;
						next_state <= stall2_state;
					when "10101011" | -- adda di
						"11101011" => -- addb di
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- adda
						left_ctrl <= accd_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_add8;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_hi_acca;
						accb_ctrl <= load_accb;
						next_state <= stall2_state;
					when "10101100" => -- cpx di
						-- default
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- cpx
						left_ctrl <= ix_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_cpx;
						cc_ctrl <= load_cc;
						ix_ctrl <= load_ix;
						next_state <= stall2_state;
					when "10101101" => -- jsr di
						-- default
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- jsr
						left_ctrl <= acca_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_nop;
						cc_ctrl <= latch_cc;
						next_state <= stall3_jsr_state;
					when "10101110" => -- lds di
						-- default
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						ix_ctrl <= latch_ix;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- lds
						left_ctrl <= md_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_ld16;
						cc_ctrl <= load_cc;
						sp_ctrl <= load_sp;
						next_state <= stall2_state;
					when "10101111" => -- sts di
						-- default
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						ix_ctrl <= latch_ix;
						pc_ctrl <= latch_pc;
						ea_ctrl <= latch_ea;
						-- sts
						left_ctrl <= sp_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_st16;
						cc_ctrl <= load_cc;
						md_ctrl <= load_md;
						next_state <= stall1_write16_state;
					when "11100000" | -- subb ix
						"10100000" => -- suba ix
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- sub
						left_ctrl <= accd_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_sub8;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_hi_acca;
						accb_ctrl <= load_accb;
						next_state <= stall2_state;
					when "11100001" | -- cmpb ix
						"10100001" => -- cmpa ix
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- cmp
						left_ctrl <= accd_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_sub8;
						cc_ctrl <= load_cc;
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						next_state <= stall2_state;
					when "11100010" | -- sbcb ix
						"10100010" => -- sbca ix
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- sbc
						left_ctrl <= accd_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_sbc;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_hi_acca;
						accb_ctrl <= load_accb;
						next_state <= stall2_state;
					when "11100011" => -- addd ix
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- addd
						left_ctrl <= accd_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_add16;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_hi_acca;
						accb_ctrl <= load_accb;
						next_state <= stall2_state;
					when "11100100" | -- andb ix
						"10100100" => -- anda ix
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- and
						left_ctrl <= accd_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_and;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_hi_acca;
						accb_ctrl <= load_accb;
						next_state <= stall2_state;
					when "11100101" | -- bitb ix
						"10100101" => -- bita ix
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- bit
						left_ctrl <= accd_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_and;
						cc_ctrl <= load_cc;
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						next_state <= stall2_state;
					when "11100110" | -- ldab ix
						"10100110" => -- ldaa ix
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- lda
						left_ctrl <= acca_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_ld8;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_hi_acca;
						accb_ctrl <= load_accb;
						next_state <= stall2_state;
					when "11100111" | -- stab ix
						"10100111" => -- staa ix
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- staa
						left_ctrl <= accd_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_st8;
						cc_ctrl <= load_cc;
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						next_state <= stall1_write_state;
					when "11101000" | -- eorb ix
						"10101000" => -- eora ix
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- eor
						left_ctrl <= accd_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_eor;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_hi_acca;
						accb_ctrl <= load_accb;
						next_state <= stall2_state;
					when "11101001" | -- adcb ix
						"10101001" => -- adca ix
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- adc
						left_ctrl <= accd_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_adc;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_hi_acca;
						accb_ctrl <= load_accb;
						next_state <= stall2_state;
					when "11101010" | -- orab ix
						"10101010" => -- oraa ix
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- or
						left_ctrl <= accd_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_ora;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_hi_acca;
						accb_ctrl <= load_accb;
						next_state <= stall2_state;
					when "11101011" | -- addb ix
						"10101011" => -- adda ix
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- add
						left_ctrl <= accd_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_add8;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_hi_acca;
						accb_ctrl <= load_accb;
						next_state <= stall2_state;
					when "11101100" => -- ldd ix
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- ldd
						left_ctrl <= md_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_ld16;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_hi_acca;
						accb_ctrl <= load_accb;
						next_state <= stall2_state;
					when "11101101" => -- std ix
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						ea_ctrl <= latch_ea;
						-- std
						left_ctrl <= accd_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_st16;
						cc_ctrl <= load_cc;
						md_ctrl <= load_md;
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						next_state <= stall1_write16_state;
					when "11101110" => -- lds ix
						-- default
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						ix_ctrl <= latch_ix;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- lds
						left_ctrl <= md_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_ld16;
						cc_ctrl <= load_cc;
						sp_ctrl <= load_sp;
						next_state <= stall2_state;
					when "11101111" => -- sts ix
						-- default
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						ix_ctrl <= latch_ix;
						pc_ctrl <= latch_pc;
						ea_ctrl <= latch_ea;
						-- sts
						left_ctrl <= sp_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_st16;
						cc_ctrl <= load_cc;
						md_ctrl <= load_md;
						next_state <= stall1_write16_state;
					when "11110000" | -- subb ex
						"10110000" => -- suba ex
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- sub
						left_ctrl <= accd_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_sub8;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_hi_acca;
						accb_ctrl <= load_accb;
						next_state <= stall2_state;
					when "11110001" | -- cmpb ex
						"10110001" => -- cmpa ex
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- cmp
						left_ctrl <= accd_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_sub8;
						cc_ctrl <= load_cc;
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						next_state <= stall2_state;
					when "11110010" | -- sbcb ex
						"10110010" => -- sbca ex
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- sbc
						left_ctrl <= accd_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_sbc;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_hi_acca;
						accb_ctrl <= load_accb;
						next_state <= stall2_state;
					when "11110011" => -- addd ex
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- addd
						left_ctrl <= accd_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_add16;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_hi_acca;
						accb_ctrl <= load_accb;
						next_state <= stall2_state;
					when "11110100" | -- andb ex
						"10110100" => -- anda ex
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- and
						left_ctrl <= accd_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_and;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_hi_acca;
						accb_ctrl <= load_accb;
						next_state <= stall2_state;
					when "11110101" | -- bitb ex
						"10110101" => -- bita ex
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- bit
						left_ctrl <= accd_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_and;
						cc_ctrl <= load_cc;
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						next_state <= stall2_state;
					when "11110110" | -- ldab ex
						"10110110" => -- ldaa ex
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- lda
						left_ctrl <= acca_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_ld8;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_hi_acca;
						accb_ctrl <= load_accb;
						next_state <= stall2_state;
					when "11110111" | -- stab ex
						"10110111" => -- staa ex
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- staa
						left_ctrl <= accd_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_st8;
						cc_ctrl <= load_cc;
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						next_state <= stall1_write_state;
					when "11111000" | -- eorb ex
						"10111000" => -- eora ex
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- eor
						left_ctrl <= accd_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_eor;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_hi_acca;
						accb_ctrl <= load_accb;
						next_state <= stall2_state;
					when "11111001" | -- adcb ex
						"10111001" => -- adca ex
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- adc
						left_ctrl <= accd_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_adc;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_hi_acca;
						accb_ctrl <= load_accb;
						next_state <= stall2_state;
					when "11111010" | -- orab ex
						"10111010" => -- oraa ex
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- or
						left_ctrl <= accd_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_ora;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_hi_acca;
						accb_ctrl <= load_accb;
						next_state <= stall2_state;
					when "11111011" | -- addb ex
						"10111011" => -- adda ex
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- add
						left_ctrl <= accd_left;
						right_ctrl <= md_right;
						alu_ctrl <= alu_add8;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_hi_acca;
						accb_ctrl <= load_accb;
						next_state <= stall2_state;
					when "11111100" => -- ldd ex
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- ldd
						left_ctrl <= md_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_ld16;
						cc_ctrl <= load_cc;
						acca_ctrl <= load_hi_acca;
						accb_ctrl <= load_accb;
						next_state <= stall2_state;
					when "11111101" => -- std ex
						-- default
						ix_ctrl <= latch_ix;
						sp_ctrl <= latch_sp;
						pc_ctrl <= latch_pc;
						ea_ctrl <= latch_ea;
						-- std
						left_ctrl <= accd_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_st16;
						cc_ctrl <= load_cc;
						md_ctrl <= load_md;
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						next_state <= stall1_write16_state;
					when "11111110" => -- lds ex
						-- default
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						ix_ctrl <= latch_ix;
						pc_ctrl <= latch_pc;
						md_ctrl <= latch_md;
						ea_ctrl <= latch_ea;
						-- lds
						left_ctrl <= md_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_ld16;
						cc_ctrl <= load_cc;
						sp_ctrl <= load_sp;
						next_state <= stall2_state;
					when "11111111" => -- sts ex
						-- default
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						ix_ctrl <= latch_ix;
						pc_ctrl <= latch_pc;
						ea_ctrl <= latch_ea;
						-- sts
						left_ctrl <= sp_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_st16;
						cc_ctrl <= load_cc;
						md_ctrl <= load_md;
						next_state <= stall1_write16_state;
					when others => 
						-- default
						acca_ctrl <= latch_acca;
						accb_ctrl <= latch_accb;
						md_ctrl <= latch_md;
						pc_ctrl <= latch_pc;
						ea_ctrl <= latch_ea;
						-- nop
						left_ctrl <= acca_left;
						right_ctrl <= zero_right;
						alu_ctrl <= alu_nop;
						cc_ctrl <= latch_cc;
						next_state <= stall2_state;
				end case;

				--
				-- 16 bit mul
				--
			when mul_state => 
				-- default
				pc_ctrl <= latch_pc;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- mul
				left_ctrl <= accd_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_add16;
				cc_ctrl <= load_cc;
				md_ctrl <= load_md;
				acca_ctrl <= load_hi_acca;
				accb_ctrl <= load_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				-- idle the bus
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= mul0_state;

			when mul0_state => 
				-- default
				pc_ctrl <= latch_pc;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- mul add accb to md
				left_ctrl <= md_left;
				right_ctrl <= accb_right;
				alu_ctrl <= alu_add16;
				cc_ctrl <= latch_cc;
				md_ctrl <= load_md;
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				-- idle the bus
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				if md(0) = '0' then
					next_state <= mul1_state;
				else
					next_state <= mul0_state;
				end if;

			when mul1_state => 
				-- default
				pc_ctrl <= latch_pc;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- mul shift md right
				left_ctrl <= md_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_lsr16;
				cc_ctrl <= load_cc;
				md_ctrl <= load_md;
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				-- idle the bus
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				if accb(0) = '0' then
					next_state <= mul2_state;
				else
					next_state <= mul0_state;
				end if;

			when mul2_state => 
				-- default
				pc_ctrl <= latch_pc;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- mul add accb to md
				left_ctrl <= md_left;
				right_ctrl <= accb_right;
				alu_ctrl <= alu_add16;
				cc_ctrl <= latch_cc;
				md_ctrl <= load_md;
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				-- idle the bus
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= mul3_state;

			when mul3_state => 
				-- default
				pc_ctrl <= latch_pc;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- mul shift md right
				left_ctrl <= md_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_lsr16;
				cc_ctrl <= load_cc;
				md_ctrl <= load_md;
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				-- idle the bus
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				if accb(0) = '0' then
					next_state <= mul4_state;
				else
					next_state <= mul2_state;
				end if;

			when mul4_state => 
				-- default
				pc_ctrl <= latch_pc;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- mul add accb to md
				left_ctrl <= md_left;
				right_ctrl <= accb_right;
				alu_ctrl <= alu_add16;
				cc_ctrl <= latch_cc;
				md_ctrl <= load_md;
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				-- idle the bus
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= mul5_state;

			when mul5_state => 
				-- default
				pc_ctrl <= latch_pc;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- mul shift md right
				left_ctrl <= md_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_lsr16;
				cc_ctrl <= load_cc;
				md_ctrl <= load_md;
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				-- idle the bus
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				if accb(0) = '0' then
					next_state <= mul6_state;
				else
					next_state <= mul4_state;
				end if;

			when mul6_state => 
				-- default
				pc_ctrl <= latch_pc;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- mul add accb to md
				left_ctrl <= md_left;
				right_ctrl <= accb_right;
				alu_ctrl <= alu_add16;
				cc_ctrl <= latch_cc;
				md_ctrl <= load_md;
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				-- idle the bus
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= mul7_state;

			when mul7_state => 
				-- default
				pc_ctrl <= latch_pc;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- mul shift md right
				left_ctrl <= md_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_lsr16;
				cc_ctrl <= load_cc;
				md_ctrl <= load_md;
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				-- idle the bus
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= fetch_state;

				--
				-- jump to subroutine
				-- enter here from extended addressing
				--
			when jsr_state => 
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_sub16;
				cc_ctrl <= latch_cc;
				sp_ctrl <= load_sp;
				-- write pc low
				addr_ctrl <= push_ad;
				dout_ctrl <= pc_lo_dout;
				next_state <= jsr1_state;

			when jsr1_state => 
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_sub16;
				cc_ctrl <= latch_cc;
				pc_ctrl <= load_ea_pc;
				-- write pc hi
				addr_ctrl <= push_ad;
				dout_ctrl <= pc_hi_dout;
				next_state <= fetch_state;

				--
				-- branch to subroutine
				-- enter here from relative addressing
				--
			when bsr_state => 
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_sub16;
				cc_ctrl <= latch_cc;
				sp_ctrl <= load_sp;
				-- write pc low
				addr_ctrl <= push_ad;
				dout_ctrl <= pc_lo_dout;
				next_state <= bsr1_state;

			when bsr1_state => 
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_sub16;
				cc_ctrl <= latch_cc;
				pc_ctrl <= add_ea_pc;
				-- write pc hi
				addr_ctrl <= push_ad;
				dout_ctrl <= pc_hi_dout;
				next_state <= fetch_state;

				--
				-- return from subroutine
				--
			when rts_hi_state => 
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- increment sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_add16;
				cc_ctrl <= latch_cc;
				sp_ctrl <= load_sp;
				-- read pc hi
				pc_ctrl <= pull_hi_pc;
				addr_ctrl <= pull_ad;
				dout_ctrl <= pc_hi_dout;
				next_state <= rts_lo_state;

			when rts_lo_state => 
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- idle sp
				left_ctrl <= sp_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_nop;
				cc_ctrl <= latch_cc;
				sp_ctrl <= latch_sp;
				-- read pc low
				pc_ctrl <= pull_lo_pc;
				addr_ctrl <= pull_ad;
				dout_ctrl <= pc_lo_dout;
				next_state <= fetch_state;

				--
				-- return from interrupt
				-- enter here from bogus interrupts
				--
			when rti_state => 
				-- default registers
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- increment sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_add16;
				sp_ctrl <= load_sp;
				-- idle address bus
				cc_ctrl <= latch_cc;
				addr_ctrl <= idle_ad;
				dout_ctrl <= cc_dout;
				next_state <= rti_cc_state;

			when rti_cc_state => 
				-- default registers
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- increment sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_add16;
				sp_ctrl <= load_sp;
				-- read cc
				cc_ctrl <= pull_cc;
				addr_ctrl <= pull_ad;
				dout_ctrl <= cc_dout;
				next_state <= rti_accb_state;

			when rti_accb_state => 
				-- default registers
				acca_ctrl <= latch_acca;
				ix_ctrl <= latch_ix;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- increment sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_add16;
				cc_ctrl <= latch_cc;
				sp_ctrl <= load_sp;
				-- read accb
				accb_ctrl <= pull_accb;
				addr_ctrl <= pull_ad;
				dout_ctrl <= accb_dout;
				next_state <= rti_acca_state;

			when rti_acca_state => 
				-- default registers
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- increment sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_add16;
				cc_ctrl <= latch_cc;
				sp_ctrl <= load_sp;
				-- read acca
				acca_ctrl <= pull_acca;
				addr_ctrl <= pull_ad;
				dout_ctrl <= acca_dout;
				next_state <= rti_ixh_state;

			when rti_ixh_state => 
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- increment sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_add16;
				cc_ctrl <= latch_cc;
				sp_ctrl <= load_sp;
				-- read ix hi
				ix_ctrl <= pull_hi_ix;
				addr_ctrl <= pull_ad;
				dout_ctrl <= ix_hi_dout;
				next_state <= rti_ixl_state;

			when rti_ixl_state => 
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- increment sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_add16;
				cc_ctrl <= latch_cc;
				sp_ctrl <= load_sp;
				-- read ix low
				ix_ctrl <= pull_lo_ix;
				addr_ctrl <= pull_ad;
				dout_ctrl <= ix_lo_dout;
				next_state <= rti_pch_state;

			when rti_pch_state => 
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- increment sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_add16;
				cc_ctrl <= latch_cc;
				sp_ctrl <= load_sp;
				-- pull pc hi
				pc_ctrl <= pull_hi_pc;
				addr_ctrl <= pull_ad;
				dout_ctrl <= pc_hi_dout;
				next_state <= rti_pcl_state;

			when rti_pcl_state => 
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- idle sp
				left_ctrl <= sp_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_nop;
				cc_ctrl <= latch_cc;
				sp_ctrl <= latch_sp;
				-- pull pc low
				pc_ctrl <= pull_lo_pc;
				addr_ctrl <= pull_ad;
				dout_ctrl <= pc_lo_dout;
				next_state <= fetch_state;

				--
				-- here on interrupt
				-- iv register hold interrupt type
				--
			when int_pcl_state => 
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_sub16;
				cc_ctrl <= latch_cc;
				sp_ctrl <= load_sp;
				-- write pc low
				addr_ctrl <= push_ad;
				dout_ctrl <= pc_lo_dout;
				next_state <= int_pch_state;

			when int_pch_state => 
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_sub16;
				cc_ctrl <= latch_cc;
				sp_ctrl <= load_sp;
				-- write pc hi
				addr_ctrl <= push_ad;
				dout_ctrl <= pc_hi_dout;
				next_state <= int_ixl_state;

			when int_ixl_state => 
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_sub16;
				cc_ctrl <= latch_cc;
				sp_ctrl <= load_sp;
				-- write ix low
				addr_ctrl <= push_ad;
				dout_ctrl <= ix_lo_dout;
				next_state <= int_ixh_state;

			when int_ixh_state => 
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_sub16;
				cc_ctrl <= latch_cc;
				sp_ctrl <= load_sp;
				-- write ix hi
				addr_ctrl <= push_ad;
				dout_ctrl <= ix_hi_dout;
				next_state <= int_acca_state;

			when int_acca_state => 
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_sub16;
				cc_ctrl <= latch_cc;
				sp_ctrl <= load_sp;
				-- write acca
				addr_ctrl <= push_ad;
				dout_ctrl <= acca_dout;
				next_state <= int_accb_state;
			when int_accb_state => 
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_sub16;
				cc_ctrl <= latch_cc;
				sp_ctrl <= load_sp;
				-- write accb
				addr_ctrl <= push_ad;
				dout_ctrl <= accb_dout;
				next_state <= int_cc_state;

			when int_cc_state => 
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- decrement sp
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_sub16;
				cc_ctrl <= latch_cc;
				sp_ctrl <= load_sp;
				-- write cc
				addr_ctrl <= push_ad;
				dout_ctrl <= cc_dout;
				nmi_ctrl <= latch_nmi;
				--
				-- nmi is edge triggered
				-- nmi_req is cleared when nmi goes low.
				--
				if nmi_req = '1' then
					iv_ctrl <= nmi_iv;
					next_state <= int_mask_state;
				else
					--
					-- IRQ is level sensitive
					--
					if (irq = '1') and (cc(IBIT) = '0') then
						iv_ctrl <= irq_iv;
						next_state <= int_mask_state;
					else
						case op_code is
							when "00111110" => -- WAI (wait for interrupt)
								iv_ctrl <= latch_iv;
								next_state <= int_wai_state;
							when "00111111" => -- SWI (Software interrupt)
								iv_ctrl <= swi_iv;
								next_state <= vect_hi_state;
							when others => -- bogus interrupt (return)
								iv_ctrl <= latch_iv;
								next_state <= rti_state;
						end case;
					end if;
				end if;

			when int_wai_state => 
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				op_ctrl <= latch_op;
				ea_ctrl <= latch_ea;
				-- enable interrupts
				left_ctrl <= sp_left;
				right_ctrl <= plus_one_right;
				alu_ctrl <= alu_cli;
				cc_ctrl <= load_cc;
				sp_ctrl <= latch_sp;
				-- idle bus
				addr_ctrl <= idle_ad;
				dout_ctrl <= cc_dout;
				if (nmi_req = '1') and (nmi_ack = '0') then
					iv_ctrl <= nmi_iv;
					nmi_ctrl <= set_nmi;
					next_state <= vect_hi_state;
				else
					--
					-- nmi request is not cleared until nmi input goes low
					--
					if (nmi_req = '0') and (nmi_ack = '1') then
						nmi_ctrl <= reset_nmi;
					else
						nmi_ctrl <= latch_nmi;
					end if;
					--
					-- IRQ is level sensitive
					--
					if (irq = '1') and (cc(IBIT) = '0') then
						iv_ctrl <= irq_iv;
						next_state <= int_mask_state;
					else
						iv_ctrl <= latch_iv;
						next_state <= int_wai_state;
					end if;
				end if;

			when int_mask_state => 
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- Mask IRQ
				left_ctrl <= sp_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_sei;
				cc_ctrl <= load_cc;
				sp_ctrl <= latch_sp;
				-- idle bus cycle
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= vect_hi_state;

			when halt_state => -- halt CPU.
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- do nothing in ALU
				left_ctrl <= acca_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_nop;
				cc_ctrl <= latch_cc;
				-- idle bus cycle
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				if halt = '1' then
					next_state <= halt_state;
				else
					next_state <= fetch_state;
				end if;

			when stall2_state => -- Do nothing for two cycles
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- do nothing in ALU
				left_ctrl <= acca_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_nop;
				cc_ctrl <= latch_cc;
				-- idle bus cycle
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= stall1_state;

			when stall1_state => -- Do nothing for one cycle
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- do nothing in ALU
				left_ctrl <= acca_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_nop;
				cc_ctrl <= latch_cc;
				-- idle bus cycle
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= fetch_state;

			when stall2_write_state => -- Do nothing for two cycles
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- do nothing in ALU
				left_ctrl <= acca_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_nop;
				cc_ctrl <= latch_cc;
				-- idle bus cycle
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= stall1_write_state;

			when stall1_write_state => -- Do nothing for one cycle
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- do nothing in ALU
				left_ctrl <= acca_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_nop;
				cc_ctrl <= latch_cc;
				-- idle bus cycle
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= write8_state;

			when stall2_write16_state => -- Do nothing for two cycles
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- do nothing in ALU
				left_ctrl <= acca_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_nop;
				cc_ctrl <= latch_cc;
				-- idle bus cycle
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= stall1_write16_state;

			when stall1_write16_state => -- Do nothing for one cycle
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- do nothing in ALU
				left_ctrl <= acca_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_nop;
				cc_ctrl <= latch_cc;
				-- idle bus cycle
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= write16_state;

			when stall1_idx_read_state => -- Do nothing for one cycle
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- do nothing in ALU
				left_ctrl <= acca_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_nop;
				cc_ctrl <= latch_cc;
				-- idle bus cycle
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= read8_state;

			when stall3_jsr_state => -- Do nothing for three cycles
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				left_ctrl <= acca_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_nop;
				cc_ctrl <= latch_cc;
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= stall2_jsr_state;

			when stall2_jsr_state => -- Do nothing for two cycles
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				left_ctrl <= acca_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_nop;
				cc_ctrl <= latch_cc;
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= stall1_jsr_state;

			when stall1_jsr_state => -- Do nothing for one cycle
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				left_ctrl <= acca_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_nop;
				cc_ctrl <= latch_cc;
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= jsr_state;

			when stall2_bsr_state => -- Do nothing for two cycles
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				left_ctrl <= acca_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_nop;
				cc_ctrl <= latch_cc;
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= stall1_bsr_state;

			when stall1_bsr_state => -- Do nothing for one cycle
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				left_ctrl <= acca_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_nop;
				cc_ctrl <= latch_cc;
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= bsr_state;

			when others => -- error state halt on undefine states
				-- default
				acca_ctrl <= latch_acca;
				accb_ctrl <= latch_accb;
				ix_ctrl <= latch_ix;
				sp_ctrl <= latch_sp;
				pc_ctrl <= latch_pc;
				md_ctrl <= latch_md;
				iv_ctrl <= latch_iv;
				op_ctrl <= latch_op;
				nmi_ctrl <= latch_nmi;
				ea_ctrl <= latch_ea;
				-- do nothing in ALU
				left_ctrl <= acca_left;
				right_ctrl <= zero_right;
				alu_ctrl <= alu_nop;
				cc_ctrl <= latch_cc;
				-- idle bus cycle
				addr_ctrl <= idle_ad;
				dout_ctrl <= md_lo_dout;
				next_state <= error_state;
		end case;
	end process;

	--------------------------------
	--
	-- state machine
	--
	--------------------------------

	change_state : process (clk, rst, state, hold)
	begin
		if clk'event and clk = '0' then
			if rst = '1' then
				state <= reset_state;
			elsif hold = '1' then
				state <= state;
			else
				state <= next_state;
			end if;
		end if;
	end process;
	-- output

	test_alu <= out_alu;
	test_cc <= cc;

end CPU_ARCH;


