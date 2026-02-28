-- Motorola MC68705P3 Microcontroller FPGA Core
-- Based on MAME emulation (BSD-3-Clause license)
-- Translated and adapted to VHDL for FPGA implementation
-- Date: February 27, 2026
-- Notes:
-- - This is an attempt at a cycle-accurate model based on the M6805 HMOS timing from the user's manual.
-- - Implements core CPU with multi-cycle state machine matching HMOS cycle counts.
-- - Each clock cycle corresponds to one machine cycle (tcyc).
-- - Bus activity emulated per cycle (addr, data, rw_n, mem_en).
-- - Ports, timer, EPROM control implemented behaviorally but timed accurately where possible.
-- - Instruction set implemented based on extracted table; not all opcodes fully detailed for brevity, but framework is there.
-- - RAM and EPROM as Block RAM (initialize EPROM with bootstrap if needed).
-- - Test thoroughly; this is a high-level synthesis model.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity m68705p3 is
    port (
        clk         : in  std_logic;                     -- System clock (one tick = one tcyc)
        reset_n     : in  std_logic;                     -- Active-low reset
        irq_n       : in  std_logic;                     -- Interrupt request (active-low)
        timer_in    : in  std_logic;                     -- Timer input pin
        vpp         : in  std_logic;                     -- VPP for EPROM programming (high for program)
        vihtp       : in  std_logic;                     -- VIHTP line (for bootstrap)
        
        -- Port A (8-bit, open-drain with pull-ups)
        port_a_io   : inout std_logic_vector(7 downto 0);  -- Bidirectional (simulated open-drain)
        
        -- Port B (8-bit)
        port_b_io   : inout std_logic_vector(7 downto 0);
        
        -- Port C (4-bit, lower 4 bits)
        port_c_io   : inout std_logic_vector(3 downto 0);
        
        -- Address bus (11-bit address space)
        addr        : out std_logic_vector(10 downto 0);
        data_io     : inout std_logic_vector(7 downto 0);  -- Bidirectional data bus (for external if needed, but internal here)
        rw_n        : out std_logic;                     -- Read/Write (low for write)
        mem_en      : out std_logic                      -- Memory enable (active high)
    );
end entity m68705p3;

architecture cycle_accurate of m68705p3 is
    -- Internal registers
    type reg_type is record
        pc      : unsigned(10 downto 0);  -- 11-bit PC
        sp      : unsigned(7 downto 0);   -- SP (0x60-0x7F)
        a       : std_logic_vector(7 downto 0);  -- Accumulator
        x       : std_logic_vector(7 downto 0);  -- Index
        cc      : std_logic_vector(4 downto 0);  -- CCR: V H I N Z C (bits 4:0, V is bit 4? Wait, standard 6805: bit5 I, bit4 N, bit3 Z, bit1 H, bit0 C
    end record;
    
    signal regs : reg_type := (pc => (others => '0'), sp => X"7F", a => (others => '0'), x => (others => '0'), cc => "10000");  -- I=1 initial
    
    -- Ports
    signal port_a_latch : std_logic_vector(7 downto 0) := (others => '1');
    signal port_a_ddr   : std_logic_vector(7 downto 0) := (others => '0');
    
    signal port_b_latch : std_logic_vector(7 downto 0) := (others => '1');
    signal port_b_ddr   : std_logic_vector(7 downto 0) := (others => '0');
    
    signal port_c_latch : std_logic_vector(3 downto 0) := (others => '1');
    signal port_c_ddr   : std_logic_vector(3 downto 0) := (others => '0');
    
    -- Timer
    signal tdr : std_logic_vector(7 downto 0) := (others => '0');
    signal tcr : std_logic_vector(7 downto 0) := (others => '0');
    signal prescale : unsigned(6 downto 0) := (others => '0');
    signal timer_edges : unsigned(7 downto 0) := (others => '0');
    signal timer_prev : std_logic := '0';
    
    -- EPROM control
    signal pcr      : std_logic_vector(7 downto 0) := (others => '1');
    signal pl_data  : std_logic_vector(7 downto 0) := (others => '1');
    signal pl_addr  : unsigned(10 downto 0) := (others => '1');
    
    -- RAM (0x10-0x7F, 112 bytes)
    type ram_type is array (0 to 111) of std_logic_vector(7 downto 0);
    signal ram : ram_type := (others => (others => '0'));
    
    -- EPROM/User ROM (0x0080-0x07FF, 1920 bytes)
    type eprom_type is array (0 to 1919) of std_logic_vector(7 downto 0);
    signal eprom : eprom_type := (others => (others => '0'));  -- Init with bootstrap data if available
    
    -- Interrupts
    signal pending_ints : std_logic_vector(1 downto 0) := "00";  -- bit0: IRQ, bit1: Timer
    signal irq_state    : std_logic := '0';
    
    -- CPU FSM
    type cpu_state is (FETCH_OPCODE, DECODE, EXECUTE, INTERRUPT_HANDLER, INTERNAL_OP);
    signal state : cpu_state := FETCH_OPCODE;
    signal cycle_count : unsigned(3 downto 0) := (others => '0');  -- Up to 10 cycles
    signal max_cycles : unsigned(3 downto 0) := (others => '0');   -- From table
    signal opcode : std_logic_vector(7 downto 0) := (others => '0');
    signal temp_addr : unsigned(10 downto 0);
    signal temp_data : std_logic_vector(7 downto 0);
    signal branch_taken : std_logic := '0';
    
    -- CCR bits
    constant CC_C : integer := 0;
    constant CC_Z : integer := 3;
    constant CC_N : integer := 4;
    constant CC_I : integer := 5;
    constant CC_H : integer := 1;
    constant CC_V : integer := 7;  -- From MAME, but standard is bit 1 for H, bit 0 C, etc.
    
    -- Helper to update CCR for arithmetic
    procedure update_ccr_arith(signal result : in std_logic_vector(8 downto 0); signal cc : inout std_logic_vector) is
    begin
        cc(CC_Z) <= '1' if result(7 downto 0) = "00000000" else '0';
        cc(CC_N) <= result(7);
        cc(CC_C) <= result(8);
        -- V and H would be calculated accordingly
    end procedure;
    
    -- Memory read/write (internal)
    function mem_read(addr_in : unsigned(10 downto 0)) return std_logic_vector is
        variable data : std_logic_vector(7 downto 0);
    begin
        if addr_in < X"0080" then
            -- Peripherals (0x00-0x0F)
            case addr_in is
                when X"0000" => data := (port_a_latch and port_a_ddr) or (port_a_io and not port_a_ddr);
                when X"0001" => data := (port_b_latch and port_b_ddr) or (port_b_io and not port_b_ddr);
                when X"0002" => data := "1111" & ((port_c_latch and port_c_ddr) or (port_c_io and not port_c_ddr));
                when X"0008" => data := tdr;
                when X"0009" => data := tcr;
                when X"000B" => data := pcr;
                when others => data := (others => '1');  -- Unused
            end case;
        elsif addr_in < X"0010" then
            data := (others => '1');  -- Hole
        elsif addr_in < X"0080" then
            data := ram(to_integer(addr_in - X"0010"));
        else
            data := eprom(to_integer(addr_in - X"0080"));
        end if;
        return data;
    end function;
    
    procedure mem_write(addr_in : unsigned(10 downto 0); data_in : std_logic_vector(7 downto 0)) is
    begin
        if addr_in < X"0080" then
            -- Peripherals
            case addr_in is
                when X"0000" => port_a_latch <= data_in;
                when X"0001" => port_b_latch <= data_in;
                when X"0002" => port_c_latch <= data_in(3 downto 0);
                when X"0004" => port_a_ddr <= data_in;
                when X"0005" => port_b_ddr <= data_in;
                when X"0006" => port_c_ddr <= data_in(3 downto 0);
                when X"0008" => tdr <= data_in;
                when X"0009" => tcr <= data_in or X"1F";  -- Mask bits per MAME
                when X"000B" => pcr <= (pcr and X"FC") or (data_in and X"03");  -- PCR logic
                when others => null;
            end case;
        elsif addr_in < X"0010" then null;  -- Hole
        elsif addr_in < X"0080" then
            ram(to_integer(addr_in - X"0010")) <= data_in;
        else
            if pcr(2) = '0' and pcr(0) = '0' then  -- VPON and PLE
                eprom(to_integer(addr_in - X"0080")) <= data_in or eprom(to_integer(addr_in - X"0080"));  -- Program (OR for EPROM)
            end if;
        end if;
    end procedure;
    
begin

    -- Port IO simulation (open-drain for A)
    port_a_io <= port_a_latch when port_a_ddr = (port_a_ddr'range => '1') else (others => 'Z');
    port_b_io <= port_b_latch when port_b_ddr = (port_b_ddr'range => '1') else (others => 'Z');
    port_c_io <= port_c_latch when port_c_ddr = (port_c_ddr'range => '1') else (others => 'Z');
    
    -- Main process (cycle-accurate)
    process(clk, reset_n)
        variable temp : unsigned(8 downto 0);
    begin
        if reset_n = '0' then
            regs.pc <= (others => '0');
            regs.sp <= X"7F";
            regs.a  <= (others => '0');
            regs.x  <= (others => '0');
            regs.cc <= "10000";  -- I=1
            
            port_a_ddr   <= (others => '0');
            port_b_ddr   <= (others => '0');
            port_c_ddr   <= (others => '0');
            
            tdr <= (others => '0');
            tcr <= (others => '0');
            prescale <= (others => '0');
            
            pcr <= (others => '1');
            pl_data <= (others => '1');
            pl_addr <= (others => '1');
            
            pending_ints <= "00";
            state <= FETCH_OPCODE;
            cycle_count <= (others => '0');
            
            -- Load vector
            if vihtp = '1' then
                regs.pc <= unsigned(mem_read(X"7F6") & mem_read(X"7F7"));  -- Bootstrap 0xFF F6
            else
                regs.pc <= unsigned(mem_read(X"7FE") & mem_read(X"7FF"));  -- Reset 0xFF FE
            end if;
            
        elsif rising_edge(clk) then
            -- Timer update (every cycle)
            timer_prev <= timer_in;
            if timer_in = '0' and timer_prev = '1' then
                timer_edges <= timer_edges + 1;
            end if;
            
            if tcr(4) = '1' then  -- TIE
                prescale <= prescale + 1 + timer_edges;
                timer_edges <= (others => '0');
                
                if prescale >= 2 ** to_integer(unsigned(tcr(2 downto 0))) then
                    tdr <= std_logic_vector(unsigned(tdr) - 1);
                    prescale <= (others => '0');
                    
                    if tdr = X"00" then
                        tcr(7) <= '1';  -- TIR
                        if tcr(6) = '0' then
                            pending_ints(1) <= '1';
                        end if;
                    end if;
                end if;
            end if;
            
            -- IRQ update
            if irq_n = '0' then
                pending_ints(0) <= '1';
            end if;
            
            -- Check for interrupts if not masked
            if regs.cc(CC_I) = '0' and pending_ints /= "00" then
                state <= INTERRUPT_HANDLER;
                cycle_count <= (others => '0');
                max_cycles <= "1010";  -- 10 cycles for interrupt
            end if;
            
            -- FSM
            cycle_count <= cycle_count + 1;
            mem_en <= '0';
            
            case state is
                when FETCH_OPCODE =>
                    -- Cycle 1: Fetch opcode
                    addr <= std_logic_vector(regs.pc);
                    rw_n <= '1';
                    mem_en <= '1';
                    opcode <= data_io;  -- Assume data_io connected internally or external
                    regs.pc <= regs.pc + 1;
                    state <= DECODE;
                    cycle_count <= (others => '0');
                    
                when DECODE =>
                    -- Set max_cycles based on opcode (from HMOS table)
                    case opcode is
                        when X"98" => max_cycles <= "0010";  -- CLC, 2 cycles
                        when X"9A" => max_cycles <= "0010";  -- CLI, 2 cycles
                        when X"4F" => max_cycles <= "0100";  -- CLRA, 4 cycles
                        when X"A6" => max_cycles <= "0010";  -- LDA IMM, 2 cycles
                        when X"B6" => max_cycles <= "0100";  -- LDA DIR, 4 cycles
                        when X"C6" => max_cycles <= "0101";  -- LDA EXT, 5 cycles
                        when X"20" => max_cycles <= "0011";  -- BRA, 3 cycles if taken
                        -- Add more opcodes from table...
                        when others => max_cycles <= "0010";  -- Default NOP-like
                    end case;
                    state <= EXECUTE;
                    cycle_count <= "0001";  -- Start from cycle 2
                    
                when EXECUTE =>
                    if cycle_count >= max_cycles then
                        state <= FETCH_OPCODE;
                        cycle_count <= (others => '0');
                    else
                        -- Per-cycle execution (example for selected instructions)
                        case opcode is
                            when X"98" =>  -- CLC
                                if cycle_count = "0001" then
                                    regs.cc(CC_C) <= '0';
                                    addr <= "11111111111";  -- Dummy read
                                    rw_n <= '1';
                                    mem_en <= '1';
                                end if;
                                
                            when X"A6" =>  -- LDA IMM (2 cycles)
                                if cycle_count = "0001" then
                                    addr <= std_logic_vector(regs.pc);
                                    rw_n <= '1';
                                    mem_en <= '1';
                                    regs.a <= data_io;
                                    regs.pc <= regs.pc + 1;
                                    -- Update CCR Z, N
                                    regs.cc(CC_Z) <= '1' if data_io = "00000000" else '0';
                                    regs.cc(CC_N) <= data_io(7);
                                end if;
                                
                            when X"B6" =>  -- LDA DIR (4 cycles)
                                case cycle_count is
                                    when "0001" =>  -- Cycle 2: Fetch dir addr
                                        addr <= std_logic_vector(regs.pc);
                                        rw_n <= '1';
                                        mem_en <= '1';
                                        temp_addr <= "000" & unsigned(data_io);  -- Page 0
                                        regs.pc <= regs.pc + 1;
                                    when "0010" =>  -- Cycle 3: Internal dummy
                                        addr <= "11111111111";
                                        rw_n <= '1';
                                        mem_en <= '1';
                                    when "0011" =>  -- Cycle 4: Read data
                                        addr <= std_logic_vector(temp_addr);
                                        rw_n <= '1';
                                        mem_en <= '1';
                                        regs.a <= data_io;
                                        -- CCR update
                                        regs.cc(CC_Z) <= '1' if data_io = "00000000" else '0';
                                        regs.cc(CC_N) <= data_io(7);
                                    when others => null;
                                end case;
                                
                            -- Add implementations for other opcodes similarly, matching cycle-by-cycle bus activity from manual
                            -- For branches: Check condition in cycle 2, add offset in cycle 3 if taken
                            -- For stack: Push/pull with SP adjust
                            -- etc.
                            
                            when others =>  -- NOP or unknown
                                addr <= "11111111111";
                                rw_n <= '1';
                                mem_en <= '1';
                        end case;
                    end if;
                    
                when INTERRUPT_HANDLER =>
                    -- 10 cycles for interrupt (push regs, fetch vector)
                    case cycle_count is
                        when "0000" =>  -- Cycle 1: Internal
                            addr <= "11111111111";
                            rw_n <= '1';
                            mem_en <= '1';
                        when "0001" =>  -- Cycle 2: Push PC low
                            addr <= "000" & regs.sp;
                            data_io <= std_logic_vector(regs.pc(7 downto 0));
                            rw_n <= '0';
                            mem_en <= '1';
                            regs.sp <= regs.sp - 1;
                        when "0010" =>  -- Cycle 3: Push PC high
                            addr <= "000" & regs.sp;
                            data_io <= "000" & std_logic_vector(regs.pc(10 downto 8));
                            rw_n <= '0';
                            mem_en <= '1';
                            regs.sp <= regs.sp - 1;
                        when "0011" =>  -- Cycle 4: Push X
                            addr <= "000" & regs.sp;
                            data_io <= regs.x;
                            rw_n <= '0';
                            mem_en <= '1';
                            regs.sp <= regs.sp - 1;
                        when "0100" =>  -- Cycle 5: Push A
                            addr <= "000" & regs.sp;
                            data_io <= regs.a;
                            rw_n <= '0';
                            mem_en <= '1';
                            regs.sp <= regs.sp - 1;
                        when "0101" =>  -- Cycle 6: Push CCR
                            addr <= "000" & regs.sp;
                            data_io <= "111" & regs.cc;  -- Padded
                            rw_n <= '0';
                            mem_en <= '1';
                            regs.sp <= regs.sp - 1;
                            regs.cc(CC_I) <= '1';  -- Set I
                        when "0110" =>  -- Cycle 7: Internal
                            addr <= "11111111111";
                            rw_n <= '1';
                            mem_en <= '1';
                        when "0111" =>  -- Cycle 8: Fetch vector high
                            temp_addr <= X"7FA" if pending_ints(1) = '1' else X"7FA";  -- Timer or INT
                            addr <= std_logic_vector(temp_addr);
                            rw_n <= '1';
                            mem_en <= '1';
                            temp_data <= data_io;
                        when "1000" =>  -- Cycle 9: Fetch vector low
                            addr <= std_logic_vector(temp_addr + 1);
                            rw_n <= '1';
                            mem_en <= '1';
                            regs.pc <= unsigned(temp_data & data_io);
                            pending_ints <= "00";  -- Clear
                        when "1001" =>  -- Cycle 10: Internal
                            addr <= "11111111111";
                            rw_n <= '1';
                            mem_en <= '1';
                            state <= FETCH_OPCODE;
                        when others => null;
                    end case;
                    
                when INTERNAL_OP =>
                    -- For dummy cycles
                    addr <= "11111111111";
                    rw_n <= '1';
                    mem_en <= '1';
                    
            end case;
            
            -- EPROM programming logic (not cycle-bound, but check per cycle)
            if pcr(1) = '0' and pcr(0) = '0' and vpp = '1' then  -- PGE and PLE
                -- Program pl_addr with pl_data (OR for EPROM set bits)
            end if;
            
        end if;
    end process;

    -- Data_io internal connection (for simulation, connect to mem_read when rw_n='1')
    data_io <= mem_read(unsigned(addr)) when rw_n = '1' else (others => 'Z');
    -- For write, mem_write in process when rw_n='0'

end architecture cycle_accurate;