//============================================================================
//
// Qix Audio Board
// Copyright (C) 2026 Rodimus
//
// Hardware: MC6802 audio CPU (cpu68 core) + 2× PIA6821 + 8-bit DAC +
//           discrete stereo volume attenuator
//
// Responsibilities:
//   - Audio CPU (~0.91 MHz, cpu68 with hold-gated clocking at 20 MHz)
//   - Receives sound commands from data CPU via sndPIA1 port A
//   - 8-bit unsigned DAC via sndPIA1 port B
//   - Stereo volume scaling from data CPU sndPIA0 port B
//
//============================================================================

module Qix_Sound (
    input         clk_20m,
    input         reset,

    // Communication with data CPU (via sndPIA0 on data CPU side)
    input  [7:0]  snd_data_in,      // data from data CPU → sndPIA1 port A input
    output [7:0]  snd_data_out,     // sndPIA1 port A output → data CPU
    input         snd_irq_from_cpu, // data CPU sndPIA0 CA2 → our sndPIA1 CA1
    output        snd_irq_to_cpu,   // our sndPIA1 CA2 → data CPU sndPIA0 CA1

    // Volume control (sndPIA0 port B on data CPU, passed through)
    input  [7:0]  vol_data,         // [7:4] = left vol index, [3:0] = right vol index

    // Audio output (signed 16-bit stereo)
    output signed [15:0] audio_l,
    output signed [15:0] audio_r,

    // ROM loading (MiSTer ioctl — pre-gated by address range in Qix.sv)
    input  [24:0] ioctl_addr,
    input  [7:0]  ioctl_data,
    input         ioctl_wr,

    input         pause
);

// ---------------------------------------------------------------------------
// Audio CPU bus signals
// ---------------------------------------------------------------------------
wire [15:0] snd_A;
wire [7:0]  snd_Dout;
wire [7:0]  snd_Din;           // assigned at bottom by read mux
wire        snd_rw;            // 1 = read, 0 = write (active-high read, directly from cpu68)
wire        snd_vma;           // 1 = valid memory address (active bus cycle)
wire        snd_wr = ~snd_rw;  // convenience: 1 = write

// ---------------------------------------------------------------------------
// Clock enable: ~0.91 MHz (20 MHz ÷ 22 = 909 kHz, within 1.6% of 895 kHz target)
//
// cpu68 advances state on the falling edge of clk when hold=0.
// We assert hold on all cycles except one tick per divide period.
// Pause is applied by forcing hold=1 permanently.
// ---------------------------------------------------------------------------
reg [4:0] snd_div;
always @(posedge clk_20m)
    if (snd_div == 5'd21) snd_div <= 5'd0;
    else                  snd_div <= snd_div + 5'd1;

wire snd_cen_raw = (snd_div == 5'd0);
wire snd_cen     = snd_cen_raw & ~pause;
wire snd_hold    = ~snd_cen;     // hold CPU except during active tick

// ---------------------------------------------------------------------------
// Address decoder — qualified by VMA from cpu68
// ---------------------------------------------------------------------------
wire sndpia2_cs_addr = (snd_A[15:13] == 3'b001);   // $2000-$3FFF
wire sndpia1_cs_addr = (snd_A[15:14] == 2'b01);    // $4000-$7FFF
wire rom_cs          = (snd_A >= 16'hD000);        // $D000-$FFFF (12KB)

// PIA enables: active during valid bus cycles at the active CPU tick
// cpu68 drives vma=1 when the bus cycle is real (not idle/dead cycle)
wire sndpia2_en = snd_cen & snd_vma & sndpia2_cs_addr;
wire sndpia1_en = snd_cen & snd_vma & sndpia1_cs_addr;

// ---------------------------------------------------------------------------
// 6802 internal RAM — 128 bytes ($0000-$007F)
// cpu68 is a pure CPU core and does NOT include internal RAM.
// ---------------------------------------------------------------------------
reg [7:0] internal_ram [0:127];
wire internal_ram_cs   = (snd_A[15:7] == 9'd0);    // $0000-$007F
wire [7:0] internal_ram_dout = internal_ram[snd_A[6:0]];

always @(posedge clk_20m)
    if (snd_cen && snd_vma && snd_wr && internal_ram_cs)
        internal_ram[snd_A[6:0]] <= snd_Dout;

// ---------------------------------------------------------------------------
// cpu68 — MC6800/6802 compatible audio CPU (John Kent, OpenCores)
//   rst      : active-high synchronous reset
//   hold     : active-high, freezes state machine (used for clock gating)
//   halt     : active-high, halts CPU at next fetch (active high)
//   irq      : active-high IRQ
//   nmi      : active-high NMI — tie low (not used by Qix)
//   rw       : active-high read (directly drives PIAs)
//   vma      : valid memory address — 1 when bus cycle is real
//
// NOTE: cpu68 advances on the FALLING edge of clk.
//       hold=1 freezes it. We gate hold to get ~909 kHz effective rate.
// ---------------------------------------------------------------------------
wire snd_irq;

cpu68 audio_cpu (
    .clk      (clk_20m),
    .rst      (reset),
    .rw       (snd_rw),
    .vma      (snd_vma),
    .address  (snd_A),
    .data_in  (snd_Din),
    .data_out (snd_Dout),
    .hold     (snd_hold),
    .halt     (1'b0),
    .irq      (snd_irq),
    .nmi      (1'b0),
    .test_alu (),
    .test_cc  ()
);

// ---------------------------------------------------------------------------
// sndPIA1 ($4000-$7FFF) — main sound PIA
//   Port A: bidirectional comm with data CPU
//   Port B: 8-bit DAC value (written by audio CPU)
//   CA1: interrupt from data CPU (data CPU sndPIA0 CA2)
//   CA2: interrupt to data CPU  (data CPU sndPIA0 CA1)
// ---------------------------------------------------------------------------
wire [7:0] sndpia1_dout;
wire [7:0] sndpia1_pa_o,  sndpia1_pa_oe;
wire [7:0] sndpia1_pb_o,  sndpia1_pb_oe;
wire       sndpia1_ca2_o, sndpia1_ca2_oe;
wire       sndpia1_cb2_o, sndpia1_cb2_oe;
wire       sndpia1_irqa,  sndpia1_irqb;

pia6821 sndpia1 (
    .clk      (clk_20m),
    .rst      (reset),
    .cs       (sndpia1_en),
    .rw       (snd_rw),
    .addr     (snd_A[1:0]),
    .data_in  (snd_Dout),
    .data_out (sndpia1_dout),
    .irqa     (sndpia1_irqa),
    .irqb     (sndpia1_irqb),
    .pa_i     (snd_data_in),      // data from data CPU
    .pa_o     (sndpia1_pa_o),
    .pa_oe    (sndpia1_pa_oe),
    .ca1      (snd_irq_from_cpu), // CA2 from data CPU sndPIA0
    .ca2_i    (1'b1),
    .ca2_o    (sndpia1_ca2_o),
    .ca2_oe   (sndpia1_ca2_oe),
    .pb_i     (8'h00),
    .pb_o     (sndpia1_pb_o),     // 8-bit DAC value
    .pb_oe    (sndpia1_pb_oe),
    .cb1      (1'b0),
    .cb2_i    (1'b1),
    .cb2_o    (sndpia1_cb2_o),
    .cb2_oe   (sndpia1_cb2_oe)
);

assign snd_data_out   = sndpia1_pa_o;
assign snd_irq_to_cpu = sndpia1_ca2_o;

// ---------------------------------------------------------------------------
// sndPIA2 ($2000-$3FFF) — TMS5220 PIA (mapped per MAME, never accessed)
// ---------------------------------------------------------------------------
wire [7:0] sndpia2_dout;
wire       sndpia2_irqa, sndpia2_irqb;

pia6821 sndpia2 (
    .clk      (clk_20m),
    .rst      (reset),
    .cs       (sndpia2_en),
    .rw       (snd_rw),
    .addr     (snd_A[1:0]),
    .data_in  (snd_Dout),
    .data_out (sndpia2_dout),
    .irqa     (sndpia2_irqa),
    .irqb     (sndpia2_irqb),
    .pa_i     (8'hFF),
    .pa_o     (),
    .pa_oe    (),
    .ca1      (1'b0),
    .ca2_i    (1'b1),
    .ca2_o    (),
    .ca2_oe   (),
    .pb_i     (8'hFF),
    .pb_o     (),
    .pb_oe    (),
    .cb1      (1'b0),
    .cb2_i    (1'b1),
    .cb2_o    (),
    .cb2_oe   ()
);

// IRQ to audio CPU: OR of all PIA interrupt outputs (active-high)
assign snd_irq = sndpia1_irqa | sndpia1_irqb | sndpia2_irqa | sndpia2_irqb;

// ---------------------------------------------------------------------------
// Audio ROM — 12KB ($D000-$FFFF) in 12KB BRAM
// Loaded at ioctl_addr $0C000-$0EFFF (gated by Qix.sv).
// CPU read address: snd_A[13:0] - $1000  ($D000→0 .. $FFFF→$2FFF)
// ioctl write address: ioctl_addr[13:0] (bits [13:0] of $0C000-$0EFFF = 0-$2FFF)
// ---------------------------------------------------------------------------
reg [7:0] snd_rom [0:12287];                          // 12KB
reg [7:0] rom_dout;

wire [13:0] rom_cpu_addr   = snd_A[13:0] - 14'h1000;  // $D000→0
wire [13:0] rom_ioctl_addr = ioctl_addr[13:0];

always @(posedge clk_20m) begin
    if (ioctl_wr)
        snd_rom[rom_ioctl_addr] <= ioctl_data;
    rom_dout <= snd_rom[rom_cpu_addr];
end

// ---------------------------------------------------------------------------
// CPU data bus read mux — default $FF for unmapped regions
// Internal 6802 RAM ($0000-$007F) handled externally since cpu68 has no
// built-in RAM (unlike the real MC6802 silicon).
// ---------------------------------------------------------------------------
assign snd_Din =
    internal_ram_cs ? internal_ram_dout :
    sndpia2_cs_addr ? sndpia2_dout      :
    sndpia1_cs_addr ? sndpia1_dout      :
    rom_cs          ? rom_dout          :
    8'hFF;

// ---------------------------------------------------------------------------
// DAC + Stereo Volume Attenuation
//
// vol_table maps a 4-bit index (from vol_data) to an 8-bit scale factor.
//   Index 0 = full volume (255), index 15 = minimum (24).
//   Derived from the parallel resistor network in qix_a.cpp (MAME).
//
// Scaled output = (dac_val × vol_scale) ÷ 256  (upper byte of 16-bit product)
// MiSTer signed 16-bit: {scaled_byte, 8'h00} − 0x8000
// ---------------------------------------------------------------------------
wire [7:0] dac_val = sndpia1_pb_o;

reg [7:0] vol_table [0:15];
initial begin
    vol_table[0]  = 8'd255; vol_table[1]  = 8'd200;
    vol_table[2]  = 8'd160; vol_table[3]  = 8'd140;
    vol_table[4]  = 8'd128; vol_table[5]  = 8'd112;
    vol_table[6]  = 8'd100; vol_table[7]  = 8'd90;
    vol_table[8]  = 8'd80;  vol_table[9]  = 8'd72;
    vol_table[10] = 8'd64;  vol_table[11] = 8'd56;
    vol_table[12] = 8'd48;  vol_table[13] = 8'd40;
    vol_table[14] = 8'd32;  vol_table[15] = 8'd24;
end

wire [7:0] vol_l = vol_table[vol_data[7:4]];
wire [7:0] vol_r = vol_table[vol_data[3:0]];

// 16-bit unsigned intermediate: (0-255) × (0-255) >> 8 → 0-254 in [7:0]
wire [15:0] dac_l_scaled = ({8'd0, dac_val} * {8'd0, vol_l}) >> 8;
wire [15:0] dac_r_scaled = ({8'd0, dac_val} * {8'd0, vol_r}) >> 8;

// Unsigned 8-bit → signed 16-bit: shift to upper byte, subtract midpoint
assign audio_l = {dac_l_scaled[7:0], 8'h00} - 16'h8000;
assign audio_r = {dac_r_scaled[7:0], 8'h00} - 16'h8000;

endmodule
