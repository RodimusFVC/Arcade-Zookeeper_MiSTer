//============================================================================
//
// Qix Data CPU Board
// Copyright (C) 2026 Rodimus
//
// Hardware: mc6809e + 3× PIA6821 + 1× sndPIA0 + 1KB local RAM + 24KB ROM
//
// Responsibilities:
//   - Game logic CPU (6809E @ ~1.25 MHz, E/Q clocks supplied by top-level)
//   - Player 1/2 inputs + coin via PIAs
//   - Audio communication via sndPIA0
//   - Communicates with Video CPU via shared RAM port A + FIRQ cross-signals
//
//============================================================================

module Qix_CPU (
    input         clk_20m,
    input         reset,
    input         E,              // shared 6809 E clock
    input         Q,              // shared 6809 Q clock

    // Shared RAM — port A of dual-port RAM in Qix.sv
    output [9:0]  shared_addr,
    output [7:0]  shared_din,
    input  [7:0]  shared_dout,
    output        shared_we,

    // FIRQ cross-signals
    output        video_firq,     // assert FIRQ on video CPU (active-high pulse)
    input         data_firq_n,    // FIRQ input from video CPU (active-low)

    // Player inputs (active-low, from wrapper)
    input  [7:0]  p1_input,       // PIA0 port A: player 1 joystick + buttons
    input  [7:0]  coin_input,     // PIA0 port B: coin / service switches
    input  [7:0]  spare_input,    // PIA1 port A: spare (unused on base Qix)
    input  [7:0]  in0_input,      // PIA1 port B: spare (unused on base Qix)
    input  [7:0]  p2_input,       // PIA2 port A: player 2 joystick

    // Sound PIA interface (to/from Qix_Sound)
    output [7:0]  snd_data_out,   // sndPIA0 port A output → sound CPU
    output [7:0]  snd_vol_out,    // sndPIA0 port B output → stereo volume
    output        snd_irq_to_snd, // CA2 → sound CPU interrupt
    input         snd_irq_from_snd,// CA1 ← sound CPU interrupt
    output        flip_screen,    // CB2 → cocktail flip

    // ROM loading (MiSTer ioctl — pre-gated by address range in Qix.sv)
    input  [24:0] ioctl_addr,
    input  [7:0]  ioctl_data,
    input         ioctl_wr,

    input         pause
);

// ---------------------------------------------------------------------------
// 6809E bus signals (declared early; driven by mc6809e instance below)
// ---------------------------------------------------------------------------
wire [15:0] cpu_A;
wire [7:0]  cpu_Dout;
wire        cpu_RnW;
wire [7:0]  cpu_Din;    // read mux output, assigned at bottom
wire        n_irq;      // active-low IRQ to 6809E, driven by sndPIA0

// ---------------------------------------------------------------------------
// Write strobe: one-cycle pulse on falling edge of E while RnW is low
// ---------------------------------------------------------------------------
reg E_prev;
always @(posedge clk_20m) E_prev <= E;
wire cpu_E_fall = E_prev & ~E;          // 1-cycle pulse at every E falling edge
wire cpu_wr     = cpu_E_fall & ~cpu_RnW;

// ---------------------------------------------------------------------------
// Address decoder
// ---------------------------------------------------------------------------
wire shared_cs      = (cpu_A[15:10] == 6'b10_0000);   // $8000-$83FF
wire local_cs       = (cpu_A[15:10] == 6'b10_0001);   // $8400-$87FF
wire acia_cs        = (cpu_A[15:10] == 6'b10_0010);   // $8800-$8BFF (open-bus)
wire firq_range     = (cpu_A[15:10] == 6'b10_0011);   // $8C00-$8FFF
wire firq_assert_cs = firq_range & ~cpu_A[0];          // even addr: assert video FIRQ
wire firq_ack_cs    = firq_range &  cpu_A[0];          // odd  addr: ack data FIRQ
wire sndpia_cs      = (cpu_A[15:10] == 6'b10_0100);   // $9000-$93FF
wire pia0_cs        = (cpu_A[15:10] == 6'b10_0101);   // $9400-$97FF
wire pia1_cs        = (cpu_A[15:10] == 6'b10_0110);   // $9800-$9BFF
wire pia2_cs        = (cpu_A[15:10] == 6'b10_0111);   // $9C00-$9FFF
wire rom_cs         = (cpu_A[15:14] == 2'b11);               // $C000-$FFFF

// PIA chip-select: single-cycle pulse at E-fall so the synchronous PIA
// fires exactly once per bus cycle regardless of E-cycle width.
wire sndpia_en = cpu_E_fall & sndpia_cs;
wire pia0_en   = cpu_E_fall & pia0_cs;
wire pia1_en   = cpu_E_fall & pia1_cs;
wire pia2_en   = cpu_E_fall & pia2_cs;

// ---------------------------------------------------------------------------
// FIRQ latch (data CPU side)
//   SET  : falling edge of data_firq_n (video CPU asserts)
//   CLEAR: any E-qualified bus access to odd address in $8C00-$8FFF ($8C01)
// ---------------------------------------------------------------------------
reg firq_latch;
reg data_firq_n_prev;

always @(posedge clk_20m) begin
    data_firq_n_prev <= data_firq_n;
    if (reset) begin
        firq_latch <= 1'b0;
    end else if (cpu_E_fall & firq_ack_cs) begin
        firq_latch <= 1'b0;             // CPU acks its own FIRQ
    end else if (~data_firq_n & data_firq_n_prev) begin
        firq_latch <= 1'b1;             // video CPU fired FIRQ (falling edge)
    end
end

wire n_firq = ~firq_latch;             // active-low FIRQ input to 6809E

// ---------------------------------------------------------------------------
// video_firq: one-cycle pulse on any bus access to $8C00 (even addr in range)
// ---------------------------------------------------------------------------
reg video_firq_r;
always @(posedge clk_20m) begin
    if (reset) video_firq_r <= 1'b0;
    else       video_firq_r <= cpu_E_fall & firq_assert_cs;
end
assign video_firq = video_firq_r;

// ---------------------------------------------------------------------------
// 6809E Data CPU
// ---------------------------------------------------------------------------
mc6809e data_cpu (
    .D      (cpu_Din),
    .DOut   (cpu_Dout),
    .ADDR   (cpu_A),
    .RnW    (cpu_RnW),
    .E      (E),
    .Q      (Q),
    .nIRQ   (n_irq),
    .nFIRQ  (n_firq),
    .nNMI   (1'b1),
    .BS     (),
    .BA     (),
    .AVMA   (),
    .BUSY   (),
    .LIC    (),
    .nHALT  (~pause),
    .nRESET (~reset)
);

// ---------------------------------------------------------------------------
// Shared RAM — port A passthrough (dual-port RAM lives in Qix.sv)
// ---------------------------------------------------------------------------
assign shared_addr = cpu_A[9:0];
assign shared_din  = cpu_Dout;
assign shared_we   = shared_cs & cpu_wr;

// ---------------------------------------------------------------------------
// Local RAM — 1KB BRAM ($8400-$87FF)
// ---------------------------------------------------------------------------
reg [7:0] local_ram [0:1023];
reg [7:0] local_ram_dout;

always @(posedge clk_20m) begin
    if (local_cs & cpu_wr)
        local_ram[cpu_A[9:0]] <= cpu_Dout;
    local_ram_dout <= local_ram[cpu_A[9:0]];
end

// ---------------------------------------------------------------------------
// sndPIA0 ($9000-$93FF) — data CPU ↔ sound CPU
//   Port A: command data out to sound CPU
//   Port B: stereo volume (4-bit L + 4-bit R)
//   CA1: snd_irq_from_snd  CA2: snd_irq_to_snd
//   CB2: flip_screen (cocktail)
// ---------------------------------------------------------------------------
wire [7:0] sndpia_dout;
wire [7:0] sndpia_pa_o, sndpia_pa_oe;
wire [7:0] sndpia_pb_o, sndpia_pb_oe;
wire       sndpia_ca2_o, sndpia_ca2_oe;
wire       sndpia_cb2_o, sndpia_cb2_oe;
wire       sndpia_irqa, sndpia_irqb;

pia6821 sndpia0 (
    .clk      (clk_20m),
    .rst      (reset),
    .cs       (sndpia_en),
    .rw       (cpu_RnW),
    .addr     (cpu_A[1:0]),
    .data_in  (cpu_Dout),
    .data_out (sndpia_dout),
    .irqa     (sndpia_irqa),
    .irqb     (sndpia_irqb),
    .pa_i     (8'h00),
    .pa_o     (sndpia_pa_o),
    .pa_oe    (sndpia_pa_oe),
    .ca1      (snd_irq_from_snd),
    .ca2_i    (1'b1),
    .ca2_o    (sndpia_ca2_o),
    .ca2_oe   (sndpia_ca2_oe),
    .pb_i     (8'h00),
    .pb_o     (sndpia_pb_o),
    .pb_oe    (sndpia_pb_oe),
    .cb1      (1'b0),
    .cb2_i    (1'b1),
    .cb2_o    (sndpia_cb2_o),
    .cb2_oe   (sndpia_cb2_oe)
);

assign snd_data_out   = sndpia_pa_o;
assign snd_vol_out    = sndpia_pb_o;
assign snd_irq_to_snd = sndpia_ca2_o;
assign flip_screen    = sndpia_cb2_o;

// IRQ to data CPU: active-low merge of sndPIA0 IRQA and IRQB
assign n_irq = ~(sndpia_irqa | sndpia_irqb);

// ---------------------------------------------------------------------------
// PIA0 ($9400-$97FF) — player 1 joystick + coin inputs
// ---------------------------------------------------------------------------
wire [7:0] pia0_dout;
wire       pia0_irqa, pia0_irqb;

pia6821 pia0 (
    .clk      (clk_20m),
    .rst      (reset),
    .cs       (pia0_en),
    .rw       (cpu_RnW),
    .addr     (cpu_A[1:0]),
    .data_in  (cpu_Dout),
    .data_out (pia0_dout),
    .irqa     (pia0_irqa),
    .irqb     (pia0_irqb),
    .pa_i     (p1_input),
    .pa_o     (),
    .pa_oe    (),
    .ca1      (1'b1),
    .ca2_i    (1'b1),
    .ca2_o    (),
    .ca2_oe   (),
    .pb_i     (coin_input),
    .pb_o     (),
    .pb_oe    (),
    .cb1      (1'b1),
    .cb2_i    (1'b1),
    .cb2_o    (),
    .cb2_oe   ()
);

// ---------------------------------------------------------------------------
// PIA1 ($9800-$9BFF) — spare inputs (unused on base Qix)
// ---------------------------------------------------------------------------
wire [7:0] pia1_dout;
wire       pia1_irqa, pia1_irqb;

pia6821 pia1 (
    .clk      (clk_20m),
    .rst      (reset),
    .cs       (pia1_en),
    .rw       (cpu_RnW),
    .addr     (cpu_A[1:0]),
    .data_in  (cpu_Dout),
    .data_out (pia1_dout),
    .irqa     (pia1_irqa),
    .irqb     (pia1_irqb),
    .pa_i     (spare_input),
    .pa_o     (),
    .pa_oe    (),
    .ca1      (1'b1),
    .ca2_i    (1'b1),
    .ca2_o    (),
    .ca2_oe   (),
    .pb_i     (in0_input),
    .pb_o     (),
    .pb_oe    (),
    .cb1      (1'b1),
    .cb2_i    (1'b1),
    .cb2_o    (),
    .cb2_oe   ()
);

// ---------------------------------------------------------------------------
// PIA2 ($9C00-$9FFF) — player 2 joystick + coin counters/lockout
// ---------------------------------------------------------------------------
wire [7:0] pia2_dout;
wire       pia2_irqa, pia2_irqb;

pia6821 pia2 (
    .clk      (clk_20m),
    .rst      (reset),
    .cs       (pia2_en),
    .rw       (cpu_RnW),
    .addr     (cpu_A[1:0]),
    .data_in  (cpu_Dout),
    .data_out (pia2_dout),
    .irqa     (pia2_irqa),
    .irqb     (pia2_irqb),
    .pa_i     (p2_input),
    .pa_o     (),
    .pa_oe    (),
    .ca1      (1'b1),
    .ca2_i    (1'b1),
    .ca2_o    (),
    .ca2_oe   (),
    .pb_i     (8'h00),
    .pb_o     (),           // coin counters / lockout (unconnected for now)
    .pb_oe    (),
    .cb1      (1'b1),
    .cb2_i    (1'b1),
    .cb2_o    (),
    .cb2_oe   ()
);

// ---------------------------------------------------------------------------
// Data CPU ROM — 16KB ($C000-$FFFF) in 16KB BRAM
//
// Loaded at ioctl_addr $00000-$03FFF (gated by Qix.sv).
// CPU read address: cpu_A[13:0]  ($C000→0 .. $FFFF→$3FFF)
// ioctl write address: ioctl_addr[13:0] (0-based, base $00000)
// ---------------------------------------------------------------------------
reg [7:0] data_rom [0:16383];
reg [7:0] rom_dout;

wire [13:0] rom_cpu_addr   = cpu_A[13:0];
wire [13:0] rom_ioctl_addr = ioctl_addr[13:0];

always @(posedge clk_20m) begin
    if (ioctl_wr)
        data_rom[rom_ioctl_addr] <= ioctl_data;
    rom_dout <= data_rom[rom_cpu_addr];
end

// ---------------------------------------------------------------------------
// CPU data bus read mux — default $FF for open-bus / unimplemented reads
// ---------------------------------------------------------------------------
assign cpu_Din =
    shared_cs  ? shared_dout   :
    local_cs   ? local_ram_dout:
    sndpia_cs  ? sndpia_dout   :
    pia0_cs    ? pia0_dout     :
    pia1_cs    ? pia1_dout     :
    pia2_cs    ? pia2_dout     :
    rom_cs     ? rom_dout      :
    8'hFF;                          // acia_cs, firq_range, open bus

endmodule
