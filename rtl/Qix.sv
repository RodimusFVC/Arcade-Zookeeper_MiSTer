//============================================================================
//
// Qix Platform Top-Level
// Copyright (C) 2026 Rodimus
//
// Wires together Qix_CPU (data), Qix_Video (video), and Qix_Sound (audio).
//
// Responsibilities:
//   - 6809E E/Q clock generation (1.25 MHz quadrature from 20 MHz)
//   - 1KB shared dual-port RAM (port A = data CPU, port B = video CPU)
//   - FIRQ cross-signal routing between CPUs
//   - Sound PIA signal routing between data CPU and audio board
//   - ROM ioctl fan-out (each subsystem filters on its own ioctl_index)
//   - PIA input assembly
//
//============================================================================

module Qix (
    input         clk_20m,
    input         reset,

    // Player inputs (active-low)
    input  [1:0]  coin,
    input  [1:0]  start_buttons,
    input  [3:0]  p1_joystick,    // {R,L,D,U}
    input  [3:0]  p2_joystick,
    input         p1_fire,
    input         p2_fire,
    input  [15:0] dip_sw,

    // Video output
    output        video_hsync,
    output        video_vsync,
    output        video_vblank,
    output        video_hblank,
    output        ce_pix,
    output [7:0]  video_r,
    output [7:0]  video_g,
    output [7:0]  video_b,

    // Audio output (signed 16-bit stereo)
    output signed [15:0] sound_l,
    output signed [15:0] sound_r,

    // ROM loading (MiSTer ioctl)
    input  [24:0] ioctl_addr,
    input  [7:0]  ioctl_data,
    input         ioctl_wr,
    input  [7:0]  ioctl_index,

    // Hiscore interface (hs_address driven externally by hiscore module)
    input  [15:0] hs_address,
    input  [7:0]  hs_data_in,
    output [7:0]  hs_data_out,
    input         hs_write,

    input         pause
);

// ---------------------------------------------------------------------------
// Clock generation — 6809E E/Q quadrature at 1.25 MHz (20 MHz ÷ 16)
//
// clk_div[3:2] phase:  00 → E=0,Q=0
//                      01 → E=0,Q=1  (Q leads E by 90°)
//                      10 → E=1,Q=1
//                      11 → E=1,Q=0
// ---------------------------------------------------------------------------
reg [3:0] clk_div;
always @(posedge clk_20m) clk_div <= clk_div + 4'd1;

wire cpu_E = clk_div[3];
wire cpu_Q = clk_div[3] ^ clk_div[2];

// ---------------------------------------------------------------------------
// Shared 1KB dual-port RAM (port A = data CPU, port B = video CPU)
// Quartus infers true dual-port M10K from two independent always blocks.
// ---------------------------------------------------------------------------
reg [7:0] shared_ram [0:1023];

wire [9:0] cpu_sh_addr;
wire [7:0] cpu_sh_din;
wire [7:0] cpu_sh_dout;
wire        cpu_sh_we;

wire [9:0] vid_sh_addr;
wire [7:0] vid_sh_din;
wire [7:0] vid_sh_dout;
wire        vid_sh_we;

// Port A — data CPU
always @(posedge clk_20m)
    if (cpu_sh_we) shared_ram[cpu_sh_addr] <= cpu_sh_din;
assign cpu_sh_dout = shared_ram[cpu_sh_addr];

// Port B — video CPU
always @(posedge clk_20m)
    if (vid_sh_we) shared_ram[vid_sh_addr] <= vid_sh_din;
assign vid_sh_dout = shared_ram[vid_sh_addr];

// ---------------------------------------------------------------------------
// FIRQ cross-signals
//
// Latches are inside each CPU board; top-level inverts one-cycle pulses.
//   cpu_video_firq : data CPU asserts FIRQ on video CPU (active-high pulse)
//   vid_data_firq  : video CPU asserts FIRQ on data CPU (active-high pulse)
//
// Qix_CPU uses falling-edge detect on data_firq_n → 1-cycle low pulse works.
// Qix_Video uses level-check on video_firq_n  → 1-cycle low pulse works
//   because the internal video_firq_flag latches the assertion.
// ---------------------------------------------------------------------------
wire cpu_video_firq;   // from Qix_CPU
wire vid_data_firq;    // from Qix_Video

wire data_firq_n  = ~vid_data_firq;    // active-low to Qix_CPU
wire video_firq_n = ~cpu_video_firq;   // active-low to Qix_Video

// ---------------------------------------------------------------------------
// Sound PIA signal routing
//   sndPIA0 lives in Qix_CPU; sndPIA1 lives in Qix_Sound.
// ---------------------------------------------------------------------------
wire [7:0] snd_cmd;          // Qix_CPU sndPIA0 PA out → Qix_Sound sndPIA1 PA in
wire [7:0] snd_vol;          // Qix_CPU sndPIA0 PB out → Qix_Sound vol_data
wire        snd_irq_cpu2snd;  // Qix_CPU sndPIA0 CA2 → Qix_Sound sndPIA1 CA1
wire        snd_irq_snd2cpu;  // Qix_Sound sndPIA1 CA2 → Qix_CPU sndPIA0 CA1
wire        flip;             // Qix_CPU sndPIA0 CB2 → Qix_Video flip

// ---------------------------------------------------------------------------
// PIA input assembly (all signals active-low from wrapper)
//
// PIA0 port A (P1): [7]=fire [6]=start1 [5]=start2 [4]=1 [3:0]={R,L,D,U}
// PIA0 port B (COIN): [7]=tilt [6]=coin2 [5]=coin1 [4]=service [3:0]=1
// PIA2 port A (P2): [7]=fire [6:4]=1 [3:0]={R,L,D,U}
// ---------------------------------------------------------------------------
wire [7:0] p1_pia   = {p1_fire, start_buttons[0], start_buttons[1], 1'b1,
                        p1_joystick[3], p1_joystick[2], p1_joystick[1], p1_joystick[0]};
wire [7:0] coin_pia = {1'b1, coin[1], coin[0], 1'b1, 4'b1111};
wire [7:0] p2_pia   = {p2_fire, 3'b111,
                        p2_joystick[3], p2_joystick[2], p2_joystick[1], p2_joystick[0]};

// ---------------------------------------------------------------------------
// Qix_CPU — data CPU board
// ---------------------------------------------------------------------------
Qix_CPU cpu_board (
    .clk_20m         (clk_20m),
    .reset           (reset),
    .E               (cpu_E),
    .Q               (cpu_Q),

    .shared_addr     (cpu_sh_addr),
    .shared_din      (cpu_sh_din),
    .shared_dout     (cpu_sh_dout),
    .shared_we       (cpu_sh_we),

    .video_firq      (cpu_video_firq),
    .data_firq_n     (data_firq_n),

    .p1_input        (p1_pia),
    .coin_input      (coin_pia),
    .spare_input     (8'hFF),
    .in0_input       (8'hFF),
    .p2_input        (p2_pia),

    .snd_data_out    (snd_cmd),
    .snd_vol_out     (snd_vol),
    .snd_irq_to_snd  (snd_irq_cpu2snd),
    .snd_irq_from_snd(snd_irq_snd2cpu),
    .flip_screen     (flip),

    .ioctl_addr      (ioctl_addr),
    .ioctl_data      (ioctl_data),
    .ioctl_wr        (ioctl_wr),
    .ioctl_index     (ioctl_index),

    .pause           (pause)
);

// ---------------------------------------------------------------------------
// Qix_Video — video CPU board
// ---------------------------------------------------------------------------
Qix_Video video_board (
    .clk_20m         (clk_20m),
    .reset           (reset),
    .E               (cpu_E),
    .Q               (cpu_Q),

    .shared_addr     (vid_sh_addr),
    .shared_dout     (vid_sh_din),    // video CPU write data → shared RAM port B
    .shared_din      (vid_sh_dout),   // shared RAM port B read → video CPU
    .shared_we       (vid_sh_we),

    .data_firq       (vid_data_firq),
    .video_firq_n    (video_firq_n),

    .hsync           (video_hsync),
    .vsync           (video_vsync),
    .hblank          (video_hblank),
    .vblank          (video_vblank),
    .ce_pix          (ce_pix),
    .video_r         (video_r),
    .video_g         (video_g),
    .video_b         (video_b),
    .crtc_vsync      (),

    .ioctl_addr      (ioctl_addr),
    .ioctl_data      (ioctl_data),
    .ioctl_wr        (ioctl_wr),
    .ioctl_index     (ioctl_index),

    .hs_address      (hs_address),
    .hs_data_out     (hs_data_out),
    .hs_data_in      (hs_data_in),
    .hs_write        (hs_write),

    .pause           (pause),
    .flip            (flip)
);

// ---------------------------------------------------------------------------
// Qix_Sound — audio board
// ---------------------------------------------------------------------------
Qix_Sound sound_board (
    .clk_20m          (clk_20m),
    .reset            (reset),

    .snd_data_in      (snd_cmd),
    .snd_data_out     (),            // sound→data path unused (sndPIA0 pa_i = 8'h00)
    .snd_irq_from_cpu (snd_irq_cpu2snd),
    .snd_irq_to_cpu   (snd_irq_snd2cpu),

    .vol_data         (snd_vol),

    .audio_l          (sound_l),
    .audio_r          (sound_r),

    .ioctl_addr       (ioctl_addr),
    .ioctl_data       (ioctl_data),
    .ioctl_wr         (ioctl_wr),
    .ioctl_index      (ioctl_index),

    .pause            (pause)
);

endmodule
