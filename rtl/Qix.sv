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
//   - ROM ioctl address-range dispatch to each subsystem
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
    input         p1_btn1,        // Player 1 Draw Slow (active-low)
    input         p1_btn2,        // Player 1 Draw Fast (active-low)
    input         p2_btn1,        // Player 2 Draw Slow (active-low)
    input         p2_btn2,        // Player 2 Draw Fast (active-low)
    
    input         service,        // Test Advance button (active-low)
    input         service2,
    input         service3,
    input         service4,

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
// Explicit dpram_dc to guarantee M10K inference.
// ---------------------------------------------------------------------------
wire [9:0] cpu_sh_addr;
wire [7:0] cpu_sh_din;
wire [7:0] cpu_sh_dout;
wire        cpu_sh_we;

wire [9:0] vid_sh_addr;
wire [7:0] vid_sh_din;
wire [7:0] vid_sh_dout;
wire        vid_sh_we;

dpram_dc #(.widthad_a(10)) shared_ram_inst (
    .clock_a    (clk_20m),
    .address_a  (cpu_sh_addr),
    .data_a     (cpu_sh_din),
    .wren_a     (cpu_sh_we),
    .q_a        (cpu_sh_dout),

    .clock_b    (clk_20m),
    .address_b  (vid_sh_addr),
    .data_b     (vid_sh_din),
    .wren_b     (vid_sh_we),
    .q_b        (vid_sh_dout)
);

// ---------------------------------------------------------------------------
// ROM ioctl address-range dispatch (concatenated ROM, all ioctl_index == 0)
//   $00000-$03FFF : Data CPU ROM  (16KB)
//   $04000-$07FFF : Video CPU ROM (16KB)
//   $08000-$087FF : Audio CPU ROM  (2KB)
// ---------------------------------------------------------------------------
wire cpu_ioctl_wr = ioctl_wr & (ioctl_addr < 25'h06000);                             // 24KB
wire vid_ioctl_wr = ioctl_wr & (ioctl_addr >= 25'h06000) & (ioctl_addr < 25'h0C000); // 24KB
wire snd_ioctl_wr = ioctl_wr & (ioctl_addr >= 25'h0C000) & (ioctl_addr < 25'h0F000); // 12KB

// ---------------------------------------------------------------------------
// FIRQ cross-signals (from schematic Figure 13, U7 7474 dual flip-flop)
//
// Two SR latches using async PRE/CLR:
//   data_firq_latch:  SET by video CPU $8C00, CLEAR by data CPU $8C01
//   video_firq_latch: SET by data CPU $8C00, CLEAR by video CPU $8C01
// ---------------------------------------------------------------------------
wire cpu_firq_assert;    // data CPU accessed $8C00 (pulse)
wire cpu_firq_ack;       // data CPU accessed $8C01 (pulse)
wire vid_firq_assert;    // video CPU accessed $8C00 (pulse)
wire vid_firq_ack;       // video CPU accessed $8C01 (pulse)

reg data_firq_latch;
reg video_firq_latch;

always @(posedge clk_20m) begin
    if (reset) begin
        data_firq_latch  <= 1'b0;
        video_firq_latch <= 1'b0;
    end else begin
        if (cpu_firq_ack)         data_firq_latch <= 1'b0;
        else if (vid_firq_assert) data_firq_latch <= 1'b1;

        if (vid_firq_ack)           video_firq_latch <= 1'b0;
        else if (cpu_firq_assert)   video_firq_latch <= 1'b1;
    end
end

wire data_firq_n  = ~data_firq_latch;
wire video_firq_n = ~video_firq_latch;

// Delay VSYNC to Data CPU until CRTC is programmed.
reg [23:0] vsync_delay;
reg        vsync_enable;
always @(posedge clk_20m) begin
    if (reset) begin
        vsync_delay  <= 24'd0;
        vsync_enable <= 1'b0;
    end else if (!vsync_enable) begin
        vsync_delay <= vsync_delay + 24'd1;
        if (vsync_delay == 24'd16_777_215)  // ~0.84 sec
//        if (vsync_delay == 24'd1_000_000)     // ~0.05 sec
            vsync_enable <= 1'b1;
    end
end

wire crtc_vsync_out;
wire crtc_vsync_gated = vsync_enable ? crtc_vsync_out : 1'b0;

// wire crtc_vsync_out;
// wire crtc_vsync_gated = crtc_vsync_out;

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
// PIA input assembly — EXACT match to schematic Figure 16
//
// PIA0 port A (U11 PA): [7]=Fire1 [6]=Start1 [5]=Start2 [4]=Spare/Button2 [3:0]={R,L,D,U}
// PIA0 port B (U11 PB): [7]=Tilt [6]=Coin3 [5]=Coin2 [4]=Coin1 [3:0]={svc4,svc3,svc2,svc1}
// PIA2 port A (U30 PA): [7]=Fire2 [6:4]=Spare [3:0]={R,L,D,U}
// ---------------------------------------------------------------------------
wire [7:0] p1_pia   = {p1_btn1, start_buttons[0], start_buttons[1], p1_btn2,
                        p1_joystick[0], p1_joystick[1], p1_joystick[2], p1_joystick[3]};  // R,L,D,U order

wire [7:0] coin_pia = {1'b1, 1'b1, coin[1], coin[0], service4, service3, service2, service};

wire [7:0] p2_pia   = {p2_btn1, 3'b111,
                        p2_joystick[0], p2_joystick[1], p2_joystick[2], p2_joystick[3]};  // R,L,D,U order

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

    .video_firq      (cpu_firq_assert),
    .data_firq_ack   (cpu_firq_ack),
    .data_firq_n     (data_firq_n),

    .p1_input        (p1_pia),
    .coin_input      (coin_pia),
    .spare_input     (8'hFF),
    .in0_input       (8'hFF),
    .p2_input        (p2_pia),

    .crtc_vsync      (crtc_vsync_gated),

    .snd_data_out    (snd_cmd),
    .snd_vol_out     (snd_vol),
    .snd_irq_to_snd  (snd_irq_cpu2snd),
    .snd_irq_from_snd(snd_irq_snd2cpu),
    .flip_screen     (flip),

    .ioctl_addr      (ioctl_addr),
    .ioctl_data      (ioctl_data),
    .ioctl_wr        (cpu_ioctl_wr),

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

    .data_firq       (vid_firq_assert),
    .video_firq_ack  (vid_firq_ack),
    .video_firq_n    (video_firq_n),

    .hsync           (video_hsync),
    .vsync           (video_vsync),
    .hblank          (video_hblank),
    .vblank          (video_vblank),
    .ce_pix          (ce_pix),
    .video_r         (video_r),
    .video_g         (video_g),
    .video_b         (video_b),
    .crtc_vsync      (crtc_vsync_out),

    .ioctl_addr      (ioctl_addr),
    .ioctl_data      (ioctl_data),
    .ioctl_wr        (vid_ioctl_wr),

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
    .ioctl_wr         (snd_ioctl_wr),

    .pause            (pause)
);

endmodule
