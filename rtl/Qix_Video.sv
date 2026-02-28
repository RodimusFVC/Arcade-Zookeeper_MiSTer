//============================================================================
//
// Qix Video CPU Board
// Copyright (C) 2026 Rodimus
//
// Hardware: mc6809e + mc6845 CRTC + 64KB VRAM + 1KB palette RAM +
//           1KB NVRAM + 24KB ROM
//
// Responsibilities:
//   - Video CPU (6809E @ ~1.25 MHz, E/Q clocks supplied by top-level)
//   - Framebuffer writes (64KB VRAM at $0000-$7FFF)
//   - Palette management (qix_palette at $9000-$93FF, bank at $8800)
//   - CRTC (mc6845 via qix_display, accessed at $9C00-$9C01)
//   - Shared RAM port B ($8000-$83FF, wired externally to Data CPU)
//   - NVRAM ($8400-$87FF, hiscore interface)
//   - Cross-CPU FIRQ signaling
//
//============================================================================

module Qix_Video (
    input         clk_20m,
    input         reset,
    input         E,            // 6809E E clock (~1.25 MHz), from top-level
    input         Q,            // 6809E Q clock (90° leading E)

    // Shared RAM interface (dual-port RAM lives in Qix.sv)
    output [9:0]  shared_addr,
    output [7:0]  shared_dout,
    input  [7:0]  shared_din,
    output        shared_we,

    // Cross-CPU FIRQ
    output        data_firq,    // pulse: video CPU asserts FIRQ on data CPU
    input         video_firq_n, // from data CPU → drives video 6809E nFIRQ

    // Video outputs
    output        hsync,
    output        vsync,
    output        hblank,
    output        vblank,
    output        ce_pix,
    output [7:0]  video_r,
    output [7:0]  video_g,
    output [7:0]  video_b,
    output        crtc_vsync,

    // ROM loader (MiSTer ioctl — pre-gated by address range in Qix.sv)
    input  [24:0] ioctl_addr,
    input  [7:0]  ioctl_data,
    input         ioctl_wr,

    // Hiscore / NVRAM interface (hs_address driven by hiscore module)
    input  [15:0] hs_address,
    output [7:0]  hs_data_out,
    input  [7:0]  hs_data_in,
    input         hs_write,

    input         pause,
    input         flip
);

// ---------------------------------------------------------------------------
// 6809E Video CPU
// ---------------------------------------------------------------------------
wire [15:0] cpu_A;
wire [7:0]  cpu_Dout;
wire        cpu_RnW;

mc6809e video_cpu (
    .D      (cpu_Din),
    .DOut   (cpu_Dout),
    .ADDR   (cpu_A),
    .RnW    (cpu_RnW),
    .E      (E),
    .Q      (Q),
    .nIRQ   (1'b1),
    .nFIRQ  (~video_firq_flag),  // use internal SR latch, not raw pulse input
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
// Write strobe: one-cycle pulse on falling edge of E while RnW is low
// ---------------------------------------------------------------------------
reg E_prev;
always @(posedge clk_20m) E_prev <= E;
wire cpu_E_fall = E_prev & ~E;
wire cpu_wr     = cpu_E_fall & ~cpu_RnW;

// ---------------------------------------------------------------------------
// Address decoder
// ---------------------------------------------------------------------------
wire vram_direct_cs  =  ~cpu_A[15];                                // $0000-$7FFF
wire shared_cs       = (cpu_A[15:10] == 6'b10_0000);              // $8000-$83FF
wire nvram_cs        = (cpu_A[15:10] == 6'b10_0001);              // $8400-$87FF
wire palbank_cs      = (cpu_A        == 16'h8800);                 // $8800
wire firq_assert_cs  = (cpu_A        == 16'h8C00);                 // $8C00
wire firq_ack_cs     = (cpu_A        == 16'h8C01);                 // $8C01
wire palette_cs      = (cpu_A[15:10] == 6'b10_0100);              // $9000-$93FF
wire vram_latch_cs   = (cpu_A        == 16'h9400);                 // $9400
wire latch_hi_cs     = (cpu_A        == 16'h9402);                 // $9402
wire latch_lo_cs     = (cpu_A        == 16'h9403);                 // $9403
wire scanline_cs     = (cpu_A[15:10] == 6'b10_0110);              // $9800 (partial)
wire crtc_bus_cs     = (cpu_A[15:2]  == 14'h2700);                // $9C00-$9C01
wire rom_cs          = (cpu_A[15:14] == 2'b11);                    // $C000-$FFFF

// ---------------------------------------------------------------------------
// Shared RAM outputs (port B wired to dual-port RAM in Qix.sv)
// ---------------------------------------------------------------------------
assign shared_addr = cpu_A[9:0];
assign shared_dout = cpu_Dout;
assign shared_we   = shared_cs & cpu_wr;

// ---------------------------------------------------------------------------
// Cross-CPU FIRQ logic
//
// data_firq    : one-cycle pulse on $8C00 write; data CPU latches it
// video_firq_n : from data CPU, drives 6809E nFIRQ directly (see inst above)
// video_firq_flag : latches video_firq_n assertion; cleared by $8C01 write
// ---------------------------------------------------------------------------
reg data_firq_r;
always @(posedge clk_20m) begin
    if (reset) data_firq_r <= 1'b0;
    else       data_firq_r <= firq_assert_cs & cpu_wr;
end
assign data_firq = data_firq_r;

reg video_firq_flag;
always @(posedge clk_20m) begin
    if (reset)                     video_firq_flag <= 1'b0;
    else if (firq_ack_cs & cpu_E_fall) video_firq_flag <= 1'b0;
    else if (~video_firq_n)        video_firq_flag <= 1'b1;
end

// ---------------------------------------------------------------------------
// VRAM address latch registers (written at $9402/$9403)
// ---------------------------------------------------------------------------
reg [7:0] vram_latch_addr_hi;
reg [7:0] vram_latch_addr_lo;

always @(posedge clk_20m) begin
    if (latch_hi_cs & cpu_wr) vram_latch_addr_hi <= cpu_Dout;
    if (latch_lo_cs & cpu_wr) vram_latch_addr_lo <= cpu_Dout;
end

// ---------------------------------------------------------------------------
// CRTC / display interconnects
// ---------------------------------------------------------------------------
wire [13:0] crtc_ma;
wire [4:0]  crtc_ra;
wire        crtc_de;
wire [15:0] display_addr;
wire [7:0]  display_data;
wire [7:0]  pixel_index;
wire [7:0]  pal_r, pal_g, pal_b;
wire [7:0]  crtc_do_w;

// ---------------------------------------------------------------------------
// qix_vram — 64KB dual-port framebuffer + address latch + scanline latch
// ---------------------------------------------------------------------------
wire [7:0] vram_dout;
wire [7:0] vram_latch_dout;
wire [7:0] scanline_latch;

qix_vram vram (
    .clk            (clk_20m),
    .flip           (flip),

    // CPU direct port ($0000-$7FFF)
    .addr           (cpu_A[14:0]),
    .we             (vram_direct_cs & cpu_wr),
    .din            (cpu_Dout),
    .dout           (vram_dout),

    // CPU latched port ($9400)
    .latch_addr_hi  (vram_latch_addr_hi),
    .latch_addr_lo  (vram_latch_addr_lo),
    .latch_we       (vram_latch_cs & cpu_wr),
    .latch_din      (cpu_Dout),
    .latch_dout     (vram_latch_dout),

    // Display scanout
    .display_addr   (display_addr),
    .display_dout   (display_data),

    // Scanline latch inputs from CRTC
    .crtc_ma        (crtc_ma),
    .crtc_ra        (crtc_ra),
    .crtc_de        (crtc_de),
    .scanline_latch (scanline_latch)
);

// ---------------------------------------------------------------------------
// qix_palette — 1KB palette RAM + RRGGBBII decoder
// ---------------------------------------------------------------------------
wire [7:0] pal_cpu_dout;

qix_palette palette (
    .clk         (clk_20m),

    .cpu_addr    (cpu_A[9:0]),
    .cpu_we      (palette_cs & cpu_wr),
    .cpu_din     (cpu_Dout),
    .cpu_dout    (pal_cpu_dout),

    .bank_we     (palbank_cs & cpu_wr),
    .bank_din    (cpu_Dout[1:0]),

    .pixel_index (pixel_index),
    .rgb_r       (pal_r),
    .rgb_g       (pal_g),
    .rgb_b       (pal_b)
);

// ---------------------------------------------------------------------------
// qix_display — CRTC + scanout pipeline
// ---------------------------------------------------------------------------
qix_display display (
    .clk_20m     (clk_20m),
    .reset       (reset),
    .flip        (flip),

    .ce_pix      (ce_pix),
    .hsync       (hsync),
    .vsync       (vsync),
    .hblank      (hblank),
    .vblank      (vblank),

    .crtc_ma     (crtc_ma),
    .crtc_ra     (crtc_ra),
    .crtc_de     (crtc_de),

    .display_addr (display_addr),
    .display_data (display_data),

    .pixel_index (pixel_index),
    .rgb_r       (pal_r),
    .rgb_g       (pal_g),
    .rgb_b       (pal_b),

    .video_r     (video_r),
    .video_g     (video_g),
    .video_b     (video_b),

    .crtc_cs     (crtc_bus_cs),
    .crtc_rw     (cpu_RnW),
    .crtc_rs     (cpu_A[0]),
    .crtc_di     (cpu_Dout),
    .crtc_do     (crtc_do_w)
);

assign crtc_vsync = vsync;

// ---------------------------------------------------------------------------
// NVRAM — 1KB BRAM ($8400-$87FF)
// Port A: Video CPU  |  Port B: hiscore framework
// Quartus infers true dual-port M10K from two independent always blocks.
// ---------------------------------------------------------------------------
reg [7:0] nvram [0:1023];
reg [7:0] nvram_cpu_dout;
reg [7:0] nvram_hs_dout;

// Port A — CPU read/write
always @(posedge clk_20m) begin
    if (nvram_cs & cpu_wr)
        nvram[cpu_A[9:0]] <= cpu_Dout;
    nvram_cpu_dout <= nvram[cpu_A[9:0]];
end

// Port B — hiscore read/write
always @(posedge clk_20m) begin
    if (hs_write)
        nvram[hs_address[9:0]] <= hs_data_in;
    nvram_hs_dout <= nvram[hs_address[9:0]];
end

assign hs_data_out = nvram_hs_dout;

// ---------------------------------------------------------------------------
// Video ROM — 16KB ($C000-$FFFF) in 16KB BRAM
//
// Loaded at ioctl_addr $04000-$07FFF (gated by Qix.sv).
// CPU read address: cpu_A[13:0]  ($C000→0 .. $FFFF→$3FFF)
// ioctl write address: ioctl_addr[13:0] (bits [13:0] of $04000-$07FFF = 0-$3FFF)
// ---------------------------------------------------------------------------
reg [7:0] vid_rom [0:16383];
reg [7:0] rom_dout;

wire [13:0] rom_cpu_addr   = cpu_A[13:0];
wire [13:0] rom_ioctl_addr = ioctl_addr[13:0];

always @(posedge clk_20m) begin
    if (ioctl_wr)
        vid_rom[rom_ioctl_addr] <= ioctl_data;
    rom_dout <= vid_rom[rom_cpu_addr];
end

// ---------------------------------------------------------------------------
// CPU data bus read mux — default $FF for open-bus / unimplemented reads
// ---------------------------------------------------------------------------
wire [7:0] cpu_Din =
    vram_direct_cs ? vram_dout       :
    shared_cs      ? shared_din      :
    nvram_cs       ? nvram_cpu_dout  :
    palette_cs     ? pal_cpu_dout    :
    vram_latch_cs  ? vram_latch_dout :
    scanline_cs    ? scanline_latch  :
    crtc_bus_cs    ? crtc_do_w      :
    rom_cs         ? rom_dout        :
    8'hFF;

endmodule
