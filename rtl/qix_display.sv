// qix_display.sv — CRTC + Display Scanout Pipeline
//
// Instantiates mc6845, generates pixel clock and character clock enables,
// drives display address into qix_vram display port, feeds pixel data
// through qix_palette, and produces DE-gated RGB output.
//
// No CPU logic lives here.  The CPU bus signals (crtc_cs/rw/rs/di/do)
// pass straight through to the mc6845 register interface.

module qix_display (
    input         clk_20m,
    input         reset,
    input         flip,          // passed to qix_vram; unused inside this module

    // Pixel clock enable (5 MHz = 20 MHz ÷ 4)
    output        ce_pix,

    // Sync / blank outputs
    output        hsync,
    output        vsync,
    output        hblank,
    output        vblank,

    // CRTC MA/RA/DE — wired directly to qix_vram for scanline latch
    output [13:0] crtc_ma,
    output [4:0]  crtc_ra,
    output        crtc_de,

    // VRAM display port — wired directly to qix_vram
    output [15:0] display_addr,
    input  [7:0]  display_data,

    // Palette port — wired directly to qix_palette
    output [7:0]  pixel_index,
    input  [7:0]  rgb_r,
    input  [7:0]  rgb_g,
    input  [7:0]  rgb_b,

    // RGB output (zeroed outside DE)
    output [7:0]  video_r,
    output [7:0]  video_g,
    output [7:0]  video_b,

    // CRTC CPU bus — wired directly from Video CPU memory decoder
    input         crtc_cs,
    input         crtc_rw,
    input         crtc_rs,
    input  [7:0]  crtc_di,
    output [7:0]  crtc_do
);

// ---------------------------------------------------------------------------
// Clock enables
//   ce_pix    : 5 MHz  (every  4 clk_20m ticks)
//   clken_625k: 625 kHz (every 32 clk_20m ticks) — mc6845 character clock
// ---------------------------------------------------------------------------
reg [4:0] div;

always @(posedge clk_20m)
    div <= div + 5'd1;

assign ce_pix       = (div[1:0] == 2'b11);  // pulses every 4 clocks
wire   clken_625k   = (&div);                // pulses every 32 clocks

// ---------------------------------------------------------------------------
// 3-bit pixel counter (0–7): increments at ce_pix, resets on character clock
// ---------------------------------------------------------------------------
reg [2:0] pix_cnt;

always @(posedge clk_20m)
    if (clken_625k)   pix_cnt <= 3'd0;
    else if (ce_pix)  pix_cnt <= pix_cnt + 3'd1;

// ---------------------------------------------------------------------------
// mc6845 CRTC (VHDL entity, Quartus mixed-language synthesis)
// ---------------------------------------------------------------------------
mc6845 crtc (
    .CLOCK  (clk_20m),
    .CLKEN  (clken_625k),
    .nRESET (~reset),
    // CPU bus
    .ENABLE (crtc_cs),
    .R_nW   (crtc_rw),
    .RS     (crtc_rs),
    .DI     (crtc_di),
    .DO     (crtc_do),
    // Display outputs
    .VSYNC  (vsync),
    .HSYNC  (hsync),
    .VBLANK (vblank),
    .HBLANK (hblank),
    .DE     (crtc_de),
    .CURSOR (),
    .LPSTB  (1'b0),
    // Memory address outputs
    .MA     (crtc_ma),
    .RA     (crtc_ra)
);

// ---------------------------------------------------------------------------
// Display address
//   Formula (from MAME qix.cpp):
//     ((MA << 6) & 0xF800) | ((RA << 8) & 0x0700) | pix_cnt
//   Expanded:
//     bits [15:11] = MA[9:5]
//     bits [10: 8] = RA[2:0]
//     bits [ 7: 3] = 0
//     bits [ 2: 0] = pix_cnt
// ---------------------------------------------------------------------------
assign display_addr = {crtc_ma[9:5], crtc_ra[2:0], 5'b0, pix_cnt};

// ---------------------------------------------------------------------------
// Palette feed: raw VRAM byte is the palette index
// ---------------------------------------------------------------------------
assign pixel_index = display_data;

// ---------------------------------------------------------------------------
// RGB output — palette result, forced black outside active display area
// ---------------------------------------------------------------------------
assign video_r = crtc_de ? rgb_r : 8'd0;
assign video_g = crtc_de ? rgb_g : 8'd0;
assign video_b = crtc_de ? rgb_b : 8'd0;

endmodule
