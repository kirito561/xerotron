// =============================================================================
// Module      : servo_pwm
// Project     : XEROTON - 8-DOF Quadruped Robot (Verilog + Python/PyBullet)
// Description : Parameterized, synthesizable counter-based RC Servo PWM
//               generator with valid pulse-range clamping protection and
//               frame-synchronous shadowing for glitch-free output pulses.
//
// Timing Notes:
//   - Default PWM_HZ = 50 Hz (20 ms = 20,000 us frame period).
//   - Default CLK_HZ = 1,000,000 Hz (1 MHz) for fast RTL simulation while
//     preserving exact 20 ms servo timing (1 tick = 1 us, FRAME_TICKS = 20000).
//   - For a 50 MHz FPGA clock, override CLK_HZ = 50_000_000 (TICKS_PER_US = 50,
//     FRAME_TICKS = 1,000,000).
//   - Real hobby servos require physical endpoint calibration; default safe
//     bounds are [MIN_PULSE_US, MAX_PULSE_US] = [1000, 2000] us.
// =============================================================================
`timescale 1ns/1ps

module servo_pwm #(
    parameter integer CLK_HZ        = 1000000, // Input clock frequency in Hz
    parameter integer PWM_HZ        = 50,      // Servo PWM frame frequency in Hz
    parameter integer MIN_PULSE_US  = 1000,    // Minimum safe pulse width (us)
    parameter integer MAX_PULSE_US  = 2000,    // Maximum safe pulse width (us)
    parameter integer SAFE_PULSE_US = 1500     // Reset / safe neutral pulse (us)
)(
    input  wire        clk,           // System clock
    input  wire        reset,         // Synchronous active-high reset
    input  wire [11:0] pulse_us,      // Requested pulse width in microseconds
    output reg         pwm,           // Servo PWM output signal
    output reg         frame_tick,    // 1-cycle strobe at end of each PWM frame
    output wire        pulse_clamped  // High when requested pulse_us is out of bounds
);

    // -------------------------------------------------------------------------
    // Derived Timing Parameters
    // -------------------------------------------------------------------------
    localparam integer FRAME_TICKS  = CLK_HZ / PWM_HZ;
    localparam integer TICKS_PER_US = (CLK_HZ >= 1000000) ? (CLK_HZ / 1000000) : 1;

    // -------------------------------------------------------------------------
    // Combinational Pulse-Range Protection (Defense-in-Depth Clamping)
    // -------------------------------------------------------------------------
    reg [11:0] clamped_pulse_us;
    reg        out_of_range;

    always @(*) begin
        if (pulse_us < MIN_PULSE_US[11:0]) begin
            clamped_pulse_us = MIN_PULSE_US[11:0];
            out_of_range     = 1'b1;
        end else if (pulse_us > MAX_PULSE_US[11:0]) begin
            clamped_pulse_us = MAX_PULSE_US[11:0];
            out_of_range     = 1'b1;
        end else begin
            clamped_pulse_us = pulse_us;
            out_of_range     = 1'b0;
        end
    end

    assign pulse_clamped = out_of_range;

    // -------------------------------------------------------------------------
    // Frame Counter & Glitch-Free Shadowed Pulse Register
    // -------------------------------------------------------------------------
    // Timing alignment:
    //   - frame_tick is asserted for 1 clock when count == FRAME_TICKS - 1.
    //   - Downstream gait_controller updates pulse_us on that posedge clk.
    //   - On the very next cycle (count == 0), servo_pwm latches clamped_pulse_us
    //     into active_pulse_us and uses it for the entire 20 ms frame.
    //   - This yields an exact period of FRAME_TICKS cycles and an exact pulse
    //     duration of (active_pulse_us * TICKS_PER_US) cycles with zero glitches.
    // -------------------------------------------------------------------------
    reg  [31:0] count;
    reg  [11:0] active_pulse_us;
    wire [11:0] effective_pulse_us;
    wire [31:0] pulse_ticks;

    assign effective_pulse_us = (count == 32'd0) ? clamped_pulse_us : active_pulse_us;
    assign pulse_ticks        = effective_pulse_us * TICKS_PER_US;

    always @(posedge clk) begin
        if (reset) begin
            count           <= 32'd0;
            pwm             <= 1'b0;
            frame_tick      <= 1'b0;
            active_pulse_us <= SAFE_PULSE_US[11:0];
        end else begin
            // Latch the clamped pulse target at the start of each frame
            if (count == 32'd0) begin
                active_pulse_us <= clamped_pulse_us;
            end

            // Frame counter wrap & end-of-frame tick generation
            if (count >= FRAME_TICKS - 1) begin
                count      <= 32'd0;
                frame_tick <= 1'b0;
            end else begin
                count      <= count + 32'd1;
                frame_tick <= (count == FRAME_TICKS - 2);
            end

            // Registered PWM comparator output (high for exact pulse_ticks cycles)
            pwm <= (count < pulse_ticks);
        end
    end

endmodule
