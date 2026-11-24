// =============================================================================
// Module      : xeroton_top
// Project     : XEROTON Studio 3.0 - 8-DOF Quadruped Robot
// Description : Top-level synthesizable RTL integrating:
//                 - gait_controller   (9-state smooth FSM + prev_state + frame_counter)
//                 - safety_controller (E-STOP, invalid cmd, watchdog, clamping)
//                 - 8x servo_pwm      (50 Hz, 20 ms period, defense-in-depth limits)
// =============================================================================
`timescale 1ns/1ps

module xeroton_top #(
    parameter integer CLK_HZ           = 1000000,
    parameter integer PWM_HZ           = 50,
    parameter integer FRAMES_PER_PHASE = 4,
    parameter integer TIMEOUT_FRAMES   = 15,
    parameter integer MIN_PULSE_US     = 1000,
    parameter integer MAX_PULSE_US     = 2000,
    parameter integer SAFE_PULSE_US    = 1500
)(
    input  wire        clk,
    input  wire        reset,
    input  wire [3:0]  command,
    input  wire        cmd_valid,
    input  wire        estop,
    input  wire        fault_clear,
    input  wire        watchdog_enable,
    input  wire [7:0]  stride_us,
    input  wire [3:0]  phase_frames,

    output wire [7:0]  servo_pwm,
    output wire        frame_tick,
    output wire [3:0]  state,
    output wire [3:0]  prev_state,
    output wire [2:0]  gait_phase,
    output wire [15:0] frame_counter,
    output wire        fault_active,
    output wire [3:0]  fault_code,
    output wire [7:0]  limit_clamped,
    output wire [7:0]  pwm_clamped,

    output wire [11:0] j0,
    output wire [11:0] j1,
    output wire [11:0] j2,
    output wire [11:0] j3,
    output wire [11:0] j4,
    output wire [11:0] j5,
    output wire [11:0] j6,
    output wire [11:0] j7
);

    wire [11:0] raw_j0, raw_j1, raw_j2, raw_j3;
    wire [11:0] raw_j4, raw_j5, raw_j6, raw_j7;
    wire        fault_override;

    wire [11:0] safe_target [0:7];
    assign safe_target[0] = j0;
    assign safe_target[1] = j1;
    assign safe_target[2] = j2;
    assign safe_target[3] = j3;
    assign safe_target[4] = j4;
    assign safe_target[5] = j5;
    assign safe_target[6] = j6;
    assign safe_target[7] = j7;

    gait_controller #(
        .FRAMES_PER_PHASE (FRAMES_PER_PHASE)
    ) u_gait_controller (
        .clk           (clk),
        .reset         (reset),
        .frame_tick    (frame_tick),
        .command       (command),
        .cmd_valid     (cmd_valid),
        .estop         (estop),
        .fault_active  (fault_override),
        .stride_us     (stride_us),
        .phase_frames  (phase_frames),
        .j0            (raw_j0),
        .j1            (raw_j1),
        .j2            (raw_j2),
        .j3            (raw_j3),
        .j4            (raw_j4),
        .j5            (raw_j5),
        .j6            (raw_j6),
        .j7            (raw_j7),
        .state         (state),
        .prev_state    (prev_state),
        .gait_phase    (gait_phase),
        .frame_counter (frame_counter)
    );

    safety_controller #(
        .HIP_MIN_US     (1050),
        .HIP_MAX_US     (1900),
        .KNEE_MIN_US    (1050),
        .KNEE_MAX_US    (1950),
        .SAFE_PULSE_US  (SAFE_PULSE_US),
        .TIMEOUT_FRAMES (TIMEOUT_FRAMES)
    ) u_safety_controller (
        .clk             (clk),
        .reset           (reset),
        .frame_tick      (frame_tick),
        .command         (command),
        .cmd_valid       (cmd_valid),
        .estop           (estop),
        .fault_clear     (fault_clear),
        .watchdog_enable (watchdog_enable),
        .raw_j0          (raw_j0),
        .raw_j1          (raw_j1),
        .raw_j2          (raw_j2),
        .raw_j3          (raw_j3),
        .raw_j4          (raw_j4),
        .raw_j5          (raw_j5),
        .raw_j6          (raw_j6),
        .raw_j7          (raw_j7),
        .safe_j0         (j0),
        .safe_j1         (j1),
        .safe_j2         (j2),
        .safe_j3         (j3),
        .safe_j4         (j4),
        .safe_j5         (j5),
        .safe_j6         (j6),
        .safe_j7         (j7),
        .fault_active    (fault_active),
        .fault_override  (fault_override),
        .fault_code      (fault_code),
        .limit_clamped   (limit_clamped)
    );

    wire [7:0] channel_ticks;

    genvar i;
    generate
        for (i = 0; i < 8; i = i + 1) begin : gen_servo_channels
            servo_pwm #(
                .CLK_HZ        (CLK_HZ),
                .PWM_HZ        (PWM_HZ),
                .MIN_PULSE_US  (MIN_PULSE_US),
                .MAX_PULSE_US  (MAX_PULSE_US),
                .SAFE_PULSE_US (SAFE_PULSE_US)
            ) u_servo_pwm (
                .clk           (clk),
                .reset         (reset),
                .pulse_us      (safe_target[i]),
                .pwm           (servo_pwm[i]),
                .frame_tick    (channel_ticks[i]),
                .pulse_clamped (pwm_clamped[i])
            );
        end
    endgenerate

    assign frame_tick = channel_ticks[0];

endmodule
