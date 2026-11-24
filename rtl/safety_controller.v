// =============================================================================
// Module      : safety_controller
// Project     : XEROTON Studio 3.0 - 8-DOF Quadruped Robot
// Description : Hardware safety supervisor for XEROTON.
//               Responsibilities:
//                 1. Emergency-stop (E-STOP) immediate 1-clock fault latching.
//                 2. Invalid command trap (commands > CMD_MAX_VALID = 4'd7).
//                 3. Communication watchdog timeout (TIMEOUT_FRAMES frames).
//                 4. Per-joint servo pulse-width limit clamping (hips & knees).
//                 5. Fault override policy forcing all 8 outputs to SAFE_PULSE_US.
//                 6. Latched fault requiring deliberate fault_clear + safe command.
// =============================================================================
`timescale 1ns/1ps

module safety_controller #(
    parameter integer HIP_MIN_US     = 1050,
    parameter integer HIP_MAX_US     = 1900,
    parameter integer KNEE_MIN_US    = 1050,
    parameter integer KNEE_MAX_US    = 1950,
    parameter integer SAFE_PULSE_US  = 1500,
    parameter integer TIMEOUT_FRAMES = 15
)(
    input  wire        clk,
    input  wire        reset,
    input  wire        frame_tick,
    input  wire [3:0]  command,
    input  wire        cmd_valid,
    input  wire        estop,
    input  wire        fault_clear,
    input  wire        watchdog_enable,

    input  wire [11:0] raw_j0,
    input  wire [11:0] raw_j1,
    input  wire [11:0] raw_j2,
    input  wire [11:0] raw_j3,
    input  wire [11:0] raw_j4,
    input  wire [11:0] raw_j5,
    input  wire [11:0] raw_j6,
    input  wire [11:0] raw_j7,

    output wire [11:0] safe_j0,
    output wire [11:0] safe_j1,
    output wire [11:0] safe_j2,
    output wire [11:0] safe_j3,
    output wire [11:0] safe_j4,
    output wire [11:0] safe_j5,
    output wire [11:0] safe_j6,
    output wire [11:0] safe_j7,

    output reg         fault_active,
    output wire        fault_override,
    output reg  [3:0]  fault_code,     // [0]=ESTOP, [1]=INVALID_CMD, [2]=TIMEOUT, [3]=LIMIT_HIT
    output wire [7:0]  limit_clamped
);

    // Valid commands: 0=IDLE, 1=STAND, 2=WALK_FORWARD, 3=STOP, 4=WAVE, 5=DANCE,
    // 6=SIT, 7=TROT, 8=SLEEP, 9=WALK_BACKWARD, 10=STRAFE_LEFT, 11=STRAFE_RIGHT,
    // 12=TURN_LEFT, 13=TURN_RIGHT. Commands 14..15 are reserved/invalid -> FAULT.
    localparam [3:0] CMD_IDLE      = 4'd0;
    localparam [3:0] CMD_STOP      = 4'd3;
    localparam [3:0] CMD_MAX_VALID = 4'd13;

    wire invalid_cmd = cmd_valid && (command > CMD_MAX_VALID);

    // Watchdog Counter with frame-boundary lookahead
    reg [7:0] timeout_cnt;
    wire      timeout_tripped = watchdog_enable && (
                                    (timeout_cnt >= TIMEOUT_FRAMES[7:0]) ||
                                    (frame_tick && !cmd_valid && (timeout_cnt >= TIMEOUT_FRAMES[7:0] - 8'd1))
                                );

    function [11:0] clamp_pulse;
        input [11:0] val;
        input [11:0] lo;
        input [11:0] hi;
        begin
            if (val < lo)      clamp_pulse = lo;
            else if (val > hi) clamp_pulse = hi;
            else               clamp_pulse = val;
        end
    endfunction

    function is_out_of_bounds;
        input [11:0] val;
        input [11:0] lo;
        input [11:0] hi;
        begin
            is_out_of_bounds = (val < lo) || (val > hi);
        end
    endfunction

    wire [11:0] c_j0 = clamp_pulse(raw_j0, HIP_MIN_US[11:0],  HIP_MAX_US[11:0]);
    wire [11:0] c_j1 = clamp_pulse(raw_j1, KNEE_MIN_US[11:0], KNEE_MAX_US[11:0]);
    wire [11:0] c_j2 = clamp_pulse(raw_j2, HIP_MIN_US[11:0],  HIP_MAX_US[11:0]);
    wire [11:0] c_j3 = clamp_pulse(raw_j3, KNEE_MIN_US[11:0], KNEE_MAX_US[11:0]);
    wire [11:0] c_j4 = clamp_pulse(raw_j4, HIP_MIN_US[11:0],  HIP_MAX_US[11:0]);
    wire [11:0] c_j5 = clamp_pulse(raw_j5, KNEE_MIN_US[11:0], KNEE_MAX_US[11:0]);
    wire [11:0] c_j6 = clamp_pulse(raw_j6, HIP_MIN_US[11:0],  HIP_MAX_US[11:0]);
    wire [11:0] c_j7 = clamp_pulse(raw_j7, KNEE_MIN_US[11:0], KNEE_MAX_US[11:0]);

    assign limit_clamped[0] = is_out_of_bounds(raw_j0, HIP_MIN_US[11:0],  HIP_MAX_US[11:0]);
    assign limit_clamped[1] = is_out_of_bounds(raw_j1, KNEE_MIN_US[11:0], KNEE_MAX_US[11:0]);
    assign limit_clamped[2] = is_out_of_bounds(raw_j2, HIP_MIN_US[11:0],  HIP_MAX_US[11:0]);
    assign limit_clamped[3] = is_out_of_bounds(raw_j3, KNEE_MIN_US[11:0], KNEE_MAX_US[11:0]);
    assign limit_clamped[4] = is_out_of_bounds(raw_j4, HIP_MIN_US[11:0],  HIP_MAX_US[11:0]);
    assign limit_clamped[5] = is_out_of_bounds(raw_j5, KNEE_MIN_US[11:0], KNEE_MAX_US[11:0]);
    assign limit_clamped[6] = is_out_of_bounds(raw_j6, HIP_MIN_US[11:0],  HIP_MAX_US[11:0]);
    assign limit_clamped[7] = is_out_of_bounds(raw_j7, KNEE_MIN_US[11:0], KNEE_MAX_US[11:0]);

    assign fault_override = fault_active || estop || invalid_cmd || timeout_tripped;

    always @(posedge clk) begin
        if (reset) begin
            fault_active <= 1'b0;
            fault_code   <= 4'b0000;
            timeout_cnt  <= 8'd0;
        end else begin
            if (!watchdog_enable || cmd_valid) begin
                timeout_cnt <= 8'd0;
            end else if (frame_tick && (timeout_cnt < 8'hFF)) begin
                timeout_cnt <= timeout_cnt + 8'd1;
            end

            if (estop) begin
                fault_active  <= 1'b1;
                fault_code[0] <= 1'b1;
            end
            if (invalid_cmd) begin
                fault_active  <= 1'b1;
                fault_code[1] <= 1'b1;
            end
            if (timeout_tripped) begin
                fault_active  <= 1'b1;
                fault_code[2] <= 1'b1;
            end
            if (|limit_clamped) begin
                fault_code[3] <= 1'b1;
            end

            if (fault_clear && !estop && !invalid_cmd && !timeout_tripped &&
                ((command == CMD_IDLE) || (command == CMD_STOP))) begin
                fault_active <= 1'b0;
                fault_code   <= 4'b0000;
            end
        end
    end

    assign safe_j0 = fault_override ? SAFE_PULSE_US[11:0] : c_j0;
    assign safe_j1 = fault_override ? SAFE_PULSE_US[11:0] : c_j1;
    assign safe_j2 = fault_override ? SAFE_PULSE_US[11:0] : c_j2;
    assign safe_j3 = fault_override ? SAFE_PULSE_US[11:0] : c_j3;
    assign safe_j4 = fault_override ? SAFE_PULSE_US[11:0] : c_j4;
    assign safe_j5 = fault_override ? SAFE_PULSE_US[11:0] : c_j5;
    assign safe_j6 = fault_override ? SAFE_PULSE_US[11:0] : c_j6;
    assign safe_j7 = fault_override ? SAFE_PULSE_US[11:0] : c_j7;

endmodule
