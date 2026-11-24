// =============================================================================
// Module      : gait_controller
// Project     : XEROTON Studio 3.0 - 8-DOF Quadruped Robot
// Description : Deterministic 15-state Finite State Machine (FSM) with shared
//               differential/lateral gait generators, hardware slew-rate
//               interpolation, previous-state tracking, 16-bit frame counter,
//               and runtime-configurable stride & cadence parameters.
//
// Commands (4-bit):
//   4'd0  : CMD_IDLE          -> STATE_IDLE (0)
//   4'd1  : CMD_STAND         -> STATE_STAND (1)
//   4'd2  : CMD_WALK_FORWARD  -> STATE_WALK_FORWARD (2)
//   4'd3  : CMD_STOP          -> STATE_STOP (3)
//   4'd4  : CMD_WAVE          -> STATE_WAVE (5)
//   4'd5  : CMD_DANCE         -> STATE_DANCE (6)
//   4'd6  : CMD_SIT           -> STATE_SIT (7)
//   4'd7  : CMD_TROT          -> STATE_TROT (8)
//   4'd8  : CMD_SLEEP         -> STATE_SLEEP (9)
//   4'd9  : CMD_WALK_BACKWARD -> STATE_WALK_BACKWARD (10)
//   4'd10 : CMD_STRAFE_LEFT   -> STATE_STRAFE_LEFT (11)
//   4'd11 : CMD_STRAFE_RIGHT  -> STATE_STRAFE_RIGHT (12)
//   4'd12 : CMD_TURN_LEFT     -> STATE_TURN_LEFT (13)
//   4'd13 : CMD_TURN_RIGHT    -> STATE_TURN_RIGHT (14)
//   4'd14..15 : Reserved / Invalid -> STATE_FAULT (4)
// =============================================================================
`timescale 1ns/1ps

module gait_controller #(
    parameter integer FRAMES_PER_PHASE = 4,
    parameter integer MAX_SLEW_US      = 50
)(
    input  wire        clk,
    input  wire        reset,
    input  wire        frame_tick,
    input  wire [3:0]  command,
    input  wire        cmd_valid,
    input  wire        estop,
    input  wire        fault_active,

    // Runtime gait tuning inputs (0 = use built-in safe default)
    input  wire [7:0]  stride_us,
    input  wire [3:0]  phase_frames,

    output reg  [11:0] j0,
    output reg  [11:0] j1,
    output reg  [11:0] j2,
    output reg  [11:0] j3,
    output reg  [11:0] j4,
    output reg  [11:0] j5,
    output reg  [11:0] j6,
    output reg  [11:0] j7,

    output reg  [3:0]  state,
    output reg  [3:0]  prev_state,
    output reg  [2:0]  gait_phase,
    output reg  [15:0] frame_counter
);

    localparam [3:0] STATE_IDLE          = 4'd0;
    localparam [3:0] STATE_STAND         = 4'd1;
    localparam [3:0] STATE_WALK_FORWARD  = 4'd2;
    localparam [3:0] STATE_STOP          = 4'd3;
    localparam [3:0] STATE_FAULT         = 4'd4;
    localparam [3:0] STATE_WAVE          = 4'd5;
    localparam [3:0] STATE_DANCE         = 4'd6;
    localparam [3:0] STATE_SIT           = 4'd7;
    localparam [3:0] STATE_TROT          = 4'd8;
    localparam [3:0] STATE_SLEEP         = 4'd9;
    localparam [3:0] STATE_WALK_BACKWARD = 4'd10;
    localparam [3:0] STATE_STRAFE_LEFT   = 4'd11;
    localparam [3:0] STATE_STRAFE_RIGHT  = 4'd12;
    localparam [3:0] STATE_TURN_LEFT     = 4'd13;
    localparam [3:0] STATE_TURN_RIGHT    = 4'd14;

    localparam [3:0] CMD_IDLE            = 4'd0;
    localparam [3:0] CMD_STAND           = 4'd1;
    localparam [3:0] CMD_WALK_FORWARD    = 4'd2;
    localparam [3:0] CMD_STOP            = 4'd3;
    localparam [3:0] CMD_WAVE            = 4'd4;
    localparam [3:0] CMD_DANCE           = 4'd5;
    localparam [3:0] CMD_SIT             = 4'd6;
    localparam [3:0] CMD_TROT            = 4'd7;
    localparam [3:0] CMD_SLEEP           = 4'd8;
    localparam [3:0] CMD_WALK_BACKWARD   = 4'd9;
    localparam [3:0] CMD_STRAFE_LEFT     = 4'd10;
    localparam [3:0] CMD_STRAFE_RIGHT    = 4'd11;
    localparam [3:0] CMD_TURN_LEFT       = 4'd12;
    localparam [3:0] CMD_TURN_RIGHT      = 4'd13;

    localparam [11:0] NEUTRAL_US   = 12'd1500;
    localparam [11:0] STAND_HIP    = 12'd1350;
    localparam [11:0] STAND_KNEE   = 12'd1800;

    // Safely bounded effective stride and cadence
    wire [7:0] eff_walk_stride = (stride_us == 8'd0)  ? 8'd75  :
                                 (stride_us < 8'd30)  ? 8'd30  :
                                 (stride_us > 8'd110) ? 8'd110 : stride_us;

    wire [7:0] eff_trot_stride = (stride_us == 8'd0)  ? 8'd65  :
                                 (stride_us < 8'd35)  ? 8'd35  :
                                 (stride_us > 8'd105) ? 8'd105 : stride_us;

    // Balanced differential turn strides (tested in PyBullet for pure in-place yaw with minimal X drift)
    wire [7:0] eff_turn_fwd    = eff_walk_stride;
    wire [7:0] eff_turn_back   = (eff_walk_stride > 8'd105) ? 8'd105 : (eff_walk_stride + 8'd4);

    wire [7:0] walk_div_limit  = (phase_frames == 4'd0) ? FRAMES_PER_PHASE[7:0] :
                                 (phase_frames < 4'd2)  ? 8'd2 :
                                 (phase_frames > 4'd12) ? 8'd12 : {4'd0, phase_frames};

    wire [7:0] trot_div_limit  = (phase_frames == 4'd0) ? 8'd3 :
                                 (phase_frames < 4'd2)  ? 8'd2 :
                                 (phase_frames > 4'd10) ? 8'd10 : {4'd0, phase_frames};

    wire [11:0] walk_hip_fwd   = STAND_HIP - {4'd0, eff_walk_stride};
    wire [11:0] walk_hip_back  = STAND_HIP + {4'd0, eff_walk_stride};
    wire [11:0] trot_hip_fwd   = STAND_HIP - {4'd0, eff_trot_stride};
    wire [11:0] trot_hip_back  = STAND_HIP + {4'd0, eff_trot_stride};

    wire [11:0] turn_fwd_sw    = STAND_HIP - {4'd0, eff_turn_fwd};
    wire [11:0] turn_fwd_st    = STAND_HIP + {4'd0, eff_turn_fwd};
    wire [11:0] turn_back_sw   = STAND_HIP + {4'd0, eff_turn_back};
    wire [11:0] turn_back_st   = STAND_HIP - {4'd0, eff_turn_back};

    localparam [11:0] KNEE_LIFT_WALK  = 12'd1940;
    localparam [11:0] KNEE_LIFT_TROT  = 12'd1915;
    localparam [11:0] KNEE_STANCE     = 12'd1800;

    // Zero-X vertical poses for coordinated lateral weight-shift strafing
    localparam [11:0] STRAFE_EXT_HIP  = 12'd1385;
    localparam [11:0] STRAFE_EXT_KNEE = 12'd1730;
    localparam [11:0] STRAFE_LFT_HIP  = 12'd1272;
    localparam [11:0] STRAFE_LFT_KNEE = 12'd1940;

    reg [7:0] phase_div;

    function [11:0] slew_toward;
        input [11:0] current_us;
        input [11:0] target_us;
        begin
            if (target_us > current_us) begin
                if ((target_us - current_us) > MAX_SLEW_US[11:0])
                    slew_toward = current_us + MAX_SLEW_US[11:0];
                else
                    slew_toward = target_us;
            end else begin
                if ((current_us - target_us) > MAX_SLEW_US[11:0])
                    slew_toward = current_us - MAX_SLEW_US[11:0];
                else
                    slew_toward = target_us;
            end
        end
    endfunction

    task ramp_to_pose;
        input [11:0] g0, g1, g2, g3, g4, g5, g6, g7;
        begin
            j0 <= slew_toward(j0, g0);
            j1 <= slew_toward(j1, g1);
            j2 <= slew_toward(j2, g2);
            j3 <= slew_toward(j3, g3);
            j4 <= slew_toward(j4, g4);
            j5 <= slew_toward(j5, g5);
            j6 <= slew_toward(j6, g6);
            j7 <= slew_toward(j7, g7);
        end
    endtask

    task set_neutral_immediate;
        begin
            j0 <= NEUTRAL_US; j1 <= NEUTRAL_US;
            j2 <= NEUTRAL_US; j3 <= NEUTRAL_US;
            j4 <= NEUTRAL_US; j5 <= NEUTRAL_US;
            j6 <= NEUTRAL_US; j7 <= NEUTRAL_US;
        end
    endtask

    task enter_state;
        input [3:0] next_s;
        begin
            if (next_s != state) begin
                prev_state <= state;
                state      <= next_s;
            end
        end
    endtask

    // Shared 8-phase lift-first diagonal gait generator supporting:
    //   WALK_FORWARD  : l_sw=fwd,  l_st=back, r_sw=fwd,  r_st=back
    //   WALK_BACKWARD : l_sw=back, l_st=fwd,  r_sw=back, r_st=fwd
    //   TURN_LEFT     : l_sw=back, l_st=fwd,  r_sw=fwd,  r_st=back (CCW +yaw)
    //   TURN_RIGHT    : l_sw=fwd,  l_st=back, r_sw=back, r_st=fwd  (CW  -yaw)
    task apply_diff_locomotion_phase;
        input [2:0]  p;
        input [11:0] l_sw;
        input [11:0] l_st;
        input [11:0] r_sw;
        input [11:0] r_st;
        input [11:0] k_lift;
        begin
            case (p)
                3'd0: ramp_to_pose(STAND_HIP, k_lift,      STAND_HIP, KNEE_STANCE, STAND_HIP, KNEE_STANCE, STAND_HIP, k_lift);
                3'd1: ramp_to_pose(l_sw,      k_lift,      r_st,      KNEE_STANCE, l_st,      KNEE_STANCE, r_sw,      k_lift);
                3'd2: ramp_to_pose(l_sw,      KNEE_STANCE, r_st,      KNEE_STANCE, l_st,      KNEE_STANCE, r_sw,      KNEE_STANCE);
                3'd3: ramp_to_pose(l_sw,      KNEE_STANCE, r_st,      k_lift,      l_st,      k_lift,      r_sw,      KNEE_STANCE);
                3'd4: ramp_to_pose(STAND_HIP, KNEE_STANCE, STAND_HIP, k_lift,      STAND_HIP, k_lift,      STAND_HIP, KNEE_STANCE);
                3'd5: ramp_to_pose(l_st,      KNEE_STANCE, r_sw,      k_lift,      l_sw,      k_lift,      r_st,      KNEE_STANCE);
                3'd6: ramp_to_pose(l_st,      KNEE_STANCE, r_sw,      KNEE_STANCE, l_sw,      KNEE_STANCE, r_st,      KNEE_STANCE);
                default:
                      ramp_to_pose(l_st,      k_lift,      r_sw,      KNEE_STANCE, l_sw,      KNEE_STANCE, r_st,      k_lift);
            endcase
        end
    endtask

    // Shared 8-phase alternating lateral weight-shift & step gait (zero-X drift)
    // dir_left=1 -> STRAFE_LEFT (+Y), dir_left=0 -> STRAFE_RIGHT (-Y)
    task apply_strafe_phase;
        input [2:0] p;
        input       dir_left;
        begin
            if (dir_left) begin
                case (p)
                    3'd0, 3'd1, 3'd6:
                        ramp_to_pose(STRAFE_LFT_HIP, STRAFE_LFT_KNEE, STRAFE_EXT_HIP, STRAFE_EXT_KNEE,
                                     STAND_HIP,      STAND_KNEE,      STRAFE_LFT_HIP, STRAFE_LFT_KNEE);
                    3'd3, 3'd4:
                        ramp_to_pose(STAND_HIP,      STAND_KNEE,      STRAFE_LFT_HIP, STRAFE_LFT_KNEE,
                                     STRAFE_LFT_HIP, STRAFE_LFT_KNEE, STRAFE_EXT_HIP, STRAFE_EXT_KNEE);
                    default:
                        ramp_to_pose(STAND_HIP,      STAND_KNEE,      STAND_HIP,      STAND_KNEE,
                                     STAND_HIP,      STAND_KNEE,      STAND_HIP,      STAND_KNEE);
                endcase
            end else begin
                case (p)
                    3'd0, 3'd1, 3'd6:
                        ramp_to_pose(STRAFE_EXT_HIP, STRAFE_EXT_KNEE, STRAFE_LFT_HIP, STRAFE_LFT_KNEE,
                                     STRAFE_LFT_HIP, STRAFE_LFT_KNEE, STAND_HIP,      STAND_KNEE);
                    3'd3, 3'd4:
                        ramp_to_pose(STRAFE_LFT_HIP, STRAFE_LFT_KNEE, STAND_HIP,      STAND_KNEE,
                                     STRAFE_EXT_HIP, STRAFE_EXT_KNEE, STRAFE_LFT_HIP, STRAFE_LFT_KNEE);
                    default:
                        ramp_to_pose(STAND_HIP,      STAND_KNEE,      STAND_HIP,      STAND_KNEE,
                                     STAND_HIP,      STAND_KNEE,      STAND_HIP,      STAND_KNEE);
                endcase
            end
        end
    endtask

    task apply_wave_phase;
        input [2:0] p;
        begin
            case (p)
                3'd0: ramp_to_pose(12'd1420, 12'd1720, 12'd1140, 12'd1420, 12'd1360, 12'd1900, 12'd1360, 12'd1900);
                3'd1, 3'd3, 3'd5:
                      ramp_to_pose(12'd1420, 12'd1720, 12'd1050, 12'd1100, 12'd1360, 12'd1900, 12'd1360, 12'd1900);
                3'd2, 3'd4, 3'd6:
                      ramp_to_pose(12'd1420, 12'd1720, 12'd1050, 12'd1380, 12'd1360, 12'd1900, 12'd1360, 12'd1900);
                default:
                      ramp_to_pose(12'd1420, 12'd1720, 12'd1100, 12'd1220, 12'd1360, 12'd1900, 12'd1360, 12'd1900);
            endcase
        end
    endtask

    task apply_dance_phase;
        input [2:0] p;
        begin
            case (p)
                3'd0, 3'd1:
                      ramp_to_pose(12'd1230, 12'd1940, 12'd1230, 12'd1940, 12'd1420, 12'd1660, 12'd1420, 12'd1660);
                3'd2: ramp_to_pose(12'd1300, 12'd1920, 12'd1400, 12'd1680, 12'd1300, 12'd1920, 12'd1400, 12'd1680);
                3'd3: ramp_to_pose(12'd1400, 12'd1680, 12'd1300, 12'd1920, 12'd1400, 12'd1680, 12'd1300, 12'd1920);
                3'd4: ramp_to_pose(12'd1420, 12'd1660, 12'd1420, 12'd1660, 12'd1230, 12'd1940, 12'd1230, 12'd1940);
                3'd5: ramp_to_pose(12'd1300, 12'd1920, 12'd1400, 12'd1680, 12'd1300, 12'd1920, 12'd1400, 12'd1680);
                3'd6: ramp_to_pose(12'd1400, 12'd1680, 12'd1300, 12'd1920, 12'd1400, 12'd1680, 12'd1300, 12'd1920);
                default:
                      ramp_to_pose(12'd1380, 12'd1720, 12'd1380, 12'd1720, 12'd1380, 12'd1720, 12'd1380, 12'd1720);
            endcase
        end
    endtask

    task advance_phase_with;
        input [7:0] div_limit;
        begin
            if (phase_div >= div_limit - 8'd1) begin
                phase_div  <= 8'd0;
                gait_phase <= gait_phase + 3'd1;
            end else begin
                phase_div  <= phase_div + 8'd1;
            end
        end
    endtask

    task run_cyclic_state;
        input [3:0] target_state;
        input [7:0] div_limit;
        begin
            if (state != target_state) begin
                enter_state(target_state);
                gait_phase <= 3'd0;
                phase_div  <= 8'd0;
                case (target_state)
                    STATE_WALK_FORWARD:  apply_diff_locomotion_phase(3'd0, walk_hip_fwd, walk_hip_back, walk_hip_fwd, walk_hip_back, KNEE_LIFT_WALK);
                    STATE_WALK_BACKWARD: apply_diff_locomotion_phase(3'd0, walk_hip_back, walk_hip_fwd, walk_hip_back, walk_hip_fwd, KNEE_LIFT_WALK);
                    STATE_TROT:          apply_diff_locomotion_phase(3'd0, trot_hip_fwd, trot_hip_back, trot_hip_fwd, trot_hip_back, KNEE_LIFT_TROT);
                    STATE_TURN_LEFT:     apply_diff_locomotion_phase(3'd0, turn_back_sw, turn_back_st, turn_fwd_sw, turn_fwd_st, KNEE_LIFT_WALK);
                    STATE_TURN_RIGHT:    apply_diff_locomotion_phase(3'd0, turn_fwd_sw, turn_fwd_st, turn_back_sw, turn_back_st, KNEE_LIFT_WALK);
                    STATE_STRAFE_LEFT:   apply_strafe_phase(3'd0, 1'b1);
                    STATE_STRAFE_RIGHT:  apply_strafe_phase(3'd0, 1'b0);
                    STATE_WAVE:          apply_wave_phase(3'd0);
                    STATE_DANCE:         apply_dance_phase(3'd0);
                    default: ;
                endcase
            end else begin
                case (target_state)
                    STATE_WALK_FORWARD:  apply_diff_locomotion_phase(gait_phase, walk_hip_fwd, walk_hip_back, walk_hip_fwd, walk_hip_back, KNEE_LIFT_WALK);
                    STATE_WALK_BACKWARD: apply_diff_locomotion_phase(gait_phase, walk_hip_back, walk_hip_fwd, walk_hip_back, walk_hip_fwd, KNEE_LIFT_WALK);
                    STATE_TROT:          apply_diff_locomotion_phase(gait_phase, trot_hip_fwd, trot_hip_back, trot_hip_fwd, trot_hip_back, KNEE_LIFT_TROT);
                    STATE_TURN_LEFT:     apply_diff_locomotion_phase(gait_phase, turn_back_sw, turn_back_st, turn_fwd_sw, turn_fwd_st, KNEE_LIFT_WALK);
                    STATE_TURN_RIGHT:    apply_diff_locomotion_phase(gait_phase, turn_fwd_sw, turn_fwd_st, turn_back_sw, turn_back_st, KNEE_LIFT_WALK);
                    STATE_STRAFE_LEFT:   apply_strafe_phase(gait_phase, 1'b1);
                    STATE_STRAFE_RIGHT:  apply_strafe_phase(gait_phase, 1'b0);
                    STATE_WAVE:          apply_wave_phase(gait_phase);
                    STATE_DANCE:         apply_dance_phase(gait_phase);
                    default: ;
                endcase
                advance_phase_with(div_limit);
            end
        end
    endtask

    always @(posedge clk) begin
        if (reset) begin
            state         <= STATE_IDLE;
            prev_state    <= STATE_IDLE;
            gait_phase    <= 3'd0;
            phase_div     <= 8'd0;
            frame_counter <= 16'd0;
            set_neutral_immediate();
        end else if (estop || fault_active) begin
            enter_state(STATE_FAULT);
            gait_phase <= 3'd0;
            phase_div  <= 8'd0;
            set_neutral_immediate();
            if (frame_tick) frame_counter <= frame_counter + 16'd1;
        end else if (frame_tick) begin
            frame_counter <= frame_counter + 16'd1;

            if (cmd_valid) begin
                case (command)
                    CMD_IDLE: begin
                        enter_state(STATE_IDLE);
                        gait_phase <= 3'd0; phase_div <= 8'd0;
                        ramp_to_pose(NEUTRAL_US, NEUTRAL_US, NEUTRAL_US, NEUTRAL_US,
                                     NEUTRAL_US, NEUTRAL_US, NEUTRAL_US, NEUTRAL_US);
                    end
                    CMD_STAND: begin
                        enter_state(STATE_STAND);
                        gait_phase <= 3'd0; phase_div <= 8'd0;
                        ramp_to_pose(STAND_HIP, STAND_KNEE, STAND_HIP, STAND_KNEE,
                                     STAND_HIP, STAND_KNEE, STAND_HIP, STAND_KNEE);
                    end
                    CMD_STOP: begin
                        enter_state(STATE_STOP);
                        gait_phase <= 3'd0; phase_div <= 8'd0;
                        ramp_to_pose(STAND_HIP, STAND_KNEE, STAND_HIP, STAND_KNEE,
                                     STAND_HIP, STAND_KNEE, STAND_HIP, STAND_KNEE);
                    end
                    CMD_WALK_FORWARD:  run_cyclic_state(STATE_WALK_FORWARD,  walk_div_limit);
                    CMD_WALK_BACKWARD: run_cyclic_state(STATE_WALK_BACKWARD, walk_div_limit);
                    CMD_STRAFE_LEFT:   run_cyclic_state(STATE_STRAFE_LEFT,   walk_div_limit);
                    CMD_STRAFE_RIGHT:  run_cyclic_state(STATE_STRAFE_RIGHT,  walk_div_limit);
                    CMD_TURN_LEFT:     run_cyclic_state(STATE_TURN_LEFT,     walk_div_limit);
                    CMD_TURN_RIGHT:    run_cyclic_state(STATE_TURN_RIGHT,    walk_div_limit);
                    CMD_TROT:          run_cyclic_state(STATE_TROT,          trot_div_limit);
                    CMD_WAVE:          run_cyclic_state(STATE_WAVE,          walk_div_limit);
                    CMD_DANCE:         run_cyclic_state(STATE_DANCE,         walk_div_limit);
                    CMD_SIT: begin
                        enter_state(STATE_SIT);
                        gait_phase <= 3'd0; phase_div <= 8'd0;
                        ramp_to_pose(12'd1430, 12'd1640, 12'd1430, 12'd1640,
                                     12'd1250, 12'd1940, 12'd1250, 12'd1940);
                    end
                    CMD_SLEEP: begin
                        enter_state(STATE_SLEEP);
                        gait_phase <= 3'd0; phase_div <= 8'd0;
                        ramp_to_pose(12'd1240, 12'd1940, 12'd1240, 12'd1940,
                                     12'd1240, 12'd1940, 12'd1240, 12'd1940);
                    end
                    default: begin
                        enter_state(STATE_FAULT);
                        gait_phase <= 3'd0; phase_div <= 8'd0;
                        set_neutral_immediate();
                    end
                endcase
            end else begin
                // Hold/continue current state deterministically without ever reversing direction
                case (state)
                    STATE_WALK_FORWARD:  run_cyclic_state(STATE_WALK_FORWARD,  walk_div_limit);
                    STATE_WALK_BACKWARD: run_cyclic_state(STATE_WALK_BACKWARD, walk_div_limit);
                    STATE_STRAFE_LEFT:   run_cyclic_state(STATE_STRAFE_LEFT,   walk_div_limit);
                    STATE_STRAFE_RIGHT:  run_cyclic_state(STATE_STRAFE_RIGHT,  walk_div_limit);
                    STATE_TURN_LEFT:     run_cyclic_state(STATE_TURN_LEFT,     walk_div_limit);
                    STATE_TURN_RIGHT:    run_cyclic_state(STATE_TURN_RIGHT,    walk_div_limit);
                    STATE_TROT:          run_cyclic_state(STATE_TROT,          trot_div_limit);
                    STATE_WAVE:          run_cyclic_state(STATE_WAVE,          walk_div_limit);
                    STATE_DANCE:         run_cyclic_state(STATE_DANCE,         walk_div_limit);
                    default: ;
                endcase
            end
        end
    end

endmodule
