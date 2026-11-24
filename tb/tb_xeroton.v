// =============================================================================
// Testbench   : tb_xeroton
// Project     : XEROTON Studio 3.0 - 8-DOF Quadruped Robot
// Description : Self-checking comprehensive verification testbench verifying:
//                 1. Reset behavior & frame_counter
//                 2. IDLE, STAND, WALK, TROT, WAVE, DANCE, SIT, STOP, FAULT
//                 3. prev_state transition tracking
//                 4. Emergency Stop (1-clock override + latched fault + clear)
//                 5. Invalid command handling (cmd=9) & watchdog timeout
//                 6. Pulse-width limit clamping (servo_pwm & safety_controller)
//                 7. Exact 50.00 Hz (20.000 ms) PWM period & pulse width timing
// =============================================================================
`timescale 1ns/1ps

module tb_xeroton;

    localparam integer CLK_HZ           = 1000000;
    localparam integer PWM_HZ           = 50;
    localparam integer FRAMES_PER_PHASE = 4;
    localparam integer TIMEOUT_FRAMES   = 10;

    localparam [3:0] CMD_IDLE          = 4'd0;
    localparam [3:0] CMD_STAND         = 4'd1;
    localparam [3:0] CMD_WALK          = 4'd2;
    localparam [3:0] CMD_STOP          = 4'd3;
    localparam [3:0] CMD_WAVE          = 4'd4;
    localparam [3:0] CMD_DANCE         = 4'd5;
    localparam [3:0] CMD_SIT           = 4'd6;
    localparam [3:0] CMD_TROT          = 4'd7;
    localparam [3:0] CMD_SLEEP         = 4'd8;
    localparam [3:0] CMD_WALK_BACKWARD = 4'd9;
    localparam [3:0] CMD_STRAFE_LEFT   = 4'd10;
    localparam [3:0] CMD_STRAFE_RIGHT  = 4'd11;
    localparam [3:0] CMD_TURN_LEFT     = 4'd12;
    localparam [3:0] CMD_TURN_RIGHT    = 4'd13;

    localparam [3:0] STATE_IDLE          = 4'd0;
    localparam [3:0] STATE_STAND         = 4'd1;
    localparam [3:0] STATE_WALK          = 4'd2;
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

    reg clk = 1'b0;
    always #500 clk = ~clk;

    reg         reset           = 1'b1;
    reg  [3:0]  command         = CMD_IDLE;
    reg         cmd_valid       = 1'b1;
    reg         estop           = 1'b0;
    reg         fault_clear     = 1'b0;
    reg         watchdog_enable = 1'b0;
    reg  [7:0]  stride_us       = 8'd0;
    reg  [3:0]  phase_frames    = 4'd0;

    wire [7:0]  servo_pwm;
    wire        frame_tick;
    wire [3:0]  state;
    wire [3:0]  prev_state;
    wire [2:0]  gait_phase;
    wire [15:0] frame_counter;
    wire        fault_active;
    wire [3:0]  fault_code;
    wire [7:0]  limit_clamped;
    wire [7:0]  pwm_clamped;
    wire [11:0] j0, j1, j2, j3, j4, j5, j6, j7;

    xeroton_top #(
        .CLK_HZ           (CLK_HZ),
        .PWM_HZ           (PWM_HZ),
        .FRAMES_PER_PHASE (FRAMES_PER_PHASE),
        .TIMEOUT_FRAMES   (TIMEOUT_FRAMES),
        .MIN_PULSE_US     (1000),
        .MAX_PULSE_US     (2000),
        .SAFE_PULSE_US    (1500)
    ) dut (
        .clk             (clk),
        .reset           (reset),
        .command         (command),
        .cmd_valid       (cmd_valid),
        .estop           (estop),
        .fault_clear     (fault_clear),
        .watchdog_enable (watchdog_enable),
        .stride_us       (stride_us),
        .phase_frames    (phase_frames),
        .servo_pwm       (servo_pwm),
        .frame_tick      (frame_tick),
        .state           (state),
        .prev_state      (prev_state),
        .gait_phase      (gait_phase),
        .frame_counter   (frame_counter),
        .fault_active    (fault_active),
        .fault_code      (fault_code),
        .limit_clamped   (limit_clamped),
        .pwm_clamped     (pwm_clamped),
        .j0              (j0),
        .j1              (j1),
        .j2              (j2),
        .j3              (j3),
        .j4              (j4),
        .j5              (j5),
        .j6              (j6),
        .j7              (j7)
    );

    // Standalone pulse-width limit verification
    reg  [11:0] lim_test_pulse_us = 12'd1500;
    wire        lim_pwm_out;
    wire        lim_frame_tick;
    wire        lim_pulse_clamped;

    servo_pwm #(
        .CLK_HZ        (CLK_HZ),
        .PWM_HZ        (PWM_HZ),
        .MIN_PULSE_US  (1000),
        .MAX_PULSE_US  (2000),
        .SAFE_PULSE_US (1500)
    ) u_pwm_limit_check (
        .clk           (clk),
        .reset         (reset),
        .pulse_us      (lim_test_pulse_us),
        .pwm           (lim_pwm_out),
        .frame_tick    (lim_frame_tick),
        .pulse_clamped (lim_pulse_clamped)
    );

    reg  [11:0] sc_raw_j0 = 12'd800;
    reg  [11:0] sc_raw_j1 = 12'd2300;
    wire [11:0] sc_safe_j0, sc_safe_j1, sc_safe_j2, sc_safe_j3;
    wire [11:0] sc_safe_j4, sc_safe_j5, sc_safe_j6, sc_safe_j7;
    wire        sc_fault_active;
    wire [3:0]  sc_fault_code;
    wire [7:0]  sc_limit_clamped;

    safety_controller #(
        .HIP_MIN_US     (1050),
        .HIP_MAX_US     (1900),
        .KNEE_MIN_US    (1050),
        .KNEE_MAX_US    (1950),
        .SAFE_PULSE_US  (1500),
        .TIMEOUT_FRAMES (TIMEOUT_FRAMES)
    ) u_safety_limit_check (
        .clk             (clk),
        .reset           (reset),
        .frame_tick      (frame_tick),
        .command         (CMD_IDLE),
        .cmd_valid       (1'b1),
        .estop           (1'b0),
        .fault_clear     (1'b0),
        .watchdog_enable (1'b0),
        .raw_j0          (sc_raw_j0),
        .raw_j1          (sc_raw_j1),
        .raw_j2          (12'd1500),
        .raw_j3          (12'd1500),
        .raw_j4          (12'd1500),
        .raw_j5          (12'd1500),
        .raw_j6          (12'd1500),
        .raw_j7          (12'd1500),
        .safe_j0         (sc_safe_j0),
        .safe_j1         (sc_safe_j1),
        .safe_j2         (sc_safe_j2),
        .safe_j3         (sc_safe_j3),
        .safe_j4         (sc_safe_j4),
        .safe_j5         (sc_safe_j5),
        .safe_j6         (sc_safe_j6),
        .safe_j7         (sc_safe_j7),
        .fault_active    (sc_fault_active),
        .fault_override  (),
        .fault_code      (sc_fault_code),
        .limit_clamped   (sc_limit_clamped)
    );

    integer pass_count = 0;
    integer fail_count = 0;
    integer csv_fd;
    integer frame_num  = 0;

    task check_cond;
        input         cond;
        input [511:0] msg;
        begin
            if (cond) pass_count = pass_count + 1;
            else begin
                fail_count = fail_count + 1;
                $display("[FAIL] t=%0.3f ms : %0s", $realtime / 1.0e6, msg);
            end
        end
    endtask

    function [127:0] sname;
        input [3:0] s;
        begin
            case (s)
                STATE_IDLE:          sname = "IDLE";
                STATE_STAND:         sname = "STAND";
                STATE_WALK:          sname = "WALK";
                STATE_STOP:          sname = "STOP";
                STATE_FAULT:         sname = "FAULT";
                STATE_WAVE:          sname = "WAVE";
                STATE_DANCE:         sname = "DANCE";
                STATE_SIT:           sname = "SIT";
                STATE_TROT:          sname = "TROT";
                STATE_SLEEP:         sname = "SLEEP";
                STATE_WALK_BACKWARD: sname = "WALK_BACKWARD";
                STATE_STRAFE_LEFT:   sname = "STRAFE_LEFT";
                STATE_STRAFE_RIGHT:  sname = "STRAFE_RIGHT";
                STATE_TURN_LEFT:     sname = "TURN_LEFT";
                STATE_TURN_RIGHT:    sname = "TURN_RIGHT";
                default:             sname = "FAULT";
            endcase
        end
    endfunction

    // Real-time PWM period & pulse monitors
    realtime ch0_rise_prev = 0.0, ch0_rise_curr = 0.0;
    realtime ch0_period_us = 0.0, ch0_pulse_us  = 0.0;
    integer  ch0_edges     = 0;
    realtime ch1_rise_curr = 0.0, ch1_pulse_us  = 0.0;
    realtime lim_rise_curr = 0.0, lim_pulse_us  = 0.0;
    integer  period_checks_ok = 0, pulse_checks_ok = 0;

    always @(posedge servo_pwm[0]) begin
        if (!reset) begin
            ch0_rise_prev = ch0_rise_curr;
            ch0_rise_curr = $realtime;
            ch0_edges     = ch0_edges + 1;
            if (ch0_edges >= 2) begin
                ch0_period_us = (ch0_rise_curr - ch0_rise_prev) / 1000.0;
                if ((ch0_period_us >= 19999.0) && (ch0_period_us <= 20001.0))
                    period_checks_ok = period_checks_ok + 1;
                else begin
                    fail_count = fail_count + 1;
                    $display("[FAIL] CH0 PWM period = %0.2f us", ch0_period_us);
                end
            end
        end
    end

    always @(negedge servo_pwm[0]) begin
        if (!reset && ch0_edges >= 1) begin
            ch0_pulse_us = ($realtime - ch0_rise_curr) / 1000.0;
            if ((ch0_pulse_us >= (j0 - 1.0)) && (ch0_pulse_us <= (j0 + 1.0)))
                pulse_checks_ok = pulse_checks_ok + 1;
            else begin
                fail_count = fail_count + 1;
                $display("[FAIL] CH0 PWM pulse = %0.2f us (expected j0=%0d)", ch0_pulse_us, j0);
            end
        end
    end

    always @(posedge servo_pwm[1]) begin
        if (!reset) ch1_rise_curr = $realtime;
    end

    always @(negedge servo_pwm[1]) begin
        if (!reset && ch1_rise_curr > 0.0) begin
            ch1_pulse_us = ($realtime - ch1_rise_curr) / 1000.0;
            if ((ch1_pulse_us >= (j1 - 1.0)) && (ch1_pulse_us <= (j1 + 1.0)))
                pulse_checks_ok = pulse_checks_ok + 1;
            else begin
                fail_count = fail_count + 1;
                $display("[FAIL] CH1 PWM pulse = %0.2f us (expected j1=%0d)", ch1_pulse_us, j1);
            end
        end
    end

    always @(posedge lim_pwm_out) if (!reset) lim_rise_curr = $realtime;
    always @(negedge lim_pwm_out) if (!reset && lim_rise_curr > 0.0) lim_pulse_us = ($realtime - lim_rise_curr) / 1000.0;

    reg [7:0] phases_seen = 8'b0;
    reg       logging_en  = 1'b0;

    always @(posedge frame_tick) begin
        @(posedge clk);
        #1;
        if (!reset && logging_en) begin
            frame_num = frame_num + 1;
            if (state == STATE_WALK) phases_seen[gait_phase] = 1'b1;
            $fdisplay(csv_fd,
                "%0d,%0.2f,%0d,%0d,%0d,%0s,%0s,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d",
                frame_num,
                $realtime / 1.0e6,
                command,
                state,
                prev_state,
                sname(state),
                sname(prev_state),
                gait_phase,
                fault_active,
                fault_code,
                j0, j1, j2, j3, j4, j5, j6, j7
            );
        end
    end

    task wait_frames;
        input integer count_n;
        integer k;
        begin
            for (k = 0; k < count_n; k = k + 1) @(negedge frame_tick);
        end
    endtask

    initial begin
        $display("======================================================================");
        $display(" XEROTON Studio 3.0 Verilog RTL Verification Testbench (9-State FSM)");
        $display(" Clock = %0d Hz | PWM = %0d Hz (20.000 ms frame)", CLK_HZ, PWM_HZ);
        $display("======================================================================");

        $dumpfile("simulation/xeroton.vcd");
        $dumpvars(1, tb_xeroton);
        $dumpvars(1, dut);

        csv_fd = $fopen("simulation/joint_targets.csv", "w");
        if (csv_fd == 0) begin
            $display("[FATAL] Could not open simulation/joint_targets.csv.");
            $finish;
        end
        $fdisplay(csv_fd,
            "frame,time_ms,command,state,prev_state,state_name,prev_state_name,gait_phase,fault_active,fault_code,j0_us,j1_us,j2_us,j3_us,j4_us,j5_us,j6_us,j7_us");

        // TEST 1: RESET
        $display("[TEST 1] Verifying synchronous reset & frame_counter...");
        reset = 1'b1;
        command = CMD_IDLE; cmd_valid = 1'b1; estop = 1'b0; fault_clear = 1'b0; watchdog_enable = 1'b0;
        #10000;
        check_cond(state == STATE_IDLE && prev_state == STATE_IDLE, "Reset: state & prev_state == IDLE");
        check_cond(frame_counter == 16'd0,                          "Reset: frame_counter == 0");
        check_cond(servo_pwm == 8'b0,                               "Reset: all 8 PWM outputs low");
        check_cond(fault_active == 1'b0,                            "Reset: fault_active == 0");
        check_cond(j0 == 12'd1500 && j7 == 12'd1500,                "Reset: neutral 1500 us");

        @(negedge clk);
        reset = 1'b0;
        logging_en = 1'b1;

        // TEST 2: IDLE & Low Limit Clamp
        $display("[TEST 2] Verifying IDLE & low PWM clamp limit...");
        lim_test_pulse_us = 12'd500;
        command = CMD_IDLE;
        wait_frames(10);
        check_cond(state == STATE_IDLE,                              "IDLE: state remains IDLE");
        check_cond(frame_counter >= 16'd9,                           "IDLE: frame_counter incrementing");
        check_cond(lim_pulse_clamped == 1'b1,                        "Limit low: pulse_clamped asserted");
        check_cond(lim_pulse_us >= 999.0 && lim_pulse_us <= 1001.0,  "Limit low: 500->1000 us clamped");
        check_cond(sc_safe_j0 == 12'd1050 && sc_safe_j1 == 12'd1950, "Safety clamp: hip 1050 / knee 1950");

        // TEST 3: STAND & High Limit Clamp & prev_state check
        $display("[TEST 3] Verifying STAND, prev_state & high PWM clamp...");
        lim_test_pulse_us = 12'd2500;
        command = CMD_STAND;
        wait_frames(15);
        check_cond(state == STATE_STAND && prev_state == STATE_IDLE, "STAND: state=STAND, prev=IDLE");
        check_cond(j0 == 12'd1350 && j1 == 12'd1800,                 "STAND: settled 1350/1800 us");
        check_cond(ch0_pulse_us >= 1349.0 && ch0_pulse_us <= 1351.0, "STAND: CH0 PWM == 1350 us");
        check_cond(ch1_pulse_us >= 1799.0 && ch1_pulse_us <= 1801.0, "STAND: CH1 PWM == 1800 us");
        check_cond(lim_pulse_clamped == 1'b1 && lim_pulse_us >= 1999.0 && lim_pulse_us <= 2001.0,
                   "Limit high: 2500->2000 us clamped");

        // TEST 4: WALK
        $display("[TEST 4] Verifying WALK (8-phase lift-first gait)...");
        lim_test_pulse_us = 12'd1500;
        command = CMD_WALK;
        wait_frames(48);
        check_cond(state == STATE_WALK && prev_state == STATE_STAND, "WALK: state=WALK, prev=STAND");
        check_cond(phases_seen == 8'hFF,                             "WALK: all 8 phases (0..7) visited");
        check_cond(pwm_clamped == 8'b0,                              "WALK: zero PWM clamp violations");

        // TEST 4B: TROT (distinct brisker gait)
        $display("[TEST 4B] Verifying TROT gait...");
        command = CMD_TROT;
        wait_frames(24);
        check_cond(state == STATE_TROT && prev_state == STATE_WALK,  "TROT: state=TROT, prev=WALK");

        // TEST 5: STOP
        $display("[TEST 5] Verifying STOP during locomotion...");
        command = CMD_STOP;
        wait_frames(12);
        check_cond(state == STATE_STOP && gait_phase == 3'd0,        "STOP: halted & phase=0");
        check_cond(j0 == 12'd1350 && j1 == 12'd1800,                 "STOP: settled safe STAND pose");

        // TEST 5B: WAVE, DANCE, SIT
        $display("[TEST 5B] Verifying WAVE, DANCE, and SIT commands...");
        command = CMD_WAVE;
        wait_frames(24);
        check_cond(state == STATE_WAVE && j2 <= 12'd1150,            "WAVE: high-paw lifted");

        command = CMD_DANCE;
        wait_frames(24);
        check_cond(state == STATE_DANCE,                             "DANCE: active");

        command = CMD_SIT;
        wait_frames(15);
        check_cond(state == STATE_SIT && j5 >= 12'd1900,             "SIT: rear crouched sitting pose");

        // TEST 5C: DIRECTIONAL LOCOMOTION STATES (BACKWARD, STRAFE L/R, TURN L/R)
        $display("[TEST 5C] Verifying WALK_BACKWARD, STRAFE_LEFT/RIGHT, TURN_LEFT/RIGHT...");
        command = CMD_WALK_BACKWARD;
        wait_frames(12);
        check_cond(state == STATE_WALK_BACKWARD && pwm_clamped == 8'b0, "WALK_BACKWARD: active & unclamped");

        command = CMD_STRAFE_LEFT;
        wait_frames(12);
        check_cond(state == STATE_STRAFE_LEFT && pwm_clamped == 8'b0,   "STRAFE_LEFT: active & unclamped");

        command = CMD_STRAFE_RIGHT;
        wait_frames(12);
        check_cond(state == STATE_STRAFE_RIGHT && pwm_clamped == 8'b0,  "STRAFE_RIGHT: active & unclamped");

        command = CMD_TURN_LEFT;
        wait_frames(12);
        check_cond(state == STATE_TURN_LEFT && pwm_clamped == 8'b0,     "TURN_LEFT: active & unclamped");

        command = CMD_TURN_RIGHT;
        wait_frames(12);
        check_cond(state == STATE_TURN_RIGHT && pwm_clamped == 8'b0,    "TURN_RIGHT: active & unclamped");

        command = CMD_STOP;
        wait_frames(10);

        // TEST 6: EMERGENCY STOP + LATCH + RECOVERY
        $display("[TEST 6] Verifying Emergency Stop -> FAULT & recovery...");
        command = CMD_WALK;
        wait_frames(6);
        #5000000;
        @(negedge clk);
        estop = 1'b1;
        @(posedge clk); #1;
        check_cond(fault_active == 1'b1 && fault_code[0] == 1'b1,    "ESTOP: fault latched in 1 clock");
        check_cond(state == STATE_FAULT && j0 == 12'd1500,           "ESTOP: FAULT & 1500 us override");

        wait_frames(4);
        estop = 1'b0;
        wait_frames(4);
        check_cond(fault_active == 1'b1 && state == STATE_FAULT,     "ESTOP latch persists while cmd=WALK");

        command = CMD_IDLE;
        @(negedge clk); fault_clear = 1'b1; @(posedge clk); @(negedge clk); fault_clear = 1'b0;
        wait_frames(3);
        check_cond(fault_active == 1'b0 && state == STATE_IDLE,      "ESTOP recovery -> IDLE");

        // TEST 7A: INVALID COMMAND (cmd = 15 > 13)
        $display("[TEST 7A] Verifying invalid command trap (cmd = 4'd15)...");
        command = 4'd15; cmd_valid = 1'b1;
        wait_frames(4);
        check_cond(fault_active == 1'b1 && fault_code[1] == 1'b1 && state == STATE_FAULT,
                   "Invalid cmd 15 -> FAULT");

        command = CMD_IDLE;
        @(negedge clk); fault_clear = 1'b1; @(posedge clk); @(negedge clk); fault_clear = 1'b0;
        wait_frames(2);
        check_cond(fault_active == 1'b0 && state == STATE_IDLE,      "Invalid cmd recovery -> IDLE");

        // TEST 7B: WATCHDOG TIMEOUT
        $display("[TEST 7B] Verifying communication watchdog timeout...");
        watchdog_enable = 1'b1;
        command = CMD_STAND; cmd_valid = 1'b1;
        wait_frames(2);
        cmd_valid = 1'b0;
        wait_frames(TIMEOUT_FRAMES + 2);
        check_cond(fault_active == 1'b1 && fault_code[2] == 1'b1 && state == STATE_FAULT,
                   "Watchdog timeout -> FAULT");

        // PWM SUMMARY CHECKS
        check_cond(period_checks_ok >= 210, "PWM Period: >=210 consecutive 20.000 ms frames verified");
        check_cond(pulse_checks_ok  >= 420, "PWM Pulse Width: >=420 exact CH0/CH1 pulses verified");

        #5000;
        $fclose(csv_fd);

        $display("======================================================================");
        $display(" VERIFICATION SUMMARY");
        $display("   Total CSV frames exported : %0d (%0.2f s simulated)", frame_num, frame_num * 0.02);
        $display("   PWM period checks passed  : %0d (last = %0.2f us)", period_checks_ok, ch0_period_us);
        $display("   PWM pulse checks passed   : %0d (last CH0=%0.2f us, CH1=%0.2f us)", pulse_checks_ok, ch0_pulse_us, ch1_pulse_us);
        $display("   Assertions PASSED         : %0d", pass_count);
        $display("   Assertions FAILED         : %0d", fail_count);
        $display("======================================================================");
        if (fail_count == 0)
            $display(" RESULT: ALL VERILOG TESTS PASSED!");
        else
            $display(" RESULT: VERILOG TESTS FAILED (%0d failures)!", fail_count);
        $finish;
    end

endmodule
