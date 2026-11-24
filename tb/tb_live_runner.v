// =============================================================================
// Testbench   : tb_live_runner
// Project     : XEROTON Studio 3.0 - Live Interactive Verilog RTL Engine
// Description : Fast parameterized runner for xeroton_top.v invoked on demand
//               by the Python backend with +CMD=<0..15>, +ESTOP=<0|1>,
//               +STRIDE=<us>, +PHASE_FRAMES=<n>, +FRAMES=<N>, +OUT=<csv>.
//               For cyclic locomotion gaits, primes through STAND + 1 full
//               warm-up gait cycle before logging so exported cycles wrap
//               seamlessly with ZERO phase discontinuity or reversal kick.
// =============================================================================
`timescale 1ns/1ps

module tb_live_runner;

    localparam integer CLK_HZ           = 105000;
    localparam integer PWM_HZ           = 50;
    localparam integer FRAMES_PER_PHASE = 4;

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

    reg clk = 1'b0;
    always #500 clk = ~clk;

    reg        reset           = 1'b1;
    reg [3:0]  command         = 4'd1;
    reg        cmd_valid       = 1'b1;
    reg        estop           = 1'b0;
    reg        fault_clear     = 1'b0;
    reg        watchdog_enable = 1'b0;
    reg [7:0]  stride_us       = 8'd0;
    reg [3:0]  phase_frames    = 4'd0;

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
        .TIMEOUT_FRAMES   (15),
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

    function [127:0] sname;
        input [3:0] s;
        begin
            case (s)
                STATE_IDLE:          sname = "IDLE";
                STATE_STAND:         sname = "STAND";
                STATE_WALK_FORWARD:  sname = "WALK_FORWARD";
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

    integer arg_cmd    = 1;
    integer arg_estop  = 0;
    integer arg_frames = 0;
    integer arg_stride = 0;
    integer arg_pfr    = 0;
    reg [1023:0] out_file = "simulation/live_targets.csv";

    integer csv_fd;
    integer frame_num  = 0;
    reg     logging_en = 1'b0;

    always @(posedge frame_tick) begin
        @(posedge clk);
        #1;
        if (!reset && logging_en) begin
            frame_num = frame_num + 1;
            $fdisplay(csv_fd,
                "%0d,%0.2f,%0d,%0d,%0d,%0s,%0s,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d",
                frame_num,
                frame_num * 20.0,
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

    integer k;
    integer eff_pf;
    integer is_cyclic;
    integer warmup_frames;
    integer log_frames;

    initial begin
        if ($value$plusargs("CMD=%d", arg_cmd)) ;
        if ($value$plusargs("ESTOP=%d", arg_estop)) ;
        if ($value$plusargs("FRAMES=%d", arg_frames)) ;
        if ($value$plusargs("STRIDE=%d", arg_stride)) ;
        if ($value$plusargs("PHASE_FRAMES=%d", arg_pfr)) ;
        if ($value$plusargs("OUT=%s", out_file)) ;

        stride_us    = arg_stride[7:0];
        phase_frames = arg_pfr[3:0];

        if (arg_cmd == 7)
            eff_pf = (arg_pfr == 0) ? 3 : ((arg_pfr < 2) ? 2 : ((arg_pfr > 10) ? 10 : arg_pfr));
        else
            eff_pf = (arg_pfr == 0) ? FRAMES_PER_PHASE : ((arg_pfr < 2) ? 2 : ((arg_pfr > 12) ? 12 : arg_pfr));

        is_cyclic = (!arg_estop && (arg_cmd == 2 || arg_cmd == 4 || arg_cmd == 5 || arg_cmd == 7 ||
                                    arg_cmd == 9 || arg_cmd == 10 || arg_cmd == 11 || arg_cmd == 12 || arg_cmd == 13));

        // If FRAMES==0, export exact 1 seamless cycle (8 * eff_pf) for cyclic gaits, or 16 frames for static poses
        if (arg_frames > 0)
            log_frames = arg_frames;
        else if (is_cyclic)
            log_frames = 8 * eff_pf;
        else
            log_frames = 16;

        csv_fd = $fopen(out_file, "w");
        if (csv_fd == 0) begin
            $display("[FATAL] Unable to open %0s", out_file);
            $finish;
        end
        $fdisplay(csv_fd,
            "frame,time_ms,command,state,prev_state,state_name,prev_state_name,gait_phase,fault_active,fault_code,j0_us,j1_us,j2_us,j3_us,j4_us,j5_us,j6_us,j7_us");

        reset   = 1'b1;
        command = arg_cmd[3:0];
        estop   = (arg_estop != 0);
        #5000;
        @(negedge clk);
        reset = 1'b0;

        // Prime through STAND + 1 full warm-up cycle for cyclic gaits so logged cycle wraps seamlessly
        if (is_cyclic) begin
            command = 4'd1;
            for (k = 0; k < 8; k = k + 1) @(negedge frame_tick);
            command = arg_cmd[3:0];
            warmup_frames = 8 * eff_pf;
            for (k = 0; k < warmup_frames; k = k + 1) @(negedge frame_tick);
        end else if (!estop && (arg_cmd == 6 || arg_cmd == 8)) begin
            command = 4'd1;
            for (k = 0; k < 8; k = k + 1) @(negedge frame_tick);
            command = arg_cmd[3:0];
        end

        @(posedge clk);
        #2;
        logging_en = 1'b1;
        for (k = 0; k < log_frames; k = k + 1) begin
            @(negedge frame_tick);
        end
        @(posedge clk);
        #2;
        logging_en = 1'b0;

        #2000;
        $fclose(csv_fd);
        $finish;
    end

endmodule
