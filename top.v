module top (
    input clk,  // 27MHz from Tang Nano 9K (Pin 52)
    
    // Sensors: 1 = White, 0 = Black
    input s1,
    input s2,
    input s3,
    input s4,
    input s5,
    
    // Encoder (Placeholder)
    input enc_l_c1,
    input enc_l_c2,
    input enc_r_c1,
    input enc_r_c2,
    
    output reg pwma,
    output reg ain1,
    output reg ain2,
    
    output reg pwmb,
    output reg bin1,
    output reg bin2
);

    // =====================================================
    // Tuning Parameters
    // =====================================================
    localparam MAX_COUNT    = 2700;          
    localparam BASE_SPEED   = 2200;          
    localparam SEARCH_SPEED  = 1800;         

    localparam signed [15:0] KP = 2;         
    localparam signed [15:0] KD = 30;        

    localparam CURVE_SLOWDOWN = 1;           
    localparam CTRL_DIV       = 13500;       // 2kHz Loop

    // =====================================================
    // Internal Signals
    // =====================================================
    wire [4:0] sensor_bus = {s1, s2, s3, s4, s5};

    reg [15:0] ctrl_cnt = 0;
    wire ctrl_tick = (ctrl_cnt == CTRL_DIV - 1);

    // Temporal Filtering (Debounce)
    reg [4:0] sensor_hist   = 5'b11111;
    reg [1:0] stable_cnt    = 0;
    reg [4:0] stable_sensor = 5'b11111;

    reg signed [15:0] error           = 0;
    reg signed [15:0] smoothed_error   = 0;
    reg signed [15:0] last_smoothed    = 0;
    reg signed [15:0] pd_output        = 0;

    reg [11:0] duty_l = 0;
    reg [11:0] duty_r = 0;
    reg [1:0] last_side = 0;
    reg line_detected = 0;
    reg [11:0] pwm_cnt = 0;

    reg signed [16:0] mix_l;
    reg signed [16:0] mix_r;
    reg [15:0] abs_error;
    reg [15:0] active_speed;

    // Auto-Trim Logic
    reg [15:0] trim_timer = 0;
    reg [11:0] enc_l_cnt = 0;
    reg [11:0] enc_r_cnt = 0;
    reg signed [15:0] auto_trim = 0;
    reg [1:0] enc_l_sync, enc_r_sync;

    // =====================================================
    // 1. Sensor Filtering & Error Mapping (2kHz)
    // =====================================================
    always @(posedge clk) begin
        ctrl_cnt <= (ctrl_tick) ? 0 : ctrl_cnt + 1;

        if (ctrl_tick) begin
            // Temporal Filter: Must be stable for 3 samples
            if (sensor_bus == sensor_hist) begin
                if (stable_cnt < 2) stable_cnt <= stable_cnt + 1;
                else stable_sensor <= sensor_bus;
            end else begin
                stable_cnt <= 0;
                sensor_hist <= sensor_bus;
            end

            line_detected <= 1'b1;
            case (stable_sensor)
                // Center Deadband (3-sensor wide quiet zone)
                5'b11011, 5'b10011, 5'b11001: begin
                    error <= 0;
                    last_side <= 0;
                end
                
                // S-Curve Exponential Mapping
                5'b10111: begin error <= -100;  last_side <= 1; end  // Inner
                5'b11101: begin error <= 100;   last_side <= 2; end  
                
                5'b00111: begin error <= -600;  last_side <= 1; end  // Mid
                5'b11100: begin error <= 600;   last_side <= 2; end  
                
                5'b01111: begin error <= -2500; last_side <= 1; end  // Edge (Pivot)
                5'b11110: begin error <= 2500;  last_side <= 2; end  

                5'b00000: begin error <= 0; end
                5'b11111: begin line_detected <= 1'b0; end
                default:  begin end
            endcase

            // Error Smoothing & PD Calculation
            smoothed_error <= (error + smoothed_error * 3) >>> 2; 
            pd_output <= (KP * smoothed_error) + (KD * (smoothed_error - last_smoothed));
            last_smoothed <= smoothed_error;
        end
    end

    // =====================================================
    // 2. Auto-Trim Integration
    // =====================================================
    always @(posedge clk) begin
        enc_l_sync <= {enc_l_sync[0], enc_l_c1};
        enc_r_sync <= {enc_r_sync[0], enc_r_c1};
        if (enc_l_sync == 2'b01) enc_l_cnt <= enc_l_cnt + 1;
        if (enc_r_sync == 2'b01) enc_r_cnt <= enc_r_cnt + 1;

        if (ctrl_tick) begin
            trim_timer <= trim_timer + 1;
            if (trim_timer >= 20) begin 
                trim_timer <= 0;
                if (error == 0 && line_detected) begin
                    if (enc_l_cnt > enc_r_cnt + 2) auto_trim <= auto_trim + 2;
                    else if (enc_r_cnt > enc_l_cnt + 2) auto_trim <= auto_trim - 2;
                end
                enc_l_cnt <= 0;
                enc_r_cnt <= 0;
            end
        end
    end

    // =====================================================
    // 3. Advanced Motor Mixing (Inertia Control)
    // =====================================================
    always @(posedge clk) begin
        if (ctrl_tick) begin
            if (!line_detected) begin
                if (last_side == 1) begin 
                    duty_l <= 0; duty_r <= SEARCH_SPEED; 
                    ain1 <= 1'b0; ain2 <= 1'b1; bin1 <= 1'b0; bin2 <= 1'b1;
                end else if (last_side == 2) begin
                    duty_l <= SEARCH_SPEED; duty_r <= 0; 
                    ain1 <= 1'b0; ain2 <= 1'b1; bin1 <= 1'b0; bin2 <= 1'b1;
                end else begin 
                    duty_l <= 0; duty_r <= 0; 
                    ain1 <= 1'b0; ain2 <= 1'b0; bin1 <= 1'b0; bin2 <= 1'b0; 
                end
            end else begin
                abs_error = (error < 0) ? -error : error;
                
                // Pre-emptive Braking for Sharp Curves
                if (abs_error > 600) begin
                    active_speed = 800; // Hard brake for edges
                end else begin
                    active_speed = BASE_SPEED; // Full speed for straights/inner
                end

                mix_l = $signed({1'b0, active_speed}) + pd_output - auto_trim;
                mix_r = $signed({1'b0, active_speed}) - pd_output + auto_trim;

                // Motor Mapping (A=Right, B=Left)
                if (mix_l >= 0) begin
                    duty_l <= (mix_l > MAX_COUNT) ? MAX_COUNT : mix_l[11:0];
                    bin1 <= 1'b0; bin2 <= 1'b1;
                end else begin
                    duty_l <= (-mix_l > MAX_COUNT) ? MAX_COUNT : -mix_l[11:0];
                    bin1 <= 1'b1; bin2 <= 1'b0;
                end

                if (mix_r >= 0) begin
                    duty_r <= (mix_r > MAX_COUNT) ? MAX_COUNT : mix_r[11:0];
                    ain1 <= 1'b0; ain2 <= 1'b1;
                end else begin
                    duty_r <= (-mix_r > MAX_COUNT) ? MAX_COUNT : -mix_r[11:0];
                    ain1 <= 1'b1; ain2 <= 1'b0;
                end
            end
        end
    end

    // =====================================================
    // 4. PWM Generation
    // =====================================================
    always @(posedge clk) begin
        pwm_cnt <= (pwm_cnt >= MAX_COUNT-1) ? 0 : pwm_cnt + 1;
        pwmb <= (pwm_cnt < duty_l);
        pwma <= (pwm_cnt < duty_r);
    end

endmodule
