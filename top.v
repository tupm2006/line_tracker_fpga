module top (
    input clk,  // 27MHz from Tang Nano 9K (Pin 52)
    
    // Front Sensors (8-mắt): 1 = White, 0 = Black
    input f1, f2, f3, f4, f5, f6, f7, f8,
    
    // Back Sensors (5-mắt KD): 1 = White, 0 = Black
    input b1, b2, b3, b4, b5,
    
    // Encoder
    input enc_l_c1, enc_l_c2,
    input enc_r_c1, enc_r_c2,
    
    output reg pwma, ain1, ain2,
    output reg pwmb, bin1, bin2
);

    // =====================================================
    // Tuning Parameters
    // =====================================================
    localparam [11:0] MAX_COUNT    = 2700;          
    localparam [11:0] BASE_SPEED   = 2200;          
    localparam [11:0] SEARCH_SPEED = 2400;         

    localparam signed [15:0] KP = 1;         
    localparam signed [15:0] KD = 400;       // Physical KD needs higher scaling

    localparam CTRL_DIV       = 13500;       // 2kHz Loop

    // =====================================================
    // Internal Signals
    // =====================================================
    wire [7:0] front_bus = {f1, f2, f3, f4, f5, f6, f7, f8};
    wire [4:0] back_bus  = {b1, b2, b3, b4, b5};

    reg [15:0] ctrl_cnt = 0;
    wire ctrl_tick = (ctrl_cnt == CTRL_DIV - 1);

    // Temporal Filtering (Debounce)
    reg [7:0] front_hist   = 8'hFF;
    reg [1:0] stable_f_cnt = 0;
    reg [7:0] stable_front = 8'hFF;

    reg [4:0] back_hist   = 5'b11111;
    reg [1:0] stable_b_cnt = 0;
    reg [4:0] stable_back = 5'b11111;

    reg signed [15:0] error_front     = 0;
    reg signed [15:0] error_back      = 0;
    
    reg signed [15:0] smoothed_front  = 0;
    reg signed [15:0] smoothed_back   = 0;

    reg signed [31:0] pd_output       = 0;

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
            // Temporal Filter for Front
            if (front_bus == front_hist) begin
                if (stable_f_cnt < 2) stable_f_cnt <= stable_f_cnt + 1;
                else stable_front <= front_bus;
            end else begin
                stable_f_cnt <= 0;
                front_hist <= front_bus;
            end

            // Temporal Filter for Back
            if (back_bus == back_hist) begin
                if (stable_b_cnt < 2) stable_b_cnt <= stable_b_cnt + 1;
                else stable_back <= back_bus;
            end else begin
                stable_b_cnt <= 0;
                back_hist <= back_bus;
            end

            line_detected <= 1'b1;
            
            // Front Mapping (8-Sensor)
            case (stable_front)
                8'b11100111: begin error_front <= 0;     last_side <= 0; end
                
                8'b11101111: begin error_front <= -25;   last_side <= 1; end
                8'b11110111: begin error_front <= 25;    last_side <= 2; end
                
                8'b11001111: begin error_front <= -100;  last_side <= 1; end
                8'b11110011: begin error_front <= 100;   last_side <= 2; end
                
                8'b11011111: begin error_front <= -300;  last_side <= 1; end
                8'b11111011: begin error_front <= 300;   last_side <= 2; end
                
                8'b10011111: begin error_front <= -800;  last_side <= 1; end
                8'b11111001: begin error_front <= 800;   last_side <= 2; end
                
                8'b10111111: begin error_front <= -1600; last_side <= 1; end
                8'b11111101: begin error_front <= 1600;  last_side <= 2; end
                
                8'b00111111: begin error_front <= -2800; last_side <= 1; end
                8'b11111100: begin error_front <= 2800;  last_side <= 2; end
                
                8'b01111111: begin error_front <= -2800; last_side <= 1; end
                8'b11111110: begin error_front <= 2800;  last_side <= 2; end
                
                8'b00000000: begin error_front <= 0; end
                8'b11111111: begin line_detected <= 1'b0; end
                default: begin end
            endcase

            // Back Mapping (5-Sensor)
            case (stable_back)
                5'b11011: begin error_back <= 0; end
                5'b10011: begin error_back <= -100; end
                5'b11001: begin error_back <= 100; end
                5'b10111: begin error_back <= -300; end
                5'b11101: begin error_back <= 300; end
                5'b00111: begin error_back <= -800; end
                5'b11100: begin error_back <= 800; end
                5'b01111: begin error_back <= -2800; end
                5'b11110: begin error_back <= 2800; end
                default: begin end
            endcase

            // Lower Smoothing (75% memory) - Dual-Bar needs less temporal filter, avoiding phase lag
            smoothed_front <= (error_front + smoothed_front * 3) >>> 2; 
            smoothed_back  <= (error_back + smoothed_back * 3) >>> 2;
            
            // Dual-Bar PD Calculation
            pd_output <= ($signed({1'b0, KP}) * smoothed_front) + ($signed({1'b0, KD}) * (smoothed_front - smoothed_back));
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
            if (trim_timer >= 100) begin // 50ms interval (2kHz / 100 = 20Hz = 50ms)
                trim_timer <= 0;
                // Balance memory enabled for mostly-straight states
                if (line_detected && abs_error < 1000) begin
                    // Deadband 5 counts
                    if (enc_l_cnt > enc_r_cnt + 5) auto_trim <= auto_trim + 1;
                    else if (enc_r_cnt > enc_l_cnt + 5) auto_trim <= auto_trim - 1;
                end
                enc_l_cnt <= 0;
                enc_r_cnt <= 0;
            end
        end
    end

    // =====================================================
    // 3. Motor Mixing & Arc Recovery
    // =====================================================
    always @(posedge clk) begin
        if (ctrl_tick) begin
            if (!line_detected) begin
                // Use momentum to "sling" back onto the line (Arc Recovery)
                if (last_side == 1) begin 
                    // Tighter Pivot: Push right forward, Spin left backward
                    duty_l <= 800; duty_r <= SEARCH_SPEED; 
                    ain1 <= 1'b0; ain2 <= 1'b1; bin1 <= 1'b1; bin2 <= 1'b0;
                end else if (last_side == 2) begin
                    // Tighter Pivot: Push left forward, Spin right backward
                    duty_l <= SEARCH_SPEED; duty_r <= 800; 
                    ain1 <= 1'b1; ain2 <= 1'b0; bin1 <= 1'b0; bin2 <= 1'b1;
                end else begin 
                    // No history: Stop safely
                    duty_l <= 0; duty_r <= 0;
                    ain1 <= 1'b0; ain2 <= 1'b0; bin1 <= 1'b0; bin2 <= 1'b0;
                end
            end else begin
                abs_error = (smoothed_front < 0) ? -smoothed_front : smoothed_front;
                
                // Pre-emptive Braking for Sharp Curves
                if (abs_error > 50) begin
                    active_speed = 700; // Hard brake to handle sharp corners
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
