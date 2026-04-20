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
    localparam MAX_COUNT   = 2700;          
    localparam BASE_SPEED  = 2400;          
    localparam SEARCH_SPEED = 2300;         

    localparam signed [15:0] LEFT_TRIM  = 150;
    localparam signed [15:0] RIGHT_TRIM = 0;

    localparam signed [15:0] KP = 35;
    localparam signed [15:0] KD = 80;

    localparam CURVE_SLOWDOWN = 180;

    localparam CTRL_DIV = 5400; // 5kHz Loop

    // =====================================================
    // Internal Signals
    // =====================================================
    wire [4:0] sensor_bus = {s1, s2, s3, s4, s5};

    reg [12:0] ctrl_cnt = 0;
    wire ctrl_tick = (ctrl_cnt == CTRL_DIV - 1);

    reg signed [15:0] error      = 0;
    reg signed [15:0] last_error = 0;
    reg signed [15:0] pd_output  = 0;
    reg [15:0] abs_error_tmp     = 0;

    reg [11:0] duty_l = 0;
    reg [11:0] duty_r = 0;
    reg [1:0] last_side = 0;
    reg line_detected = 0;
    reg [11:0] pwm_cnt = 0;

    reg signed [16:0] mix_l;
    reg signed [16:0] mix_r;
    reg [15:0] abs_error;
    reg [15:0] active_speed;

    // Auto-Trim Variables
    reg [1:0] enc_l_sync = 0;
    reg [1:0] enc_r_sync = 0;
    reg [11:0] enc_l_cnt = 0;
    reg [11:0] enc_r_cnt = 0;
    reg [19:0] trim_timer = 0;
    localparam TRIM_WINDOW = 540_000; // 20ms at 27MHz
    reg signed [15:0] auto_trim = 0;
    wire going_straight = (error == 0);

    // 0. Auto-Trim Logic
    always @(posedge clk) begin
        // Synchronize and edge detect
        enc_l_sync <= {enc_l_sync[0], enc_l_c1};
        enc_r_sync <= {enc_r_sync[0], enc_r_c1};
        
        if (trim_timer >= TRIM_WINDOW - 1) begin
            trim_timer <= 0;
            if (going_straight) begin
                if (enc_l_cnt > enc_r_cnt + 1) begin
                    // Left is faster -> slow it down by increasing auto_trim
                    if (auto_trim < 300) auto_trim <= auto_trim + 2;
                end else if (enc_r_cnt > enc_l_cnt + 1) begin
                    // Right is faster -> slow it down by decreasing auto_trim
                    if (auto_trim > -300) auto_trim <= auto_trim - 2;
                end
            end
            enc_l_cnt <= 0;
            enc_r_cnt <= 0;
        end else begin
            trim_timer <= trim_timer + 1;
            if (enc_l_sync == 2'b01 && enc_l_cnt < 4095) enc_l_cnt <= enc_l_cnt + 1;
            if (enc_r_sync == 2'b01 && enc_r_cnt < 4095) enc_r_cnt <= enc_r_cnt + 1;
        end
    end

    // 1. Control Loop Divider
    always @(posedge clk) begin
        if (ctrl_tick) ctrl_cnt <= 0;
        else ctrl_cnt <= ctrl_cnt + 1'b1;
    end

    // 2. PD Logic
    always @(posedge clk) begin
        if (ctrl_tick) begin
            line_detected <= 1'b1;
            case (sensor_bus)
                // Center Deadband
                5'b11011, 5'b10011, 5'b11001: begin
                    error <= 0;
                    last_side <= 0;
                end
                5'b10111: begin error <= -6;  last_side <= 1; end  
                5'b00111: begin error <= -15; last_side <= 1; end  
                5'b01111: begin error <= -40; last_side <= 1; end  
                5'b11101: begin error <= 6;   last_side <= 2; end  
                5'b11100: begin error <= 15;  last_side <= 2; end  
                5'b11110: begin error <= 40;  last_side <= 2; end  // far: boosted
                5'b00000: begin error <= 0; end
                5'b11111: begin line_detected <= 1'b0; end
                default: begin end
            endcase

            pd_output <= (KP * error) + (KD * (error - last_error));
            last_error <= error;
        end
    end

    // 3. Motor Mixing & Direction Control
    always @(posedge clk) begin
        if (ctrl_tick) begin
            if (!line_detected) begin
                // --- Search Mode ---
                if (last_side == 1) begin 
                    duty_l <= 0; duty_r <= SEARCH_SPEED; 
                    ain1 <= 1'b0; ain2 <= 1'b1; // Forward
                    bin1 <= 1'b0; bin2 <= 1'b1; // Forward
                end
                else if (last_side == 2) begin 
                    duty_l <= SEARCH_SPEED; duty_r <= 0; 
                    ain1 <= 1'b0; ain2 <= 1'b1;
                    bin1 <= 1'b0; bin2 <= 1'b1;
                end
                else begin 
                    duty_l <= 0; duty_r <= 0; 
                    ain1 <= 1'b0; ain2 <= 1'b0; // Stop
                    bin1 <= 1'b0; bin2 <= 1'b0; 
                end
            end else begin
                abs_error = (error < 0) ? -error : error;
                if (abs_error * CURVE_SLOWDOWN >= BASE_SPEED || (BASE_SPEED - abs_error * CURVE_SLOWDOWN) < 500)
                    active_speed = 500;
                else
                    active_speed = BASE_SPEED - (abs_error * CURVE_SLOWDOWN);

                // Calculate mixed speeds (Signed)
                mix_l = $signed({1'b0, active_speed}) + pd_output - auto_trim;
                mix_r = $signed({1'b0, active_speed}) - pd_output + auto_trim;

                // --- Motor L (bin) Control ---
                if (mix_l >= 0) begin
                    bin1 <= 1'b0; bin2 <= 1'b1; // Forward
                    duty_l <= (mix_l > MAX_COUNT) ? MAX_COUNT : mix_l[11:0];
                end else begin
                    bin1 <= 1'b1; bin2 <= 1'b0; // Reverse!
                    duty_l <= (-mix_l > MAX_COUNT) ? MAX_COUNT : -mix_l[11:0]; 
                end

                // --- Motor R (ain) Control ---
                if (mix_r >= 0) begin
                    ain1 <= 1'b0; ain2 <= 1'b1; // Forward
                    duty_r <= (mix_r > MAX_COUNT) ? MAX_COUNT : mix_r[11:0];
                end else begin
                    ain1 <= 1'b1; ain2 <= 1'b0; // Reverse!
                    duty_r <= (-mix_r > MAX_COUNT) ? MAX_COUNT : -mix_r[11:0]; 
                end
            end
        end
    end

    // 4. PWM Generator
    always @(posedge clk) begin
        if (pwm_cnt < (MAX_COUNT - 1)) pwm_cnt <= pwm_cnt + 1'b1;
        else pwm_cnt <= 0;

        pwma <= (pwm_cnt < duty_r); 
        pwmb <= (pwm_cnt < duty_l); 
    end

endmodule
