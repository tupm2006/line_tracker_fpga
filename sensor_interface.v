module sensor_interface (
    input clk,
    input rst,
    input [7:0] ir_sensors,
    output reg signed [7:0] error,
    output reg valid
);
    
    // Filtered internal state of sensors
    reg [7:0] clean_sensors;
    reg [13:0] filter_cnt [7:0]; // 14-bit counter per pin (~0.6ms at 27MHz)
    
    integer i;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            for (i=0; i<8; i=i+1) begin
                filter_cnt[i] <= 14'd0;
                clean_sensors[i] <= 1'b0;
            end
        end else begin
            for (i=0; i<8; i=i+1) begin
                // Saturating counters
                if (ir_sensors[i]) begin
                    if (filter_cnt[i] != 14'h3FFF) filter_cnt[i] <= filter_cnt[i] + 1;
                end else begin
                    if (filter_cnt[i] != 14'h0000) filter_cnt[i] <= filter_cnt[i] - 1;
                end
                
                // Digital Schmitt-trigger hysteresis thresholds
                if (filter_cnt[i] == 14'h3FFF)
                    clean_sensors[i] <= 1'b1;
                else if (filter_cnt[i] == 14'h0000)
                    clean_sensors[i] <= 1'b0;
            end
        end
    end
    
    // Calculates position error based on an 8-channel Array
    // D1 (clean_sensors[7]) is Far Left, D8 (clean_sensors[0]) is Far Right.
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            error <= 8'd0;
            valid <= 1'b0;
        end else begin
            valid <= 1'b1;
            
            // Priority check: Prioritize the outermost sensors to aggressively 
            // recover from sharp turns. If a sensor is triggered, it assigns 
            // an error magnitude. (Left = Negative, Right = Positive)
            if (clean_sensors[7])      error <= -8'd120;  // D1 (Far Left, HUGE steering correction)
            else if (clean_sensors[0]) error <= 8'd120;   // D8 (Far Right)
            else if (clean_sensors[6]) error <= -8'd80;   // D2
            else if (clean_sensors[1]) error <= 8'd80;    // D7
            else if (clean_sensors[5]) error <= -8'd45;   // D3
            else if (clean_sensors[2]) error <= 8'd45;    // D6
            else if (clean_sensors[4] && clean_sensors[3]) error <= 8'd0; // Perfectly Centered
            else if (clean_sensors[4]) error <= -8'd20;   // D4 (Slightly left)
            else if (clean_sensors[3]) error <= 8'd20;    // D5 (Slightly right)
            else if (clean_sensors == 8'b00000000) begin
                valid <= 1'b0; // Line lost!
                // We keep the LAST KNOWN error to steer back into the line's direction.
            end
        end
    end
endmodule
