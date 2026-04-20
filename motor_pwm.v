module motor_pwm #(
    parameter CLK_FREQ = 27000000,   // Tang Nano 9k Onboard Clock: 27 MHz
    parameter PWM_FREQ = 20000       // Target Frequency: 20 kHz
) (
    input clk,
    input rst,
    input [15:0] duty, // Control signal (0 to MAX_COUNT)
    output reg pwm
);
    localparam MAX_COUNT = CLK_FREQ / PWM_FREQ; // 27M / 20k = 1350
    reg [15:0] counter;
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            counter <= 0;
            pwm <= 0;
        end else begin
            if (counter >= MAX_COUNT - 1)
                counter <= 0;
            else
                counter <= counter + 1;
                
            pwm <= (counter < duty) ? 1'b1 : 1'b0;
        end
    end
endmodule
