module pid_controller #(
    parameter signed [15:0] KP = 16'd12, 
    parameter signed [15:0] KI = 16'd0,
    parameter signed [15:0] KD = 16'd4,
    parameter SHIFT = 0  // Number of bit shifts to divide the result
) (
    input clk,
    input rst,
    input enable, // Driven by a timer tick (e.g., 200 Hz control loop)
    input signed [7:0] error,
    output signed [15:0] control_out
);
    reg signed [7:0] prev_error;
    reg signed [15:0] integral;
    
    // Parallel combinational multiplications (inferred as DSP48 multipliers)
    wire signed [15:0] p_term = error * KP;
    wire signed [15:0] d_term = (error - prev_error) * KD;
    wire signed [15:0] i_term = integral * KI;
    
    // Sum and shift right to restore scaling (fixed point representation)
    wire signed [15:0] total = (p_term + i_term + d_term) >>> SHIFT;
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            prev_error <= 0;
            integral <= 0;
        end else if (enable) begin
            prev_error <= error;
            integral <= integral + error; // Simple integrator
        end
    end
    
    assign control_out = total;
endmodule
