`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/12/2026 08:59:24 PM
// Design Name: 
// Module Name: mpu6050
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module mpu6050(
    input  wire        clk,
    input  wire        rst,
    // I2C 인터페이스 (MPU6050 연결용)
    inout  wire        sda,
    output wire        scl,
    // 출력
    output wire [15:0] accel_x
);

endmodule
