`timescale 1ns / 1ps
//================================================================================
// top
// - Basys3 보드용 최상위 모듈
// - MPU6050 + UART 디버그
//================================================================================

module top
(
    input  wire       clk,
    input  wire       rst_n,

    inout  wire       i2c_sda,
    inout  wire       i2c_scl,

    output wire       uart_tx,
    output wire [9:0] led
);

wire tick;

wire        start_req;
wire        rw;
wire [7:0]  reg_addr;
wire [7:0]  tx_data;
wire        busy;
wire        done;
wire        ack_ok;
wire [7:0]  rx_data;

wire sda_enable;
wire scl_enable;

wire signed [15:0] accel_x;
wire signed [15:0] accel_y;
wire signed [15:0] accel_z;

wire signed [15:0] gyro_x;
wire signed [15:0] gyro_y;
wire signed [15:0] gyro_z;

wire               init_done;
wire               data_valid;

wire uart_tx_wire;

//================================================================================
// clk divider
//================================================================================
clk_divider u_clk_divider
(
    .clk   (clk),
    .rst_n (rst_n),
    .tick  (tick)
);

//================================================================================
// MPU6050 control
//================================================================================
mpu6050_ctrl u_mpu6050_ctrl
(
    .clk        (clk),
    .rst_n      (rst_n),
    .tick       (tick),

    .busy       (busy),
    .done       (done),
    .ack_ok     (ack_ok),
    .rx_data    (rx_data),

    .start_req  (start_req),
    .rw         (rw),
    .reg_addr   (reg_addr),
    .tx_data    (tx_data),

    .accel_x    (accel_x),
    .accel_y    (accel_y),
    .accel_z    (accel_z),

    .gyro_x     (gyro_x),
    .gyro_y     (gyro_y),
    .gyro_z     (gyro_z),

    .init_done  (init_done),
    .data_valid (data_valid)
);

//================================================================================
// I2C master
//================================================================================
i2c_master u_i2c_master
(
    .clk        (clk),
    .rst_n      (rst_n),
    .tick       (tick),

    .start_req  (start_req),
    .rw         (rw),
    .reg_addr   (reg_addr),
    .tx_data    (tx_data),
    .sda_in     (i2c_sda),

    .busy       (busy),
    .done       (done),
    .ack_ok     (ack_ok),
    .rx_data    (rx_data),

    .sda_enable (sda_enable),
    .scl_enable (scl_enable)
);

//================================================================================
// UART debug
//================================================================================
mpu6050_debug_uart u_mpu6050_debug_uart
(
    .clk        (clk),
    .rst_n      (rst_n),
    .init_done  (init_done),
    .data_valid (data_valid),

    .accel_x    (accel_x),
    .accel_y    (accel_y),
    .accel_z    (accel_z),

    .gyro_x     (gyro_x),
    .gyro_y     (gyro_y),
    .gyro_z     (gyro_z),

    .uart_tx_o  (uart_tx_wire)
);

//================================================================================
// open-drain I2C
//================================================================================
assign i2c_sda = (sda_enable) ? 1'b0 : 1'bz;
assign i2c_scl = (scl_enable) ? 1'b0 : 1'bz;

//================================================================================
// UART output
//================================================================================
assign uart_tx = uart_tx_wire;

//================================================================================
// LED debug
// led[7:0] : accel_x 상위 바이트
// led[8]   : init_done
// led[9]   : uart 상태
//================================================================================
assign led[7:0] = accel_x[15:8];
assign led[8]   = init_done;
assign led[9]   = uart_tx_wire;

endmodule