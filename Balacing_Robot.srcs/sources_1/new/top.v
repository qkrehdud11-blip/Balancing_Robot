`timescale 1ns / 1ps
//================================================================================
// top
// - Basys3 보드용 최상위 모듈
// - MPU6050 + I2C master + UART debug
// - LED는 현재 디버깅용으로 재배치
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

    //--------------------------------------------------------------------------
    // 공통 tick
    //--------------------------------------------------------------------------
    wire         tick;

    //--------------------------------------------------------------------------
    // mpu6050_ctrl <-> i2c_master 제어 신호
    //--------------------------------------------------------------------------
    wire         start_req;
    wire         rw;
    wire [7:0]   reg_addr;
    wire [7:0]   tx_data;
    wire [7:0]   burst_len;

    //--------------------------------------------------------------------------
    // i2c_master 상태 신호
    //--------------------------------------------------------------------------
    wire         busy;
    wire         done;
    wire         ack_ok;
    wire [2:0]   err_code;
    wire [7:0]   rx_data;
    wire [127:0] rx_buf;
    wire [7:0]   rx_count;

    //--------------------------------------------------------------------------
    // open-drain 제어
    //--------------------------------------------------------------------------
    wire         sda_enable;
    wire         scl_enable;

    //--------------------------------------------------------------------------
    // MPU raw 데이터
    //--------------------------------------------------------------------------
    wire signed [15:0] accel_x;
    wire signed [15:0] accel_y;
    wire signed [15:0] accel_z;
    wire signed [15:0] gyro_x;
    wire signed [15:0] gyro_y;
    wire signed [15:0] gyro_z;

    //--------------------------------------------------------------------------
    // 상위 상태
    //--------------------------------------------------------------------------
    wire               init_done;
    wire               data_valid;
    wire [2:0]         last_err;

    //--------------------------------------------------------------------------
    // UART 출력선
    //--------------------------------------------------------------------------
    wire               uart_tx_wire;

    //==========================================================================
    // tick 생성
    //==========================================================================
    clk_divider u_clk_divider
    (
        .clk   (clk),
        .rst_n (rst_n),
        .tick  (tick)
    );

    //==========================================================================
    // MPU6050 상위 제어
    //==========================================================================
    mpu6050_ctrl u_mpu6050_ctrl
    (
        .clk        (clk),
        .rst_n      (rst_n),
        .tick       (tick),

        .busy       (busy),
        .done       (done),
        .ack_ok     (ack_ok),
        .err_code   (err_code),
        .rx_buf     (rx_buf),
        .rx_count   (rx_count),

        .start_req  (start_req),
        .rw         (rw),
        .reg_addr   (reg_addr),
        .tx_data    (tx_data),
        .burst_len  (burst_len),

        .init_done  (init_done),
        .data_valid (data_valid),
        .last_err   (last_err),

        .ax         (accel_x),
        .ay         (accel_y),
        .az         (accel_z),
        .gx         (gyro_x),
        .gy         (gyro_y),
        .gz         (gyro_z)
    );

    //==========================================================================
    // I2C master
    //==========================================================================
    i2c_master u_i2c_master
    (
        .clk        (clk),
        .rst_n      (rst_n),
        .tick       (tick),

        .start_req  (start_req),
        .rw         (rw),
        .reg_addr   (reg_addr),
        .tx_data    (tx_data),
        .burst_len  (burst_len),
        .sda_in     (i2c_sda),

        .busy       (busy),
        .done       (done),
        .ack_ok     (ack_ok),
        .err_code   (err_code),
        .rx_data    (rx_data),
        .rx_buf     (rx_buf),
        .rx_count   (rx_count),

        .sda_enable (sda_enable),
        .scl_enable (scl_enable)
    );

    //==========================================================================
    // UART debug
    //==========================================================================
    mpu6050_debug_uart u_debug_uart
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

    //==========================================================================
    // open-drain I2C
    //==========================================================================
    assign i2c_sda = (sda_enable) ? 1'b0 : 1'bz;
    assign i2c_scl = (scl_enable) ? 1'b0 : 1'bz;

    //==========================================================================
    // UART 출력
    //==========================================================================
    assign uart_tx = uart_tx_wire;

    //==========================================================================
    // LED 디버그
    // led[0] : rst_n
    // led[1] : init_done
    // led[2] : busy
    // led[3] : ack_ok
    // led[4] : done
    // led[5] : data_valid
    // led[6] : last_err[0]
    // led[7] : last_err[1]
    // led[8] : last_err[2]
    // led[9] : uart_tx_wire
    //==========================================================================
    assign led[0] = rst_n;
    assign led[1] = init_done;
    assign led[2] = busy;
    assign led[3] = ack_ok;
    assign led[4] = done;
    assign led[5] = data_valid;
    assign led[6] = last_err[0];
    assign led[7] = last_err[1];
    assign led[8] = last_err[2];
    assign led[9] = uart_tx_wire;

endmodule