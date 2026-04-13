`timescale 1ns / 1ps
//================================================================================
// mpu6050_ctrl
// - MPU6050 초기화 + 가속도 3축 + 자이로 3축 읽기 상위 FSM
//
// 초기화
//   PWR_MGMT_1   = 0x00
//   ACCEL_CONFIG = 0x00  (±2g)
//   GYRO_CONFIG  = 0x00  (±250 dps)
//
// 측정
//   AX_H -> AX_L -> AY_H -> AY_L -> AZ_H -> AZ_L
//   -> GX_H -> GX_L -> GY_H -> GY_L -> GZ_H -> GZ_L 반복
//================================================================================

module mpu6050_ctrl
(
    input  wire              clk,
    input  wire              rst_n,
    input  wire              tick,

    input  wire              busy,
    input  wire              done,
    input  wire              ack_ok,
    input  wire [7:0]        rx_data,

    output reg               start_req,
    output reg               rw,
    output reg  [7:0]        reg_addr,
    output reg  [7:0]        tx_data,

    output reg signed [15:0] accel_x,
    output reg signed [15:0] accel_y,
    output reg signed [15:0] accel_z,

    output reg signed [15:0] gyro_x,
    output reg signed [15:0] gyro_y,
    output reg signed [15:0] gyro_z,

    output reg               init_done,
    output reg               data_valid
);

//================================================================================
// Register Address
//================================================================================
localparam [7:0] REG_PWR_MGMT_1   = 8'h6B;
localparam [7:0] REG_GYRO_CONFIG  = 8'h1B;
localparam [7:0] REG_ACCEL_CONFIG = 8'h1C;

localparam [7:0] REG_ACCEL_XOUT_H = 8'h3B;
localparam [7:0] REG_ACCEL_XOUT_L = 8'h3C;
localparam [7:0] REG_ACCEL_YOUT_H = 8'h3D;
localparam [7:0] REG_ACCEL_YOUT_L = 8'h3E;
localparam [7:0] REG_ACCEL_ZOUT_H = 8'h3F;
localparam [7:0] REG_ACCEL_ZOUT_L = 8'h40;

localparam [7:0] REG_GYRO_XOUT_H  = 8'h43;
localparam [7:0] REG_GYRO_XOUT_L  = 8'h44;
localparam [7:0] REG_GYRO_YOUT_H  = 8'h45;
localparam [7:0] REG_GYRO_YOUT_L  = 8'h46;
localparam [7:0] REG_GYRO_ZOUT_H  = 8'h47;
localparam [7:0] REG_GYRO_ZOUT_L  = 8'h48;

//================================================================================
// Init Value
//================================================================================
localparam [7:0] VAL_PWR_MGMT_1   = 8'h00;
localparam [7:0] VAL_ACCEL_CONFIG = 8'h00;
localparam [7:0] VAL_GYRO_CONFIG  = 8'h00;

//================================================================================
// State Definition
//================================================================================
localparam [5:0] ST_IDLE         = 6'd0;
localparam [5:0] ST_WR_REQ       = 6'd1;
localparam [5:0] ST_WR_WAIT_BUSY = 6'd2;
localparam [5:0] ST_WR_WAIT_DONE = 6'd3;
localparam [5:0] ST_WR_CHECK     = 6'd4;
localparam [5:0] ST_RD_REQ       = 6'd5;
localparam [5:0] ST_RD_WAIT_BUSY = 6'd6;
localparam [5:0] ST_RD_WAIT_DONE = 6'd7;
localparam [5:0] ST_RD_LATCH     = 6'd8;
localparam [5:0] ST_INIT_PWR     = 6'd9;
localparam [5:0] ST_INIT_ACCEL   = 6'd10;
localparam [5:0] ST_INIT_GYRO    = 6'd11;
localparam [5:0] ST_INIT_DONE    = 6'd12;
localparam [5:0] ST_READ_AX_H    = 6'd13;
localparam [5:0] ST_READ_AX_L    = 6'd14;
localparam [5:0] ST_READ_AY_H    = 6'd15;
localparam [5:0] ST_READ_AY_L    = 6'd16;
localparam [5:0] ST_READ_AZ_H    = 6'd17;
localparam [5:0] ST_READ_AZ_L    = 6'd18;
localparam [5:0] ST_UPDATE_DATA  = 6'd19;
localparam [5:0] ST_READ_GX_H    = 6'd20;
localparam [5:0] ST_READ_GX_L    = 6'd21;
localparam [5:0] ST_READ_GY_H    = 6'd22;
localparam [5:0] ST_READ_GY_L    = 6'd23;
localparam [5:0] ST_READ_GZ_H    = 6'd24;
localparam [5:0] ST_READ_GZ_L    = 6'd25;
localparam [5:0] ST_FAIL         = 6'd26;

//================================================================================
// Internal Register
//================================================================================
reg [5:0] state;
reg [5:0] next_state_after_write;
reg [5:0] next_state_after_read;

reg [7:0] ax_h;
reg [7:0] ax_l;
reg [7:0] ay_h;
reg [7:0] ay_l;
reg [7:0] az_h;
reg [7:0] az_l;

reg [7:0] gx_h;
reg [7:0] gx_l;
reg [7:0] gy_h;
reg [7:0] gy_l;
reg [7:0] gz_h;
reg [7:0] gz_l;

//================================================================================
// FSM
//================================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state                  <= ST_IDLE;
        next_state_after_write <= ST_IDLE;
        next_state_after_read  <= ST_IDLE;

        start_req              <= 1'b0;
        rw                     <= 1'b1;
        reg_addr               <= 8'd0;
        tx_data                <= 8'd0;

        accel_x                <= 16'sd0;
        accel_y                <= 16'sd0;
        accel_z                <= 16'sd0;
        gyro_x                 <= 16'sd0;
        gyro_y                 <= 16'sd0;
        gyro_z                 <= 16'sd0;

        ax_h                   <= 8'd0;
        ax_l                   <= 8'd0;
        ay_h                   <= 8'd0;
        ay_l                   <= 8'd0;
        az_h                   <= 8'd0;
        az_l                   <= 8'd0;

        gx_h                   <= 8'd0;
        gx_l                   <= 8'd0;
        gy_h                   <= 8'd0;
        gy_l                   <= 8'd0;
        gz_h                   <= 8'd0;
        gz_l                   <= 8'd0;

        init_done              <= 1'b0;
        data_valid             <= 1'b0;
    end
    else begin
        if (tick) begin
            case (state)
                ST_IDLE: begin
                    start_req  <= 1'b0;
                    init_done  <= 1'b0;
                    data_valid <= 1'b0;
                    state      <= ST_INIT_PWR;
                end

                ST_INIT_PWR: begin
                    rw                     <= 1'b0;
                    reg_addr               <= REG_PWR_MGMT_1;
                    tx_data                <= VAL_PWR_MGMT_1;
                    next_state_after_write <= ST_INIT_ACCEL;
                    state                  <= ST_WR_REQ;
                end

                ST_INIT_ACCEL: begin
                    rw                     <= 1'b0;
                    reg_addr               <= REG_ACCEL_CONFIG;
                    tx_data                <= VAL_ACCEL_CONFIG;
                    next_state_after_write <= ST_INIT_GYRO;
                    state                  <= ST_WR_REQ;
                end

                ST_INIT_GYRO: begin
                    rw                     <= 1'b0;
                    reg_addr               <= REG_GYRO_CONFIG;
                    tx_data                <= VAL_GYRO_CONFIG;
                    next_state_after_write <= ST_INIT_DONE;
                    state                  <= ST_WR_REQ;
                end

                ST_INIT_DONE: begin
                    init_done  <= 1'b1;
                    data_valid <= 1'b0;
                    state      <= ST_READ_AX_H;
                end

                ST_WR_REQ: begin
                    start_req <= 1'b1;
                    state     <= ST_WR_WAIT_BUSY;
                end

                ST_WR_WAIT_BUSY: begin
                    if (busy) begin
                        start_req <= 1'b0;
                        state     <= ST_WR_WAIT_DONE;
                    end
                end

                ST_WR_WAIT_DONE: begin
                    if (done) begin
                        state <= ST_WR_CHECK;
                    end
                end

                ST_WR_CHECK: begin
                    if (ack_ok)
                        state <= next_state_after_write;
                    else
                        state <= ST_FAIL;
                end

                ST_READ_AX_H: begin
                    rw                    <= 1'b1;
                    reg_addr              <= REG_ACCEL_XOUT_H;
                    tx_data               <= 8'h00;
                    next_state_after_read <= ST_READ_AX_L;
                    state                 <= ST_RD_REQ;
                    data_valid            <= 1'b0;
                end

                ST_READ_AX_L: begin
                    rw                    <= 1'b1;
                    reg_addr              <= REG_ACCEL_XOUT_L;
                    tx_data               <= 8'h00;
                    next_state_after_read <= ST_READ_AY_H;
                    state                 <= ST_RD_REQ;
                end

                ST_READ_AY_H: begin
                    rw                    <= 1'b1;
                    reg_addr              <= REG_ACCEL_YOUT_H;
                    tx_data               <= 8'h00;
                    next_state_after_read <= ST_READ_AY_L;
                    state                 <= ST_RD_REQ;
                end

                ST_READ_AY_L: begin
                    rw                    <= 1'b1;
                    reg_addr              <= REG_ACCEL_YOUT_L;
                    tx_data               <= 8'h00;
                    next_state_after_read <= ST_READ_AZ_H;
                    state                 <= ST_RD_REQ;
                end

                ST_READ_AZ_H: begin
                    rw                    <= 1'b1;
                    reg_addr              <= REG_ACCEL_ZOUT_H;
                    tx_data               <= 8'h00;
                    next_state_after_read <= ST_READ_AZ_L;
                    state                 <= ST_RD_REQ;
                end

                ST_READ_AZ_L: begin
                    rw                    <= 1'b1;
                    reg_addr              <= REG_ACCEL_ZOUT_L;
                    tx_data               <= 8'h00;
                    next_state_after_read <= ST_READ_GX_H;
                    state                 <= ST_RD_REQ;
                end

                ST_READ_GX_H: begin
                    rw                    <= 1'b1;
                    reg_addr              <= REG_GYRO_XOUT_H;
                    tx_data               <= 8'h00;
                    next_state_after_read <= ST_READ_GX_L;
                    state                 <= ST_RD_REQ;
                end

                ST_READ_GX_L: begin
                    rw                    <= 1'b1;
                    reg_addr              <= REG_GYRO_XOUT_L;
                    tx_data               <= 8'h00;
                    next_state_after_read <= ST_READ_GY_H;
                    state                 <= ST_RD_REQ;
                end

                ST_READ_GY_H: begin
                    rw                    <= 1'b1;
                    reg_addr              <= REG_GYRO_YOUT_H;
                    tx_data               <= 8'h00;
                    next_state_after_read <= ST_READ_GY_L;
                    state                 <= ST_RD_REQ;
                end

                ST_READ_GY_L: begin
                    rw                    <= 1'b1;
                    reg_addr              <= REG_GYRO_YOUT_L;
                    tx_data               <= 8'h00;
                    next_state_after_read <= ST_READ_GZ_H;
                    state                 <= ST_RD_REQ;
                end

                ST_READ_GZ_H: begin
                    rw                    <= 1'b1;
                    reg_addr              <= REG_GYRO_ZOUT_H;
                    tx_data               <= 8'h00;
                    next_state_after_read <= ST_READ_GZ_L;
                    state                 <= ST_RD_REQ;
                end

                ST_READ_GZ_L: begin
                    rw                    <= 1'b1;
                    reg_addr              <= REG_GYRO_ZOUT_L;
                    tx_data               <= 8'h00;
                    next_state_after_read <= ST_UPDATE_DATA;
                    state                 <= ST_RD_REQ;
                end

                ST_RD_REQ: begin
                    start_req <= 1'b1;
                    state     <= ST_RD_WAIT_BUSY;
                end

                ST_RD_WAIT_BUSY: begin
                    if (busy) begin
                        start_req <= 1'b0;
                        state     <= ST_RD_WAIT_DONE;
                    end
                end

                ST_RD_WAIT_DONE: begin
                    if (done) begin
                        state <= ST_RD_LATCH;
                    end
                end

                ST_RD_LATCH: begin
                    if (ack_ok) begin
                        case (reg_addr)
                            REG_ACCEL_XOUT_H: ax_h <= rx_data;
                            REG_ACCEL_XOUT_L: ax_l <= rx_data;
                            REG_ACCEL_YOUT_H: ay_h <= rx_data;
                            REG_ACCEL_YOUT_L: ay_l <= rx_data;
                            REG_ACCEL_ZOUT_H: az_h <= rx_data;
                            REG_ACCEL_ZOUT_L: az_l <= rx_data;

                            REG_GYRO_XOUT_H : gx_h <= rx_data;
                            REG_GYRO_XOUT_L : gx_l <= rx_data;
                            REG_GYRO_YOUT_H : gy_h <= rx_data;
                            REG_GYRO_YOUT_L : gy_l <= rx_data;
                            REG_GYRO_ZOUT_H : gz_h <= rx_data;
                            REG_GYRO_ZOUT_L : gz_l <= rx_data;
                            default: ;
                        endcase
                        state <= next_state_after_read;
                    end
                    else begin
                        state <= ST_FAIL;
                    end
                end

                ST_UPDATE_DATA: begin
                    accel_x    <= {ax_h, ax_l};
                    accel_y    <= {ay_h, ay_l};
                    accel_z    <= {az_h, az_l};

                    gyro_x     <= {gx_h, gx_l};
                    gyro_y     <= {gy_h, gy_l};
                    gyro_z     <= {gz_h, gz_l};

                    data_valid <= 1'b1;
                    state      <= ST_READ_AX_H;
                end

                ST_FAIL: begin
                    start_req  <= 1'b0;
                    init_done  <= 1'b0;
                    data_valid <= 1'b0;
                    state      <= ST_INIT_PWR;
                end

                default: begin
                    state                  <= ST_IDLE;
                    next_state_after_write <= ST_IDLE;
                    next_state_after_read  <= ST_IDLE;

                    start_req              <= 1'b0;
                    rw                     <= 1'b1;
                    reg_addr               <= 8'd0;
                    tx_data                <= 8'd0;

                    accel_x                <= 16'sd0;
                    accel_y                <= 16'sd0;
                    accel_z                <= 16'sd0;
                    gyro_x                 <= 16'sd0;
                    gyro_y                 <= 16'sd0;
                    gyro_z                 <= 16'sd0;

                    ax_h                   <= 8'd0;
                    ax_l                   <= 8'd0;
                    ay_h                   <= 8'd0;
                    ay_l                   <= 8'd0;
                    az_h                   <= 8'd0;
                    az_l                   <= 8'd0;

                    gx_h                   <= 8'd0;
                    gx_l                   <= 8'd0;
                    gy_h                   <= 8'd0;
                    gy_l                   <= 8'd0;
                    gz_h                   <= 8'd0;
                    gz_l                   <= 8'd0;

                    init_done              <= 1'b0;
                    data_valid             <= 1'b0;
                end
            endcase
        end
    end
end

endmodule