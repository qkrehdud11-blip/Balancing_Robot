`timescale 1ns / 1ps
//================================================================================
// i2c_master
// - I2C 프로토콜의 START / 1-byte WRITE / ACK / STOP 기본 동작을 담당하는 모듈
// - 현재 단계는 MPU6050의 slave address byte(Write) 전송 + ACK 확인용 FSM
// - 아직 Register Write / Repeated Start / Read는 구현하지 않음
//================================================================================
module i2c_master
(
    input  wire clk,             // System clock
    input  wire rst_n,           // Active low reset
    input  wire tick,            // Slow tick from clk_divider
    input  wire sda_in,          // SDA input sampled from top-level pin

    output reg  sda_enable,      // 1: SDA Low, 0: SDA Release
    output reg  scl_enable,      // 1: SCL Low, 0: SCL Release
    output reg  ack_ok           // 1: ACK received, 0: NACK or not checked yet
);

//================================================================================
// MPU6050 Address / Register Definition
//================================================================================
localparam [6:0] MPU6050_ADDR_7BIT = 7'h68;
localparam [7:0] MPU6050_ADDR_W    = {MPU6050_ADDR_7BIT, 1'b0}; // 0xD0
localparam [7:0] MPU6050_ADDR_R    = {MPU6050_ADDR_7BIT, 1'b1}; // 0xD1

localparam [7:0] REG_PWR_MGMT_1    = 8'h6B;
localparam [7:0] REG_WHO_AM_I      = 8'h75;
localparam [7:0] REG_ACCEL_XOUT_H  = 8'h3B;
localparam [7:0] REG_GYRO_XOUT_H   = 8'h43;

//================================================================================
// State Definition
//================================================================================
localparam [3:0] IDLE     = 4'd0; // SDA, SCL release
localparam [3:0] START_A  = 4'd1; // IDLE 안정화
localparam [3:0] START_B  = 4'd2; // START 조건 생성
localparam [3:0] BIT_LOW  = 4'd3; // SCL Low, SDA에 bit 세팅
localparam [3:0] BIT_HIGH = 4'd4; // SCL High, SDA 유지
localparam [3:0] ACK_LOW  = 4'd5; // ACK 준비: SDA release, SCL Low
localparam [3:0] ACK_HIGH = 4'd6; // ACK 확인: SCL High에서 SDA 읽기
localparam [3:0] STOP_A   = 4'd7; // STOP 준비
localparam [3:0] STOP_B   = 4'd8; // STOP 조건 생성

//================================================================================
// Internal Register
//================================================================================
reg [3:0] state;
reg [7:0] tx_data;   // 현재 전송할 1-byte 데이터
reg [2:0] bit_cnt;   // MSB -> LSB 전송용 비트 카운터

//================================================================================
// FSM
//================================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state      <= IDLE;
        sda_enable <= 1'b0;           // release -> High
        scl_enable <= 1'b0;           // release -> High
        ack_ok     <= 1'b0;
        tx_data    <= MPU6050_ADDR_W; // 예제: MPU6050 write address byte
        bit_cnt    <= 3'd7;
    end
    else begin
        if (tick) begin
            case (state)
                IDLE: begin
                    // I2C idle 상태
                    // SDA, SCL 모두 release 상태(High)
                    sda_enable <= 1'b0;
                    scl_enable <= 1'b0;
                    ack_ok     <= 1'b0;

                    // 현재 단계에서는 MPU6050 write address byte를 전송 연습
                    tx_data    <= MPU6050_ADDR_W;
                    bit_cnt    <= 3'd7;

                    state      <= START_A;
                end

                START_A: begin
                    // IDLE 안정화
                    sda_enable <= 1'b0; // SDA High
                    scl_enable <= 1'b0; // SCL High
                    state      <= START_B;
                end

                START_B: begin
                    // START 조건:
                    // SCL이 High일 때 SDA를 High -> Low로 변경
                    sda_enable <= 1'b1; // SDA Low
                    scl_enable <= 1'b0; // SCL High
                    state      <= BIT_LOW;
                end

                BIT_LOW: begin
                    // SCL Low 구간에서 현재 전송할 bit를 SDA에 준비
                    scl_enable <= 1'b1; // SCL Low

                    // I2C open-drain:
                    // 0 전송 -> SDA Low
                    // 1 전송 -> SDA release
                    if (tx_data[bit_cnt] == 1'b0)
                        sda_enable <= 1'b1;
                    else
                        sda_enable <= 1'b0;

                    state <= BIT_HIGH;
                end

                BIT_HIGH: begin
                    // SCL High 구간에서 SDA 값을 유지
                    // 슬레이브는 이 구간에서 데이터를 샘플링
                    scl_enable <= 1'b0; // SCL High

                    if (bit_cnt == 3'd0)
                        state <= ACK_LOW;
                    else begin
                        bit_cnt <= bit_cnt - 3'd1;
                        state   <= BIT_LOW;
                    end
                end

                ACK_LOW: begin
                    // ACK 준비:
                    // Master는 SDA를 release 해야 Slave가 ACK를 보낼 수 있음
                    sda_enable <= 1'b0; // SDA release
                    scl_enable <= 1'b1; // SCL Low
                    state      <= ACK_HIGH;
                end

                ACK_HIGH: begin
                    // ACK 확인:
                    // SCL High 상태에서 SDA를 읽음
                    scl_enable <= 1'b0; // SCL High

                    // SDA가 Low면 ACK, High면 NACK
                    if (sda_in == 1'b0)
                        ack_ok <= 1'b1;
                    else
                        ack_ok <= 1'b0;

                    state <= STOP_A;
                end

                STOP_A: begin
                    // STOP 준비:
                    // SDA Low 상태 유지, SCL High
                    sda_enable <= 1'b1; // SDA Low
                    scl_enable <= 1'b0; // SCL High
                    state      <= STOP_B;
                end

                STOP_B: begin
                    // STOP 조건:
                    // SCL High일 때 SDA를 Low -> High(release)로 변경
                    sda_enable <= 1'b0; // SDA release -> High
                    scl_enable <= 1'b0; // SCL High
                    state      <= IDLE;
                end

                default: begin
                    state      <= IDLE;
                    sda_enable <= 1'b0;
                    scl_enable <= 1'b0;
                    ack_ok     <= 1'b0;
                    tx_data    <= MPU6050_ADDR_W;
                    bit_cnt    <= 3'd7;
                end
            endcase
        end
    end
end

endmodule