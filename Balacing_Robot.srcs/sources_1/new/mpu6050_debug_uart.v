`timescale 1ns / 1ps
//==============================================================================
// mpu6050_debug_uart
// - MPU6050 accel / gyro 6축 값을 Moserial에서 16진수로 출력
// - 내부 값은 signed 16bit 그대로 사용
// - 출력만 보기 쉽게 AX=1234 형태로 보냄
// - 기존 uart_tx 모듈 그대로 사용
//
// 출력 예:
// AX=FB34 AY=0124 AZ=3F84 GX=0012 GY=FF98 GZ=000A
//==============================================================================

module mpu6050_debug_uart
(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               init_done,
    input  wire               data_valid,

    input  wire signed [15:0] accel_x,
    input  wire signed [15:0] accel_y,
    input  wire signed [15:0] accel_z,

    input  wire signed [15:0] gyro_x,
    input  wire signed [15:0] gyro_y,
    input  wire signed [15:0] gyro_z,

    output wire               uart_tx_o
);

    //==========================================================================
    // 기존 uart_tx 사용
    //==========================================================================
    reg        tx_start;
    reg [7:0]  tx_data;
    wire       tx_busy;
    wire       tx_done;

    uart_tx u_uart_tx
    (
        .clk      (clk),
        .rst_n    (rst_n),
        .tx_start (tx_start),
        .tx_data  (tx_data),
        .tx       (uart_tx_o),
        .busy     (tx_busy),
        .done     (tx_done)
    );

    //==========================================================================
    // 출력 주기 설정
    // 너무 자주 출력하지 않도록 10Hz 정도로 제한
    //==========================================================================
    localparam [31:0] PRINT_CNT_MAX = 32'd9_999_999;

    reg [31:0] print_cnt;
    reg        print_tick;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            print_cnt  <= 32'd0;
            print_tick <= 1'b0;
        end
        else begin
            print_tick <= 1'b0;

            if (print_cnt == PRINT_CNT_MAX) begin
                print_cnt  <= 32'd0;
                print_tick <= 1'b1;
            end
            else begin
                print_cnt <= print_cnt + 32'd1;
            end
        end
    end

    //==========================================================================
    // data_valid pulse 보관
    //==========================================================================
    reg sample_ready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sample_ready <= 1'b0;
        end
        else begin
            if (!init_done) begin
                sample_ready <= 1'b0;
            end
            else begin
                if (data_valid)
                    sample_ready <= 1'b1;

                if (state == ST_CAPTURE)
                    sample_ready <= 1'b0;
            end
        end
    end

    //==========================================================================
    // 출력 중 값 고정용 스냅샷
    //==========================================================================
    reg [15:0] ax_r, ay_r, az_r;
    reg [15:0] gx_r, gy_r, gz_r;

    //==========================================================================
    // 현재 출력 중인 센서값 / 라벨 / 16진수 각 자리
    //==========================================================================
    reg [2:0]  sensor_idx;
    reg [15:0] cur_val;
    reg [7:0]  label0;
    reg [7:0]  label1;

    reg [3:0] hex3, hex2, hex1, hex0;
    reg [7:0] ascii_ch;

    //==========================================================================
    // 상태 정의
    //==========================================================================
    localparam [5:0]
        ST_IDLE         = 6'd0,
        ST_CAPTURE      = 6'd1,
        ST_SELECT       = 6'd2,
        ST_SPLIT        = 6'd3,

        ST_TX_L0        = 6'd4,
        ST_WAIT_L0      = 6'd5,
        ST_TX_L1        = 6'd6,
        ST_WAIT_L1      = 6'd7,
        ST_TX_EQ        = 6'd8,
        ST_WAIT_EQ      = 6'd9,

        ST_TX_H3        = 6'd10,
        ST_WAIT_H3      = 6'd11,
        ST_TX_H2        = 6'd12,
        ST_WAIT_H2      = 6'd13,
        ST_TX_H1        = 6'd14,
        ST_WAIT_H1      = 6'd15,
        ST_TX_H0        = 6'd16,
        ST_WAIT_H0      = 6'd17,

        ST_TX_SEP       = 6'd18,
        ST_WAIT_SEP     = 6'd19,
        ST_NEXT_SENSOR  = 6'd20,

        ST_TX_CR        = 6'd21,
        ST_WAIT_CR      = 6'd22,
        ST_TX_LF        = 6'd23,
        ST_WAIT_LF      = 6'd24;

    reg [5:0] state;

    //==========================================================================
    // 4bit hex nibble -> ASCII
    //==========================================================================
    function [7:0] hex_to_ascii;
        input [3:0] nib;
        begin
            case (nib)
                4'h0: hex_to_ascii = "0";
                4'h1: hex_to_ascii = "1";
                4'h2: hex_to_ascii = "2";
                4'h3: hex_to_ascii = "3";
                4'h4: hex_to_ascii = "4";
                4'h5: hex_to_ascii = "5";
                4'h6: hex_to_ascii = "6";
                4'h7: hex_to_ascii = "7";
                4'h8: hex_to_ascii = "8";
                4'h9: hex_to_ascii = "9";
                4'hA: hex_to_ascii = "A";
                4'hB: hex_to_ascii = "B";
                4'hC: hex_to_ascii = "C";
                4'hD: hex_to_ascii = "D";
                4'hE: hex_to_ascii = "E";
                default: hex_to_ascii = "F";
            endcase
        end
    endfunction

    //==========================================================================
    // 메인 FSM
    //==========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_start   <= 1'b0;
            tx_data    <= 8'd0;

            ax_r       <= 16'd0;
            ay_r       <= 16'd0;
            az_r       <= 16'd0;
            gx_r       <= 16'd0;
            gy_r       <= 16'd0;
            gz_r       <= 16'd0;

            sensor_idx <= 3'd0;
            cur_val    <= 16'd0;
            label0     <= 8'd0;
            label1     <= 8'd0;

            hex3       <= 4'd0;
            hex2       <= 4'd0;
            hex1       <= 4'd0;
            hex0       <= 4'd0;
            ascii_ch   <= 8'd0;

            state      <= ST_IDLE;
        end
        else begin
            // tx_start는 1클럭 pulse로만 사용
            tx_start <= 1'b0;

            case (state)
                //------------------------------------------------------------------
                // 대기
                //------------------------------------------------------------------
                ST_IDLE: begin
                    if (init_done && sample_ready && print_tick) begin
                        state <= ST_CAPTURE;
                    end
                end

                //------------------------------------------------------------------
                // 한 줄 시작 전에 현재 값 고정
                //------------------------------------------------------------------
                ST_CAPTURE: begin
                    ax_r       <= accel_x[15:0];
                    ay_r       <= accel_y[15:0];
                    az_r       <= accel_z[15:0];
                    gx_r       <= gyro_x[15:0];
                    gy_r       <= gyro_y[15:0];
                    gz_r       <= gyro_z[15:0];
                    sensor_idx <= 3'd0;
                    state      <= ST_SELECT;
                end

                //------------------------------------------------------------------
                // 현재 출력할 축 선택
                //------------------------------------------------------------------
                ST_SELECT: begin
                    case (sensor_idx)
                        3'd0: begin cur_val <= ax_r; label0 <= "A"; label1 <= "X"; end
                        3'd1: begin cur_val <= ay_r; label0 <= "A"; label1 <= "Y"; end
                        3'd2: begin cur_val <= az_r; label0 <= "A"; label1 <= "Z"; end
                        3'd3: begin cur_val <= gx_r; label0 <= "G"; label1 <= "X"; end
                        3'd4: begin cur_val <= gy_r; label0 <= "G"; label1 <= "Y"; end
                        default: begin cur_val <= gz_r; label0 <= "G"; label1 <= "Z"; end
                    endcase
                    state <= ST_SPLIT;
                end

                //------------------------------------------------------------------
                // 16비트 값을 nibble 4개로 분리
                //------------------------------------------------------------------
                ST_SPLIT: begin
                    hex3  <= cur_val[15:12];
                    hex2  <= cur_val[11:8];
                    hex1  <= cur_val[7:4];
                    hex0  <= cur_val[3:0];
                    state <= ST_TX_L0;
                end

                //------------------------------------------------------------------
                // label0
                //------------------------------------------------------------------
                ST_TX_L0: begin
                    if (!tx_busy) begin
                        tx_data  <= label0;
                        tx_start <= 1'b1;
                        state    <= ST_WAIT_L0;
                    end
                end

                ST_WAIT_L0: begin
                    if (tx_done)
                        state <= ST_TX_L1;
                end

                //------------------------------------------------------------------
                // label1
                //------------------------------------------------------------------
                ST_TX_L1: begin
                    if (!tx_busy) begin
                        tx_data  <= label1;
                        tx_start <= 1'b1;
                        state    <= ST_WAIT_L1;
                    end
                end

                ST_WAIT_L1: begin
                    if (tx_done)
                        state <= ST_TX_EQ;
                end

                //------------------------------------------------------------------
                // '='
                //------------------------------------------------------------------
                ST_TX_EQ: begin
                    if (!tx_busy) begin
                        tx_data  <= "=";
                        tx_start <= 1'b1;
                        state    <= ST_WAIT_EQ;
                    end
                end

                ST_WAIT_EQ: begin
                    if (tx_done)
                        state <= ST_TX_H3;
                end

                //------------------------------------------------------------------
                // hex3
                //------------------------------------------------------------------
                ST_TX_H3: begin
                    if (!tx_busy) begin
                        tx_data  <= hex_to_ascii(hex3);
                        tx_start <= 1'b1;
                        state    <= ST_WAIT_H3;
                    end
                end

                ST_WAIT_H3: begin
                    if (tx_done)
                        state <= ST_TX_H2;
                end

                //------------------------------------------------------------------
                // hex2
                //------------------------------------------------------------------
                ST_TX_H2: begin
                    if (!tx_busy) begin
                        tx_data  <= hex_to_ascii(hex2);
                        tx_start <= 1'b1;
                        state    <= ST_WAIT_H2;
                    end
                end

                ST_WAIT_H2: begin
                    if (tx_done)
                        state <= ST_TX_H1;
                end

                //------------------------------------------------------------------
                // hex1
                //------------------------------------------------------------------
                ST_TX_H1: begin
                    if (!tx_busy) begin
                        tx_data  <= hex_to_ascii(hex1);
                        tx_start <= 1'b1;
                        state    <= ST_WAIT_H1;
                    end
                end

                ST_WAIT_H1: begin
                    if (tx_done)
                        state <= ST_TX_H0;
                end

                //------------------------------------------------------------------
                // hex0
                //------------------------------------------------------------------
                ST_TX_H0: begin
                    if (!tx_busy) begin
                        tx_data  <= hex_to_ascii(hex0);
                        tx_start <= 1'b1;
                        state    <= ST_WAIT_H0;
                    end
                end

                ST_WAIT_H0: begin
                    if (tx_done)
                        state <= ST_TX_SEP;
                end

                //------------------------------------------------------------------
                // 센서 사이 공백 또는 줄바꿈
                //------------------------------------------------------------------
                ST_TX_SEP: begin
                    if (!tx_busy) begin
                        if (sensor_idx == 3'd5) begin
                            state <= ST_TX_CR;
                        end
                        else begin
                            tx_data  <= " ";
                            tx_start <= 1'b1;
                            state    <= ST_WAIT_SEP;
                        end
                    end
                end

                ST_WAIT_SEP: begin
                    if (tx_done)
                        state <= ST_NEXT_SENSOR;
                end

                //------------------------------------------------------------------
                // 다음 센서 선택
                //------------------------------------------------------------------
                ST_NEXT_SENSOR: begin
                    sensor_idx <= sensor_idx + 3'd1;
                    state      <= ST_SELECT;
                end

                //------------------------------------------------------------------
                // CR
                //------------------------------------------------------------------
                ST_TX_CR: begin
                    if (!tx_busy) begin
                        tx_data  <= 8'h0D;
                        tx_start <= 1'b1;
                        state    <= ST_WAIT_CR;
                    end
                end

                ST_WAIT_CR: begin
                    if (tx_done)
                        state <= ST_TX_LF;
                end

                //------------------------------------------------------------------
                // LF
                //------------------------------------------------------------------
                ST_TX_LF: begin
                    if (!tx_busy) begin
                        tx_data  <= 8'h0A;
                        tx_start <= 1'b1;
                        state    <= ST_WAIT_LF;
                    end
                end

                ST_WAIT_LF: begin
                    if (tx_done)
                        state <= ST_IDLE;
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule