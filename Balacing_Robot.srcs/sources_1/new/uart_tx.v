`timescale 1ns / 1ps
//================================================================================
// uart_tx
// - 8N1 UART 송신기
// - Basys3 100MHz 클럭 기준
// - Baud rate = 9600 고정
//
// 동작
//   tx_start가 1이 되면 tx_data 1바이트를 송신 시작
//   busy=1 동안 송신 진행
//   done=1은 송신 완료 pulse
//================================================================================

module uart_tx
(
    input  wire       clk,
    input  wire       rst_n,

    input  wire       tx_start,
    input  wire [7:0] tx_data,

    output reg        tx,
    output reg        busy,
    output reg        done
);

//================================================================================
// UART Baud 설정
//================================================================================
localparam integer CLK_HZ       = 100_000_000;
localparam integer BAUD         = 9_600;
localparam integer BAUD_CNT_MAX = CLK_HZ / BAUD;

//================================================================================
// State Definition
//================================================================================
localparam [1:0] ST_IDLE  = 2'd0;
localparam [1:0] ST_START = 2'd1;
localparam [1:0] ST_DATA  = 2'd2;
localparam [1:0] ST_STOP  = 2'd3;

reg [1:0] state;
reg [13:0] baud_cnt;
reg [2:0] bit_idx;
reg [7:0] shifter;

//================================================================================
// FSM
//================================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state    <= ST_IDLE;
        baud_cnt <= 14'd0;
        bit_idx  <= 3'd0;
        shifter  <= 8'd0;

        tx       <= 1'b1;   // UART idle = High
        busy     <= 1'b0;
        done     <= 1'b0;
    end
    else begin
        // done은 1클럭 pulse로 사용
        done <= 1'b0;

        case (state)
            //------------------------------------------------------------------
            // 대기 상태
            //------------------------------------------------------------------
            ST_IDLE: begin
                tx       <= 1'b1;
                busy     <= 1'b0;
                baud_cnt <= 14'd0;
                bit_idx  <= 3'd0;

                if (tx_start) begin
                    shifter <= tx_data;
                    busy    <= 1'b1;
                    state   <= ST_START;
                end
            end

            //------------------------------------------------------------------
            // Start bit
            //------------------------------------------------------------------
            ST_START: begin
                tx <= 1'b0;

                if (baud_cnt == BAUD_CNT_MAX - 1) begin
                    baud_cnt <= 14'd0;
                    state    <= ST_DATA;
                end
                else begin
                    baud_cnt <= baud_cnt + 14'd1;
                end
            end

            //------------------------------------------------------------------
            // Data bits (LSB first)
            //------------------------------------------------------------------
            ST_DATA: begin
                tx <= shifter[bit_idx];

                if (baud_cnt == BAUD_CNT_MAX - 1) begin
                    baud_cnt <= 14'd0;

                    if (bit_idx == 3'd7) begin
                        bit_idx <= 3'd0;
                        state   <= ST_STOP;
                    end
                    else begin
                        bit_idx <= bit_idx + 3'd1;
                    end
                end
                else begin
                    baud_cnt <= baud_cnt + 14'd1;
                end
            end

            //------------------------------------------------------------------
            // Stop bit
            //------------------------------------------------------------------
            ST_STOP: begin
                tx <= 1'b1;

                if (baud_cnt == BAUD_CNT_MAX - 1) begin
                    baud_cnt <= 14'd0;
                    busy     <= 1'b0;
                    done     <= 1'b1;
                    state    <= ST_IDLE;
                end
                else begin
                    baud_cnt <= baud_cnt + 14'd1;
                end
            end

            //------------------------------------------------------------------
            // default
            //------------------------------------------------------------------
            default: begin
                state    <= ST_IDLE;
                baud_cnt <= 14'd0;
                bit_idx  <= 3'd0;
                shifter  <= 8'd0;

                tx       <= 1'b1;
                busy     <= 1'b0;
                done     <= 1'b0;
            end
        endcase
    end
end

endmodule