//==============================================================================
// 模块名称: uart_rx (UART 接收器)
// 功能描述: 实现异步串行数据接收，支持可配置波特率、起始位检测、
//           数据位采样、停止位校验及错误检测。
//==============================================================================
//
// 【外设功能与寄存器映射】
//   本模块是一个纯硬线控制的 UART 接收器，无外部可寻址寄存器。
//   内部寄存器与信号的用途如下：
//     - state / next_state : 2-bit FSM 状态寄存器，控制接收流程
//     - counter            : 16-bit 波特率分频计数器，决定每位采样时刻
//     - bit_idx            : 3-bit 接收位索引，计数已接收的数据位 (0~7)
//     - shift_reg          : 8-bit 移位寄存器，存放正在接收的数据字节
//     - start_found        : 1-bit 标记，防止在 IDLE 态重复捕获起始位
//     - rx_data            : 8-bit 输出寄存器，存放完整接收字节
//     - rx_ready           : 1-bit 输出，指示 rx_data 上数据有效（单周期脉冲）
//     - rx_error           : 1-bit 输出，指示停止位校验失败
//
//==============================================================================
// 【接口协议】
//
//  A. 总线侧（系统接口）
//     - clk       : 系统时钟输入，所有时序逻辑同步于此上升沿
//     - reset     : 同步复位（高有效），清零所有状态寄存器
//     - baud_div [15:0]
//                : 波特率分频系数。
//                  每位持续时间 = (baud_div + 1) / clk_freq  （因 counter 从
//                  baud_div 递减至 0 共计 baud_div+1 个时钟周期）
//     - rx_data [7:0]
//                : 接收到的 8-bit 数据，LSB 优先（UART 标准帧序）
//                  仅在 rx_ready=1 时有效
//     - rx_ready  : 数据有效标志，持续一个 clk 周期的正脉冲
//     - rx_error  : 帧错误标志，当停止位采样值不为 1 时拉高
//
//  B. 外部侧（UART 物理接口）
//     - rx        : 串行数据输入（异步信号，本模块内部直接同步采样，
//                   无专用同步器级联——依赖外部调用者确保信号已同步
//                   或波特率足够低，满足亚稳态抑制要求）
//
//==============================================================================
// 【UART 帧格式】
//   标准 8N1 格式（固定，不可配置）：
//     - 1 起始位（低电平）
//     - 8 数据位（LSB 在先）
//     - 1 停止位（高电平）
//     - 无奇偶校验位
//
//==============================================================================
// 【有限状态机 (FSM)】
//
//   状态编码:
//     RX_IDLE  = 2'b00  —  空闲，等待起始位
//     RX_START = 2'b01  —  起始位确认阶段
//     RX_DATA  = 2'b10  —  数据位接收
//     RX_STOP  = 2'b11  —  停止位采样
//
//   状态转移:
//
//     RX_IDLE ──(rx==0)──> RX_START      // 检测到下降沿（起始位起始）
//     RX_START──(counter==0 && rx==0)──> RX_DATA   // 半位后确认仍为低
//     RX_START──(counter==0 && rx!=0)──> RX_IDLE   // 伪起始位，退回空闲
//     RX_DATA ──(counter==0 && bit_idx==7)──> RX_STOP  // 8 位收完
//     RX_DATA ──(counter==0 && bit_idx!=7)──> RX_DATA  // 继续收下一位
//     RX_STOP ──(counter==0)──> RX_IDLE   // 停止位采样完毕
//
//==============================================================================
// 【时序与同步设计】
//
//  1. 波特率发生器：
//     用一个 16-bit 递减计数器 (counter) 实现。
//     - 每次计到 0 时触发一个"位节拍"，在该节拍执行状态机动作
//       （采样当前位、移位、或跳转状态）
//     - 每一位的时间长度 = baud_div + 1 个 clk 周期
//
//  2. 过采样策略：
//     采用单倍采样（每位采样一次），采样时刻位于位窗口末尾
//     （counter==0 时采样）。
//     起始位的特殊处理：
//     - 在 RX_IDLE 检测到 rx==0 后，counter 初始化为 baud_div>>1
//       （即 baud_div/2），使得第一个采样时刻落在起始位的中点，
//       从而在起始位确认阶段 (RX_START) 验证起始位时，
//       采样偏移已对齐到位中心。
//     - 后续各数据位和停止位均以完整 baud_div 周期为间隔采样，
//       因此所有采样点都落在各位的近似中心位置。
//
//  3. 同步策略：
//     - rx 输入作为异步信号直接进入组合逻辑（用于起始位检测）
//       及同步时序逻辑（用于数据/停止位采样）。
//     - 本模块未在内部对 rx 做多级同步器同步；
//       在 SoC 集成环境中，通常由顶层模块在 uart_rx 之前插入
//       2~3 级 DFF 同步链，以抑制亚稳态。
//
//  4. 输出时序：
//     - rx_ready 在 RX_STOP 状态下 counter==0 的那个时钟周期拉高，
//       持续仅 1 个时钟周期（下一个时钟周期状态返回 RX_IDLE 并清零）。
//     - rx_data 与 rx_ready 同时更新，便于下游模块在同一周期采样。
//     - rx_error 是组合逻辑输出，在停止位采样点的那个周期有效。
//
//==============================================================================

module uart_rx(
    input           clk,
    input           reset,
    input           rx,
    input  [15:0]   baud_div,
    output reg      rx_ready,
    output reg [7:0] rx_data,
    output          rx_error
);

    // ---- FSM 状态编码 ----
    localparam RX_IDLE  = 2'd0,   // 空闲：等待起始位
               RX_START = 2'd1,   // 起始位确认
               RX_DATA  = 2'd2,   // 数据位接收
               RX_STOP  = 2'd3;   // 停止位校验

    // ---- 内部寄存器 ----
    reg [2:0]  state, next_state;   // FSM 当前状态与下一状态
    reg [15:0] counter;             // 波特率分频计数器
    reg [2:0]  bit_idx;             // 已接收数据位计数 (0~7)
    reg [7:0]  shift_reg;           // 移位寄存器 (LSB 先收)
    reg        start_found;         // 起始位已检测标志（防重入）

    // ---- 停止位错误检测（组合逻辑） ----
    // 在停止位采样点 (state==RX_STOP 且 counter==1 的下一个上升沿
    // counter 变为 0 的那个周期)，若 rx 不为 1，则报帧错误。
    // 注意：由于 rx_error 是组合逻辑，它在 counter==1 的这个周期
    // （即停止位即将被采样的那个周期）提前反映错误状态。
    assign rx_error = (state == RX_STOP) && (counter == 16'd1) && (rx != 1'b1);

    // ---- 状态寄存器（同步复位） ----
    always @(posedge clk)
        if (reset)
            state <= RX_IDLE;
        else
            state <= next_state;

    // ---- FSM 次态逻辑与数据通路（单 always 块风格） ----
    always @(posedge clk)
        if (reset) begin
            // 复位：清零所有寄存器，强制进入 IDLE
            next_state  <= RX_IDLE;
            counter     <= 16'd0;
            bit_idx     <= 3'd0;
            shift_reg   <= 8'd0;
            rx_data     <= 8'd0;
            rx_ready    <= 1'b0;
            start_found <= 1'b0;
        end else begin
            case (state)

                //--------------------------------------------------------------
                // RX_IDLE: 空闲状态，检测起始位（rx 下降沿）
                //--------------------------------------------------------------
                RX_IDLE: begin
                    rx_ready    <= 1'b0;       // 清零数据有效标志
                    start_found <= 1'b0;       // 清除防重入标记
                    if (rx == 1'b0 && !start_found) begin
                        // 检测到 rx 低电平：可能是起始位
                        start_found <= 1'b1;
                        // 初始化计数器为半个位周期，使采样点落在起始位中点
                        counter     <= baud_div >> 1;
                        next_state  <= RX_START;
                    end else begin
                        next_state  <= RX_IDLE;
                    end
                end

                //--------------------------------------------------------------
                // RX_START: 等待半位周期后，确认起始位有效性
                //--------------------------------------------------------------
                RX_START: begin
                    if (counter == 16'd0) begin
                        // 到达采样点：检测 rx 是否仍为低
                        if (rx == 1'b0) begin
                            // 确认为有效起始位，准备接收数据
                            bit_idx    <= 3'd0;
                            counter    <= baud_div;     // 恢复完整位周期
                            next_state <= RX_DATA;
                        end else begin
                            // 伪起始位（噪声毛刺），回归空闲
                            next_state <= RX_IDLE;
                        end
                    end else begin
                        counter    <= counter - 1;
                        next_state <= RX_START;
                    end
                end

                //--------------------------------------------------------------
                // RX_DATA: 逐位接收 8 个数据位（LSB 最先到达）
                //--------------------------------------------------------------
                RX_DATA: begin
                    if (counter == 16'd0) begin
                        // 采样 rx，右移装入 shift_reg (LSB first)
                        // 新收到的 bit 放在 shift_reg[7]，原有数据右移
                        shift_reg <= {rx, shift_reg[7:1]};
                        if (bit_idx == 3'd7) begin
                            // 已收满 8 位，转入停止位接收
                            counter    <= baud_div;
                            next_state <= RX_STOP;
                        end else begin
                            // 继续接收下一位
                            bit_idx    <= bit_idx + 1;
                            counter    <= baud_div;
                            next_state <= RX_DATA;
                        end
                    end else begin
                        counter    <= counter - 1;
                        next_state <= RX_DATA;
                    end
                end

                //--------------------------------------------------------------
                // RX_STOP: 等待一个完整位周期后，锁存数据并返回空闲
                //--------------------------------------------------------------
                RX_STOP: begin
                    if (counter == 16'd0) begin
                        // 停止位采样时刻：将移位寄存器值输出到 rx_data
                        rx_data    <= shift_reg;
                        rx_ready   <= 1'b1;        // 拉高一个周期
                        next_state <= RX_IDLE;
                    end else begin
                        counter    <= counter - 1;
                        next_state <= RX_STOP;
                    end
                end

                //--------------------------------------------------------------
                // 非法状态兜底
                //--------------------------------------------------------------
                default: begin
                    next_state <= RX_IDLE;
                end

            endcase
        end

endmodule
