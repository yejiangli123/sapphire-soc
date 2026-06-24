// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  uart_agent.sv — UART 被动 Agent（Passive Agent）                             ║
// ║  ★ 中点采样法 + 可配波特率 + 复位安全 + TX/RX 双向监听                         ║
// ╚══════════════════════════════════════════════════════════════════════════════════╝
//
// 【UVM 角色 — uvm_agent (PASSIVE)】
//   仅包含 uart_monitor——监听 SoC 的 UART TX 和 RX 引脚，解析字节流。
//   和 gpio_agent 一样，本 Agent 是**被动**的——不驱动任何信号。
//
// 【UART 协议背景】
//   UART（Universal Asynchronous Receiver/Transmitter）是一种串行通信协议。
//   一帧数据 = 1 起始位(0) + 8 数据位(LSB first) + 1 停止位(1)。
//   空闲时线路保持高电平(1)，起始位拉低(0)表示一帧开始。
//
//   波特率 = 每秒传输的 bit 数（bps）。115200 bps 是常见速率。
//   本 Monitor 的 baud_div = 868（100MHz / 115200 ≈ 868 周期/bit）
//
// 【中点采样法（Midpoint Sampling）】
//   为什么要在"bit 中点"采样？
//     UART 信号可能有抖动和上升/下降沿的斜坡。bit 边界处信号不稳定，
//     中点处信号已经稳定，采样最可靠。
//
//   采样流程：
//     1. 检测起始位（negedge）→ 表明一帧开始
//     2. 等 half_baud 个周期 → 到达起始位的中点 → 确认起始位确实是 0
//     3. 等 baud_div 个周期 → 到达第 1 数据位中点 → 采样 bit 0
//     4. ...重复 8 次 → 采样完 8 个数据位
//     5. 等 baud_div 个周期 → 到达停止位中点 → 确认停止位是 1
//
// 【复位安全】
//   在每个等待周期之前检查 vif.resetn。如果复位发生 → 立即退出采样任务。
//   这防止了"复位时卡在采样循环里"的死锁。
//
// 【uart_byte_t 结构体】
//   data[7:0]      — 8-bit 数据字节
//   framing_error   — 帧错误（停止位不为 1 或起始位不为 0）
//   timestamp       — 收到此字节的时间戳
//
//   ★ 为什么用 typedef struct 而非 class？
//     struct 是值类型——赋值是复制（不是引用）。简单数据用 struct 更轻量。
//     uvm_sequence_item（class）更适合需要 constraint/randomize 的事务。
// ═══════════════════════════════════════════════════════════════════════════════

`include "uvm_macros.svh"
import uvm_pkg::*;

// ★ UART 字节结构体（值类型，轻量）★
typedef struct {
    bit [7:0] data;          // 收到的 8-bit 数据
    bit       framing_error; // 帧错误标志（1=停止位不是 1）
    time      timestamp;     // 收到时间戳
} uart_byte_t;

// ═══════════════════════════════════════════════════════════════════════════════
//  uart_monitor — UART 信号采样器
//
//  ★ 双任务并行：
//    monitor_tx() 监听 uart_tx 引脚的 negedge（SoC 发送数据）
//    monitor_rx() 监听 uart_rx 引脚的 negedge（外部发送数据）
//    两个任务同时运行——UART 是全双工的。
//
//  ★ 波特率可配置：
//    baud_div = 100MHz / baud_rate（默认 115200 → 868 周期/bit）
//    half_baud = baud_div / 2（中点偏移量）
//    可通过 config_db 覆盖——测试不同波特率时无需修改 Agent 代码。
// ═══════════════════════════════════════════════════════════════════════════════
class uart_monitor extends uvm_monitor;
    `uvm_component_utils(uart_monitor)
    virtual riscv_if vif;
    uvm_analysis_port#(uart_byte_t) ap;

    int baud_div  = 868;               // ★ 默认 115200 bps @ 100MHz
    int half_baud = 434;               // ★ 半 bit 周期（起始位中点偏移）

    function new(string n, uvm_component p);
        super.new(n, p);
        ap = new("ap", this);
    endfunction

    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db#(virtual riscv_if)::get(this, "", "vif", vif))
            `uvm_fatal("NOVIF", "")
        void'(uvm_config_db#(int)::get(this, "", "baud_div", baud_div));
        half_baud = baud_div / 2;
    endfunction

    virtual task run_phase(uvm_phase phase);
        fork
            monitor_tx();              // ★ 监听 TX（SoC → 外部）
            monitor_rx();              // ★ 监听 RX（外部 → SoC）
        join
    endtask

    // ═══════════════════════════════════════════════════════════════════════════
    //  sample_uart_byte() — 中点采样一个 UART 字节
    //
    //  ★ 采样算法：
    //    Step 1: 等 half_baud 周期 → 到达起始位中点 → 确认起始位=0
    //    Step 2: 等 baud_div 周期 → 采样 8 个数据位
    //    Step 3: 等 baud_div 周期 → 确认停止位=1
    //
    //  ★ 复位安全：每个等待步骤都检查 resetn，复位时立即返回
    //  ★ framing_error：起始位不是 0 或停止位不是 1 → 帧错误
    // ═══════════════════════════════════════════════════════════════════════════
    task sample_uart_byte(input logic sig, output uart_byte_t b);
        b.timestamp = $time;
        // ── 到达起始位中点 ──
        repeat (half_baud) @(posedge vif.clk);
        if (!vif.resetn) begin {b.data, b.framing_error} = 0; return; end  // ★ 复位退出
        if (sig != 0) begin b.framing_error = 1; return; end                // ★ 起始位错误
        b.data = 0;
        // ── 采样 8 个数据位（LSB first）──
        for (int i = 0; i < 8; i++) begin
            repeat (baud_div) @(posedge vif.clk);
            if (!vif.resetn) return;                                        // ★ 复位退出
            b.data[i] = sig;
        end
        // ── 检查停止位 ──
        repeat (baud_div) @(posedge vif.clk);
        if (!vif.resetn) return;
        b.framing_error = (sig != 1);   // ★ 停止位应 = 1（高电平）
    endtask

    task monitor_tx();
        forever begin
            uart_byte_t b;
            @(negedge vif.uart_tx);    // ★ 等 TX 起始位（negedge = 起始位开始）
            if (!vif.resetn) continue;  // ★ 复位毛刺过滤
            sample_uart_byte(vif.uart_tx, b);
            if (!vif.resetn) continue;
            ap.write(b);               // ★ 广播采样结果
        end
    endtask

    task monitor_rx();
        forever begin
            uart_byte_t b;
            @(negedge vif.uart_rx);    // ★ 等 RX 起始位
            if (!vif.resetn) continue;
            sample_uart_byte(vif.uart_rx, b);
            if (!vif.resetn) continue;
            ap.write(b);
        end
    endtask

endclass

// ═══════════════════════════════════════════════════════════════════════════════
//  uart_agent — UART 被动 Agent 容器
// ═══════════════════════════════════════════════════════════════════════════════
class uart_agent extends uvm_agent;
    `uvm_component_utils(uart_agent)
    uvm_analysis_port#(uart_byte_t) ap;
    uart_monitor monitor;

    function new(string n, uvm_component p);
        super.new(n, p);
        is_active = UVM_PASSIVE;       // ★ 被动模式
    endfunction

    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        monitor = uart_monitor::type_id::create("monitor", this);
        ap = new("ap", this);
    endfunction

    virtual function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
        monitor.ap.connect(ap);
    endfunction

endclass
