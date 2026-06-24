// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  axi_monitor.sv — AXI4-Lite 总线监测器（Passive Monitor）                     ║
// ║  ★ 5 通道独立监测 + wstrb 校验 + 复位安全 + 多拍防竞争                        ║
// ╚══════════════════════════════════════════════════════════════════════════════════╝
//
// 【UVM 角色 — uvm_monitor】
//   Monitor 是**被动组件**——只采样信号，不驱动。通过 analysis_port 把打包好的
//   transaction 广播给 Scoreboard 和 Coverage 组件。
//
//   Monitor 的关键职责：
//     1. 在 AXI 接口上检测事务（写：AW+W+B 三次握手，读：AR+R 两次握手）
//     2. 组装完整的 axi_transaction（含 wstrb 实际值、响应码、时间戳）
//     3. 校验 wstrb 合法性（非法写掩码立即报错）
//     4. 通过 analysis_port.write(tr) 广播给下游
//
// 【"被动"意味着什么】
//   - Monitor 不调用 get_next_item（那不是它的职责）
//   - Monitor 不驱动任何 DUT 信号
//   - Monitor 的 analysis_port 是**广播**模式——可以有多个 subscriber 同时接收
//   - 即使没有 Scoreboard 连接，Monitor 也能独立工作
//
// 【多拍防竞争设计（busy 标志）】
//   AXI 5 个通道可以同时活动——但 Monitor 只有一个 run_phase 线程。
//   如果 AW 握手和 AR 握手发生在同一拍（或连续拍），Monitor 需要同时处理两个事务。
//
//   本 Monitor 的设计：
//     ★ wr_busy/rd_busy 标志：防止同一通道重复触发（AW 握手后还没处理完的
//        情况下又来了第二个 AW——rare but possible with pipelining）
//     ★ fork...join_none：每个检测到的事务 fork 出独立线程，互不阻塞
//     ★ 复位时 kill 所有线程 + 清零 busy 标志
//
// 【复位安全】
//   在每个等待握手的 do-while 循环中检查 resetn。如果复位发生：
//     - 立即释放 busy 标志（其他事务可以正常启动）
//     - return 中止当前检测（不产生垃圾 transaction）
//
// 【wstrb 校验规则】
//   AXI4-Lite 允许的 wstrb 组合：
//     - 单字节：4'b0001, 0010, 0100, 1000
//     - 双字节：4'b0011（低 2B）, 4'b1100（高 2B）
//     - 全字：4'b1111
//   禁止的组合：4'b0000（全不写——无意义）, 4'b0101（不连续——非法）
// ═══════════════════════════════════════════════════════════════════════════════

`include "uvm_macros.svh"
import uvm_pkg::*;

class axi_monitor extends uvm_monitor;
    `uvm_component_utils(axi_monitor)

    virtual riscv_if vif;
    uvm_analysis_port #(axi_transaction) ap;   // ★ 广播端口：所有 subscriber 都能收到
    process wr_proc, rd_proc;                   // ★ 写/读线程句柄（用于 reset 时 kill）
    bit     wr_busy, rd_busy;                   // ★ 忙标志：防止同一通道重复触发

    function new(string n, uvm_component p);
        super.new(n, p);
    endfunction

    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db #(virtual riscv_if)::get(this, "", "vif", vif))
            `uvm_fatal("MON/NOVIF", "Virtual interface not found")
        ap = new("ap", this);
    endfunction

    // ═══════════════════════════════════════════════════════════════════════
    //  valid_wstrb() — 检查 wstrb 是否为合法组合
    //  ★ 为什么需要显式校验？DUT 可能因 bug 输出非法 wstrb（如 4'b1010）。
    //     Monitor 应尽早发现问题，不要在 Scoreboard 才发现协议层违规。
    // ═══════════════════════════════════════════════════════════════════════
    function bit valid_wstrb(bit [3:0] w);
        case (w) 4'b0001,4'b0010,4'b0100,4'b1000,4'b0011,4'b1100,4'b1111: return 1;
        default: return 0; endcase
    endfunction

    // ═══════════════════════════════════════════════════════════════════════
    //  expected_wstrb() — 根据 size 和 addr_low 计算期望的 wstrb
    //
    //  size=0(1B): wstrb = 4'b0001 << addr_low → 单字节，移位到目标 byte lane
    //  size=1(2B): wstrb = addr[1]?4'b1100:4'b0011 → 半字在高/低两字节
    //  size=2(4B): wstrb = 4'b1111 → 全字
    //  ★ 期望值和实际值比对——如果 DUT store_unit 有 bug，这里立刻暴露
    // ═══════════════════════════════════════════════════════════════════════
    function bit [3:0] expected_wstrb(bit [2:0] sz, bit [1:0] off);
        case (sz) 0: return 4'b0001 << off;
        1: return off[1] ? 4'b1100 : 4'b0011;
        2: return 4'b1111; default: return 4'b0; endcase
    endfunction

    // ★ 安全 kill 线程：先 kill 再置 null（避免 null 调用）
    task kill_proc(inout process p);
        if (p) begin p.kill(); p = null; end
    endtask

    // ═══════════════════════════════════════════════════════════════════════
    //  run_phase — 主监测循环
    //
    //  每拍检查：
    //    1. resetn=0 → kill 所有线程，清零 busy 标志
    //    2. AW 握手（awvalid && awready）→ 启动写检测线程
    //    3. AR 握手（arvalid && arready）→ 启动读检测线程
    //
    //  ★ 为什么用 fork...join_none 而不是直接调用 task？
    //    因为读写可能同时发生 → 需要两个并行线程分别跟踪 W/B 和 R 通道。
    //    join_none 确保 Monitor 主循环不阻塞，可以继续检测新事务。
    // ═══════════════════════════════════════════════════════════════════════
    virtual task run_phase(uvm_phase phase);
        forever begin
            @(vif.monitor_cb);
            // ★ 复位：终止所有进行中线程 ★
            if (!vif.resetn) begin
                kill_proc(wr_proc); wr_busy = 1'b0;
                kill_proc(rd_proc); rd_busy = 1'b0;
                continue;
            end
            // ★ 先置 busy 标志（原子性），再 fork ★
            //    防止同一拍检测到两次握手导致竞争
            if (vif.monitor_cb.axi_awvalid && vif.monitor_cb.axi_awready
                && !wr_busy) begin
                wr_busy = 1'b1;
                fork begin wr_proc = process::self(); detect_write(); end join_none
            end
            if (vif.monitor_cb.axi_arvalid && vif.monitor_cb.axi_arready
                && !rd_busy) begin
                rd_busy = 1'b1;
                fork begin rd_proc = process::self(); detect_read();  end join_none
            end
        end
    endtask

    // ═══════════════════════════════════════════════════════════════════════
    //  detect_write() — 跟踪一次完整的 AXI 写事务（W 通道 + B 通道）
    //
    //  ★ 检测流程：
    //    AW 握手（已在外层检测到） → 等 W 握手 → 捕获 wdata/wstrb
    //    → 校验 wstrb → 等 B 握手 → 捕获 bresp → ap.write(tr)
    //
    //  ★ 每步都检查复位：防止在等待 W/B 时系统复位导致死锁
    // ═══════════════════════════════════════════════════════════════════════
    task detect_write();
        reg [3:0] exp;                                   // ★ VCS 2018: task 级局部变量声明
        axi_transaction tr = axi_transaction::type_id::create("wr_tr");
        tr.kind  = 1;                                    // 写事务
        tr.addr  = vif.monitor_cb.axi_awaddr;            // AW 握手时的地址
        // ★ awsize 可能为 X（上电未初始化），安全处理
        tr.size  = (^vif.monitor_cb.axi_awsize === 1'bx) ? 3'd2 : vif.monitor_cb.axi_awsize;
        if (tr.size > 3'd2) `uvm_error("MON", $sformatf("Illegal awsize=%0d", tr.size))
        tr.start_time = $time;

        // ── 等 W 握手（WVALID && WREADY）──
        do begin
            @(vif.monitor_cb);
            if (!vif.resetn) begin wr_busy = 1'b0; wr_proc = null;
                `uvm_info("MON", "Write aborted by reset (W phase)", UVM_MEDIUM) return; end
        end while (!(vif.monitor_cb.axi_wvalid && vif.monitor_cb.axi_wready));
        tr.data = vif.monitor_cb.axi_wdata;              // ★ 捕获实际写数据
        tr.wstrb = vif.monitor_cb.axi_wstrb;             // ★ 捕获实际写掩码

        // ★ wstrb 合法性校验 ★
        if (!valid_wstrb(vif.monitor_cb.axi_wstrb))
            `uvm_error("MON", $sformatf("Illegal wstrb=0b%04b", vif.monitor_cb.axi_wstrb))
        else begin
            exp = expected_wstrb(tr.size, tr.addr[1:0]);
            if (vif.monitor_cb.axi_wstrb != exp)
                `uvm_error("MON", $sformatf("wstrb mismatch: got=0b%04b exp=0b%04b",
                           vif.monitor_cb.axi_wstrb, exp))
        end

        // ── 等 B 握手（BVALID && BREADY）──
        do begin
            @(vif.monitor_cb);
            if (!vif.resetn) begin wr_busy = 1'b0; wr_proc = null;
                `uvm_info("MON", "Write aborted by reset (B phase)", UVM_MEDIUM) return; end
        end while (!(vif.monitor_cb.axi_bvalid && vif.monitor_cb.axi_bready));
        tr.resp = vif.monitor_cb.axi_bresp;              // ★ 捕获写响应
        tr.done = 1'b1;
        tr.end_time = $time;
        ap.write(tr);                                    // ★ 广播给 Scoreboard/Coverage
        wr_busy = 1'b0; wr_proc = null;                  // ★ 释放 busy
    endtask

    // ═══════════════════════════════════════════════════════════════════════
    //  detect_read() — 跟踪一次完整的 AXI 读事务（R 通道）
    //
    //  读比写简单：AR 握手后直接等 R 握手，一次返回 data+resp。
    // ═══════════════════════════════════════════════════════════════════════
    task detect_read();
        axi_transaction tr = axi_transaction::type_id::create("rd_tr");
        tr.kind  = 0;
        tr.addr  = vif.monitor_cb.axi_araddr;
        tr.size  = (^vif.monitor_cb.axi_arsize === 1'bx) ? 3'd2 : vif.monitor_cb.axi_arsize;
        if (tr.size > 3'd2) `uvm_error("MON", $sformatf("Illegal arsize=%0d", tr.size))
        tr.start_time = $time;

        // ── 等 R 握手（RVALID && RREADY）──
        do begin
            @(vif.monitor_cb);
            if (!vif.resetn) begin rd_busy = 1'b0; rd_proc = null;
                `uvm_info("MON", "Read aborted by reset (R phase)", UVM_MEDIUM) return; end
        end while (!(vif.monitor_cb.axi_rvalid && vif.monitor_cb.axi_rready));
        tr.data = vif.monitor_cb.axi_rdata;
        tr.resp = vif.monitor_cb.axi_rresp;
        tr.done = 1'b1;
        tr.end_time = $time;
        ap.write(tr);
        rd_busy = 1'b0; rd_proc = null;
    endtask

endclass
