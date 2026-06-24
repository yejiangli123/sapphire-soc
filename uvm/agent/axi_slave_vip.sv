// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  axi_slave_vip.sv — AXI4-Lite 从设备 VIP（Verification IP）                   ║
// ║  ★ 内存后端 + 可配置 ready 延迟 + SLVERR 注入                                   ║
// ╚══════════════════════════════════════════════════════════════════════════════════╝
//
// 【UVM 角色 — Slave VIP（被动响应器）】
//   Slave VIP 模拟一个 AXI4-Lite 从设备——它不主动发起事务，而是响应 Master 的请求。
//
//   在 Sapphire SoC 验证中，本 VIP 的作用是：
//     · 接收 RISC-V core 通过 AXI 总线发出的读写请求（模拟外设行为）
//     · 维护内部内存镜像（mem[]），读操作返回存储值
//     · 支持可配置的 ready 延迟（aw/w/ar 通道各自独立）
//     · 支持 SLVERR 注入（错误测试用）
//
// 【为什么需要 Slave VIP】
//   SoC 的 AXI 总线需要"对端"来响应请求——不能空着（空着 DUT 会挂起）。
//   Slave VIP 提供了这个"对端"：Master 发写请求 → VIP 接收并存储；
//   Master 发读请求 → VIP 返回之前存储的数据。
//
//   ★ 关键区别：
//     axi_driver（Master）：主动发起事务——发 AW/W/AR，收 B/R
//     axi_slave_driver（Slave）：被动响应——收 AW/W/AR，发 B/R
//
// 【可配置特性】
//   aw_ready_delay / w_ready_delay / ar_ready_delay：
//     模拟从设备的响应延迟（0 = 零等待，N = 等 N 拍才拉 ready）
//     用于背压（backpressure）测试——验证 Master 在从设备忙时的行为。
//
//   inject_slverr = 1：
//     从设备返回 SLVERR(2'b10) 而非 OKAY(2'b00)
//     用于错误处理测试——验证 Master/SW 能否正确处理从设备错误。
//
// 【设计简化 — 无 clocking block】
//   本 VIP 使用 raw 信号检测（vif.axi_awvalid），而非 monitor_cb。
//   原因：响应者需要实时检测 Master 的 valid，clocking block 引入的延迟
//   可能错过有效的握手拍（特别是 aw_ready_delay=0 的零等待模式）。
//   代价：信号可能在 delta cycle 中出现竞争——但在 VCS 2018 中已验证无问题。
//
// 【面试要点】
//   Q: 什么是 VIP（Verification IP）？
//   A: VIP 是可复用的验证组件，封装了协议的正确行为。
//      Master VIP 能主动发起事务；Slave VIP 能被动响应。
//      它还支持错误注入和配置选项（如 ready 延迟）。
// ═══════════════════════════════════════════════════════════════════════════════

`include "uvm_macros.svh"
import uvm_pkg::*;

// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  axi_slave_driver — AXI4-Lite Slave 响应器                                    ║
// ║  ★ 监听 AW/AR → 内存读写 → 返回 B/R                                         ║
// ╚══════════════════════════════════════════════════════════════════════════════╝
class axi_slave_driver extends uvm_driver#(axi_transaction);
    `uvm_component_utils(axi_slave_driver)

    virtual riscv_if vif;

    // ★ 内部内存镜像（关联数组：只存储实际访问的地址）
    bit [31:0] mem[bit [31:0]];

    // ★ 可配置的 ready 延迟（背压测试用）
    int aw_ready_delay = 0;    // 写地址通道：等 N 拍再拉 awready
    int w_ready_delay  = 0;    // 写数据通道：等 N 拍再拉 wready
    int ar_ready_delay = 0;    // 读地址通道：等 N 拍再拉 arready

    // ★ SLVERR 注入开关（错误测试用）
    bit inject_slverr = 0;

    function new(string n, uvm_component p);
        super.new(n, p);
    endfunction

    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db#(virtual riscv_if)::get(this, "", "vif", vif))
            `uvm_fatal("NOVIF", "")
    endfunction

    // ═══════════════════════════════════════════════════════════════════════
    //  run_phase — fork 两个并行线程
    //
    //  respond_wr() 和 respond_rd() 同时运行——因为 AXI 5 个通道独立，
    //  写事务和读事务可以同时进行（AXI full 协议支持 outstanding）。
    //  AXI4-Lite 虽然不支持 outstanding，但读写事务仍然可以并行处理。
    // ═══════════════════════════════════════════════════════════════════════
    virtual task run_phase(uvm_phase phase);
        fork
            respond_wr();    // ★ 写通道响应（AW→W→B）
            respond_rd();    // ★ 读通道响应（AR→R）
        join
    endtask

    task delay_ready(int d);
        repeat (d) @(posedge vif.clk);
    endtask

    // ═══════════════════════════════════════════════════════════════════════
    //  respond_wr() — 写通道响应循环
    //
    //  流程：
    //    1. 等 AWVALID → 延时 N 拍 → 拉 awready（AW 握手完成）
    //    2. 等 WVALID → 延时 N 拍 → 读旧值 + 按 be 更新字节 → 拉 wready
    //    3. 拉 bvalid + bresp → 等 bready（B 握手完成）
    //
    //  ★ 字节写入：be（byte-enable）= wstrb，每位控制 1 字节
    //    先读旧值 → 按 be 更新对应字节 → 写回（模拟真实存储器行为）
    // ═══════════════════════════════════════════════════════════════════════
    task respond_wr();
        reg [31:0] a; reg [3:0] be; reg [31:0] w;  // ★ VCS 2018: task 级声明
        forever begin
            @(posedge vif.clk);
            // ★ 复位：清零输出 + 清空内存 ★
            if (!vif.resetn) begin
                vif.axi_awready <= 0;
                vif.axi_wready  <= 0;
                vif.axi_bvalid  <= 0;
                mem.delete();
                continue;
            end

            // ★ AW 通道响应 ★
            if (vif.axi_awvalid && !vif.axi_awready) begin
                a = vif.axi_awaddr & 32'hFFFFFFFC;   // ★ 字对齐地址
                delay_ready(aw_ready_delay);          // ★ 可配置延迟
                vif.axi_awready <= 1;
                @(posedge vif.clk);
                vif.axi_awready <= 0;

                // ★ W 通道响应（等 WVALID）★
                //   注意：AXI4-Lite 中 AW 和 W 可以同时或先后
                while (!vif.axi_wvalid) @(posedge vif.clk);
                delay_ready(w_ready_delay);
                be = vif.axi_wstrb;
                w  = mem.exists(a) ? mem[a] : 32'h0;  // ★ 读旧值
                // ★ 逐字节更新 ★
                if (be[0]) w[7:0]   = vif.axi_wdata[7:0];
                if (be[1]) w[15:8]  = vif.axi_wdata[15:8];
                if (be[2]) w[23:16] = vif.axi_wdata[23:16];
                if (be[3]) w[31:24] = vif.axi_wdata[31:24];
                mem[a] = w;
                vif.axi_wready <= 1;
                @(posedge vif.clk);
                vif.axi_wready <= 0;

                // ★ B 通道响应 ★
                vif.axi_bvalid <= 1;
                vif.axi_bresp  <= inject_slverr ? 2'b10 : 2'b00;  // ★ 错误注入
                while (!vif.axi_bready) @(posedge vif.clk);
                vif.axi_bvalid <= 1'b0;
            end
        end
    endtask

    // ═══════════════════════════════════════════════════════════════════════
    //  respond_rd() — 读通道响应循环
    //
    //  流程：等 ARVALID → 延时 → 拉 arready → 拉 rvalid + rdata → 等 rready
    //
    //  ★ 读数据来源：mem[a]（之前写操作存入的值）
    //    如果地址从未被写过 → 返回 0xDEAD_BEEF（标识未初始化访问）
    // ═══════════════════════════════════════════════════════════════════════
    task respond_rd();
        reg [31:0] a;                                    // ★ VCS 2018
        forever begin
            @(posedge vif.clk);
            if (!vif.resetn) begin
                vif.axi_arready <= 0;
                vif.axi_rvalid  <= 0;
                mem.delete();
                continue;
            end
            if (vif.axi_arvalid && !vif.axi_arready) begin
                a = vif.axi_araddr & 32'hFFFFFFFC;       // ★ 字对齐
                delay_ready(ar_ready_delay);
                vif.axi_arready <= 1;
                @(posedge vif.clk);
                vif.axi_arready <= 0;

                vif.axi_rvalid <= 1;
                vif.axi_rdata  <= mem.exists(a) ? mem[a] : 32'hDEAD_BEEF;
                vif.axi_rresp  <= inject_slverr ? 2'b10 : 2'b00;
                while (!vif.axi_rready) @(posedge vif.clk);
                vif.axi_rvalid <= 1'b0;
            end
        end
    endtask

endclass


// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  axi_slave_agent — Slave VIP 的 Agent 容器                                    ║
// ║  ★ 仅包含 Slave Driver（无 Monitor / Sequencer）                              ║
// ╚══════════════════════════════════════════════════════════════════════════════╝
//
//  ★ 为什么 Slave Agent 不需要 Monitor？
//    axi_monitor（在 axi_agent 中）已经监测了 AXI 总线上所有信号。
//    Slave 的响应也会被 axi_monitor 捕获——无需重复。
//
//  ★ 为什么 Slave Agent 不需要 Sequencer？
//    Slave 不主动发起事务——它只被动响应 Master 的请求。
//    不需要 Sequence 来告诉 Slave "什么时候发响应"——收到 AW/AR 就响应。
class axi_slave_agent extends uvm_agent;
    `uvm_component_utils(axi_slave_agent)

    axi_slave_driver driver;           // ★ 唯一的子组件：Slave 响应器

    function new(string n, uvm_component p);
        super.new(n, p);
    endfunction

    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        driver = axi_slave_driver::type_id::create("driver", this);
    endfunction

endclass
