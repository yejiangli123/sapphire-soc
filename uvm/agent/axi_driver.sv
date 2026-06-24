// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  axi_driver.sv — AXI4-Lite 主端驱动（Master Driver）                          ║
// ║  ★ 支持 5 通道握手 + wstrb 生成 + 复位安全中止                                ║
// ╚══════════════════════════════════════════════════════════════════════════════════╝
//
// 【UVM 角色 — uvm_driver】
//   Driver 是 UVM 中唯一"主动驱动 DUT 信号"的组件。
//   工作流程：从 sequencer 取 transaction → 按协议时序驱动 interface 信号 → 通知完成。
//
//   在 AXI4-Lite 验证环境中，本 Driver 扮演 AXI **Master** 角色：
//     - 发起写事务：驱动 AW(地址)→W(数据)→收 B(响应)
//     - 发起读事务：驱动 AR(地址)→收 R(数据+响应)
//
// 【为什么 Driver 和 Sequencer 要分开】
//   ┌──────────┐       ┌──────────┐       ┌──────────┐
//   │ Sequence │──────→│Sequencer │──────→│  Driver  │
//   │(场景描述)│       │(调度管理)│       │(信号驱动)│
//   └──────────┘       └──────────┘       └──────────┘
//   分开的好处：
//     1. Driver 只关心"时序"（valid 什么时候拉高，等 ready 几拍）
//     2. Sequence 只关心"场景"（发多少包、什么地址范围、什么数据模式）
//     3. 换测试场景只改 Sequence，不改 Driver → 复用性
//
// 【AXI4-Lite 协议时序要点】
//   写事务（必须按顺序）：
//     Step 1: AW 握手（awvalid && awready）
//     Step 2: W  握手（wvalid && wready）—— 可以和 AW 同时或之后
//     Step 3: B  握手（bvalid && bready）—— 从设备返回写响应
//
//   读事务：
//     Step 1: AR 握手（arvalid && arready）
//     Step 2: R  握手（rvalid && rready）—— 从设备返回读数据
//
//   Valid/Ready 规则：
//     - Valid 一方拉高 valid 后**必须保持**直到握手完成（不能等 ready 才拉 valid）
//     - Ready 一方可以在 valid 之前或之后拉高 ready（可以等 valid）
//     - 握手 = valid && ready 同时为 1 的那一拍
//
// 【复位安全设计】
//   本 Driver 实现了"复位检测"——在等待握手的过程中，如果复位突然拉低：
//     - 立即中止当前传输（join_any + disable fork）
//     - 清零所有 AXI 输出（避免 protocol violation）
//     - 标记事务 resp=2'b11（异常响应）
//   这保证了 UVM 环境在随机复位场景下不会死锁。
//
// 【Clock Block（clocking block）的使用】
//   本 Driver 通过 vif.driver_cb 驱动信号（而非直接 vif.signal）：
//     - clocking block 保证所有驱动信号使用非阻塞赋值（<=）
//     - 解决了 Verilog 中"组合逻辑驱动时序信号"的竞争风险
//     - VCS 2018 要求 clocking block 内的信号必须用 <=
// ═══════════════════════════════════════════════════════════════════════════════

`include "uvm_macros.svh"
import uvm_pkg::*;

class axi_driver extends uvm_driver#(axi_transaction);
    // ★ 参数化的 uvm_driver 基类指定了 transaction 类型为 axi_transaction
    `uvm_component_utils(axi_driver)

    virtual riscv_if vif;  // ★ 通过 config_db 传入的 virtual interface

    function new(string n, uvm_component p);
        super.new(n, p);
    endfunction

    // ── build_phase：从 config_db 获取 virtual interface ──
    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db#(virtual riscv_if)::get(this, "", "vif", vif))
            `uvm_fatal("NOVIF", "Virtual interface not found")
    endfunction

    // ═══════════════════════════════════════════════════════════════════════
    //  clear_axi_outputs() — 清零所有 AXI 输出
    //
    //  目的：复位时确保所有 AXI 信号回到初始状态，防止残留值误触发。
    //  使用 clocking block 的 <= 驱动（非阻塞赋值 + 同步驱动）。
    // ═══════════════════════════════════════════════════════════════════════
    task clear_axi_outputs();
        vif.driver_cb.axi_awaddr  <= 32'h0;
        vif.driver_cb.axi_awsize  <= 3'd2;          // 默认 4 字节
        vif.driver_cb.axi_awvalid <= 1'b0;
        vif.driver_cb.axi_wdata   <= 32'h0;
        vif.driver_cb.axi_wstrb   <= 4'b0;
        vif.driver_cb.axi_wvalid  <= 1'b0;
        vif.driver_cb.axi_bready  <= 1'b0;
        vif.driver_cb.axi_araddr  <= 32'h0;
        vif.driver_cb.axi_arsize  <= 3'd2;
        vif.driver_cb.axi_arvalid <= 1'b0;
        vif.driver_cb.axi_rready  <= 1'b0;
    endtask

    // ═══════════════════════════════════════════════════════════════════════
    //  run_phase — Driver 主循环
    //
    //  ★ 标准 UVM Driver 模式（get_next_item / item_done 握手机制）:
    //    1. 复位期间阻塞，不取新事务（避免在 DUT 未就绪时发请求）
    //    2. get_next_item(req) — 从 sequencer 取一个 transaction（阻塞）
    //    3. drive_write() 或 drive_read() — 按协议驱动信号
    //    4. item_done() — 通知 sequencer 本 transaction 完成
    //    5. 循环回到步骤 2
    //
    //  ★ get_next_item 和 try_next_item 的区别：
    //    get_next_item 会阻塞等待直到 sequencer 有 item
    //    try_next_item 不阻塞，无 item 时立即返回 null（用于非阻塞轮询）
    //    本 Driver 使用 get_next_item——因为 Driver 的职责就是"有请求就发"，
    //    没有请求时等待是合理的。
    // ═══════════════════════════════════════════════════════════════════════
    virtual task run_phase(uvm_phase phase);
        clear_axi_outputs();          // 上电初始清零
        forever begin
            // ★ 复位期间阻塞，不取事务
            if (!vif.resetn) @(posedge vif.resetn);
            // ★ 标准握手：取 item → 执行 → 完成
            seq_item_port.get_next_item(req);
            req.start_time = $time;
            req.done   = 1'b0;
            req.resp   = 2'b00;       // 默认正常响应（OKAY）
            if (req.kind) drive_write(req);
            else          drive_read(req);
            req.end_time = $time;
            seq_item_port.item_done();
        end
    endtask

    // ═══════════════════════════════════════════════════════════════════════
    //  drive_write() — 写事务（3 阶段：AW → W → B）+ 复位安全
    //
    //  ★ fork...join_any + disable 模式（超时/复位安全）:
    //    两个并行线程：
    //      线程1（正常传输）：按 AXI 协议驱动
    //      线程2（监护线程）：监听复位 → 一旦复位，立即中止线程1
    //    join_any：谁先完成听谁的
    //    disable write_fork：确保另一个线程也被杀死
    //
    //  ★ wstrb（字节写掩码）生成规则：
    //    size=0(1B)：掩码 = 4'b0001 << addr[1:0] → 单字节掩码移位到目标 lane
    //      例：addr[1:0]=2 → 掩码 4'b0100（写第 3 字节）
    //    size=1(2B)：掩码 = addr[1]?4'b1100:4'b0011 → 半字在高/低两字节
    //    size=2(4B)：掩码 = 4'b1111 → 全字写
    //    ★ 注意：wstrb 生成逻辑必须与 DUT 中 store_unit 的实现一致
    // ═══════════════════════════════════════════════════════════════════════
    task drive_write(axi_transaction tr);
        fork : write_fork
            // ── 线程 1：正常 AXI 写事务 ──
            begin
                @(vif.driver_cb);
                // ★ AW 通道：写地址（与 W 通道同时驱动）★
                vif.driver_cb.axi_awaddr  <= tr.addr;
                vif.driver_cb.axi_awsize  <= tr.size;
                vif.driver_cb.axi_awvalid <= 1'b1;
                // ★ W 通道：写数据 + 写字节掩码 ★
                vif.driver_cb.axi_wdata   <= tr.data;
                // ★ wstrb 生成：根据 size 和 addr_low 计算
                tr.wstrb = (tr.size == 0) ? (4'b0001 << tr.addr[1:0]) :
                           (tr.size == 1) ? (tr.addr[1] ? 4'b1100 : 4'b0011) :
                                            4'b1111;
                vif.driver_cb.axi_wstrb   <= tr.wstrb;
                vif.driver_cb.axi_wvalid  <= 1'b1;

                // ★ AW 和 W 并行等待握手 ★
                fork
                    begin
                        do @(vif.driver_cb); while (!vif.driver_cb.axi_awready);
                        vif.driver_cb.axi_awvalid <= 1'b0;
                    end
                    begin
                        do @(vif.driver_cb); while (!vif.driver_cb.axi_wready);
                        vif.driver_cb.axi_wvalid <= 1'b0;
                    end
                join

                // ★ B 通道：收写响应 ★
                vif.driver_cb.axi_bready <= 1'b1;
                do @(vif.driver_cb); while (!vif.driver_cb.axi_bvalid);
                vif.driver_cb.axi_bready <= 1'b0;
                tr.resp = vif.driver_cb.axi_bresp;
                tr.done = 1'b1;
            end
            // ── 线程 2：复位监护 ──
            begin
                if (!vif.resetn) ;                       // 已复位→立即触发
                else @(negedge vif.resetn);              // 等下次复位
                clear_axi_outputs();
                tr.resp = 2'b11;                         // ★ 复位中止：标记异常
                tr.done = 1'b1;
                `uvm_warning("DRV", "Write aborted by reset")
            end
        join_any
        disable write_fork;                              // ★ 确保两个线程都终止
    endtask

    // ═══════════════════════════════════════════════════════════════════════
    //  drive_read() — 读事务（2 阶段：AR → R）+ 复位安全
    //
    //  设计同 drive_write，但少了 W 通道，R 通道取代 B 通道。
    //  读数据（rdata）和读响应（rresp）同时返回。
    // ═══════════════════════════════════════════════════════════════════════
    task drive_read(axi_transaction tr);
        fork : read_fork
            // ── 线程 1：正常 AXI 读事务 ──
            begin
                @(vif.driver_cb);
                // ★ AR 通道：读地址 ★
                vif.driver_cb.axi_araddr  <= tr.addr;
                vif.driver_cb.axi_arsize  <= tr.size;
                vif.driver_cb.axi_arvalid <= 1'b1;
                do @(vif.driver_cb); while (!vif.driver_cb.axi_arready);
                vif.driver_cb.axi_arvalid <= 1'b0;

                // ★ R 通道：收读数据+响应 ★
                vif.driver_cb.axi_rready <= 1'b1;
                do @(vif.driver_cb); while (!vif.driver_cb.axi_rvalid);
                tr.data = vif.driver_cb.axi_rdata;
                tr.resp = vif.driver_cb.axi_rresp;
                vif.driver_cb.axi_rready <= 1'b0;
                tr.done = 1'b1;
            end
            // ── 线程 2：复位监护 ──
            begin
                if (!vif.resetn) ;
                else @(negedge vif.resetn);
                clear_axi_outputs();
                tr.resp = 2'b11;
                tr.done = 1'b1;
                `uvm_warning("DRV", "Read aborted by reset")
            end
        join_any
        disable read_fork;
    endtask

endclass
