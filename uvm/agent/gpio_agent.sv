// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  gpio_agent.sv — GPIO 被动 Agent（Passive Agent）                              ║
// ║  ★ 只监控 AXI 总线上的 GPIO 地址空间访问，不驱动任何信号                        ║
// ╚══════════════════════════════════════════════════════════════════════════════════╝
//
// 【UVM 角色 — uvm_agent (PASSIVE)】
//   本 Agent 是**被动**的——它只包含 Monitor，不包含 Driver。
//   它通过监控 AXI 总线来"窃听"CPU 对 GPIO 外设的读写操作。
//
//   ★ 与 axi_agent 的重要区别：
//     axi_agent（ACTIVE）：包含 Driver，能主动发起 AXI 事务到 DUT
//     gpio_agent（PASSIVE）：只监听 AXI 总线，不做任何驱动
//
// 【为什么用 PASSIVE Agent 而非直接复用 axi_monitor】
//   AXI Monitor 监听所有 AXI 事务（不分地址）。而 gpio_monitor 只关心
//   地址落在 GPIO 区域的事务——它过滤地址空间，只捕获相关事务。
//   这种"按地址过滤"的设计让 Scoreboard 不需要自己筛选——它只收到
//   GPIO 相关的 transaction，逻辑更清晰。
//
// 【gpio_transaction 结构】
//   reg_sel：GPIO 寄存器选择（DIR=0, OUT=1, IN=2）
//   data：写入/读出的数据
//   kind：0=读, 1=写
//   error：响应码是否为错误（bresp/rresp != OKAY）
//
// 【地址空间检测】
//   base_addr = 32'h20000000（GPIO 基地址）
//   addr_mask = 32'h00000FFF（64KB 空间）
//   地址在 [base_addr, base_addr | addr_mask] 范围内 → 是 GPIO 访问
//
//   ★ 可配置性：base_addr 可通过 config_db 覆盖
//     这意味着同一个 gpio_agent 代码可以复用于不同 SoC（只要基地址不同）
//
// 【gpio_output_monitor vs gpio_monitor — 两个 Monitor 的区别】
//   本文件中的 gpio_monitor：监控 AXI 总线上的 GPIO 寄存器读写（地址空间过滤）
//   gpio_monitor.sv 中的 gpio_output_monitor：监控 gpio_out 引脚的信号变化（边沿检测）
//   它们互补——前者看"CPU 写了什么"，后者看"引脚实际输出了什么"
// ═══════════════════════════════════════════════════════════════════════════════

`include "uvm_macros.svh"
import uvm_pkg::*;

// ═══════════════════════════════════════════════════════════════════════════════
//  gpio_reg_e — GPIO 寄存器类型枚举
//
//  ★ 为什么用 typedef enum？
//    避免"魔数"（magic number）——代码中不裸写 0/1/2，而是用有意义的名字。
//    gpio_reg_e'(a[3:2]) 将地址的 bit[3:2] 映射为寄存器类型：
//      a[3:2]=0 → GPIO_DIR, =1 → GPIO_OUT, =2 → GPIO_IN
//    这与 RTL 中 GPIO 的寄存器偏移一致（DIR@0x00, OUT@0x04, IN@0x08）
// ═══════════════════════════════════════════════════════════════════════════════
typedef enum bit [1:0] { GPIO_DIR=0, GPIO_OUT=1, GPIO_IN=2 } gpio_reg_e;

// ═══════════════════════════════════════════════════════════════════════════════
//  gpio_transaction — GPIO 寄存器访问事务
//
//  ★ 约束：写的目标寄存器不能是 GPIO_IN（输入寄存器只读）
//    constraint legal_gpio { (kind==1) -> (reg_sel != GPIO_IN); }
//    这是常识性约束——写入输入寄存器没有意义。
// ═══════════════════════════════════════════════════════════════════════════════
class gpio_transaction extends uvm_sequence_item;
    `uvm_object_utils(gpio_transaction)
    rand gpio_reg_e reg_sel;           // ★ 目标寄存器（DIR/OUT/IN）
    rand bit [31:0] data;              // 写入数据
    rand bit         kind;             // 0=读, 1=写
    bit              error;            // 响应是否异常
    bit [31:0]       actual_data;      // 实际读回的数据（备用）
    constraint legal_gpio {
        (kind == 1) -> (reg_sel != GPIO_IN);  // ★ 不能写输入寄存器
    }
    function new(string n = "gpio_transaction");
        super.new(n);
    endfunction
endclass

// ═══════════════════════════════════════════════════════════════════════════════
//  gpio_monitor — AXI 总线 GPIO 地址空间监控器
//
//  ★ 监控策略：
//    监听 AXI 总线的 AW/AR 握手 → 检查地址是否在 GPIO 范围内
//    → 跟踪 W/B 或 R 通道 → 打包 gpio_transaction → ap.write()
//
//  ★ 和 axi_monitor 的区别：
//    axi_monitor 捕获**所有** AXI 事务（不分地址，发给 riscv_scoreboard）
//    gpio_monitor 只捕获**GPIO 地址空间**的事务（发给 firmware_scoreboard）
//    两者可以并存——同一笔 AXI 写会被两个 Monitor 同时捕获，发给不同的 Scoreboard
// ═══════════════════════════════════════════════════════════════════════════════
class gpio_monitor extends uvm_monitor;
    `uvm_component_utils(gpio_monitor)
    virtual riscv_if vif;
    uvm_analysis_port#(gpio_transaction) ap;

    // ★ 可配置基地址（通过 config_db 覆盖）★
    bit [31:0] base_addr = 32'h20000000;   // GPIO 基地址
    bit [31:0] addr_mask  = 32'h00000FFF;  // 地址掩码（64KB）

    function new(string n, uvm_component p);
        super.new(n, p);
    endfunction

    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db#(virtual riscv_if)::get(this, "", "vif", vif))
            `uvm_fatal("NOVIF", "")
        void'(uvm_config_db#(bit[31:0])::get(this, "", "base_addr", base_addr));
        ap = new("ap", this);
    endfunction

    virtual task run_phase(uvm_phase phase);
        reg [31:0] a;                    // ★ VCS 2018: task 级变量声明
        forever begin
            gpio_transaction tr;
            @(vif.monitor_cb);
            if (!vif.resetn) continue;   // ★ 复位期间跳过

            // ── 写检测（AW → W → B）──
            if (vif.monitor_cb.axi_awvalid && vif.monitor_cb.axi_awready) begin
                a = vif.monitor_cb.axi_awaddr;
                // ★ 地址过滤：只在 GPIO 范围内才处理 ★
                if (a >= base_addr && a <= (base_addr | addr_mask)) begin
                    tr = gpio_transaction::type_id::create("gpio");
                    tr.kind = 1;         // 写
                    tr.reg_sel = gpio_reg_e'(a[3:2]);  // ★ 地址映射到寄存器类型
                    // ── W 通道（带复位保护）──
                    do begin @(vif.monitor_cb); if (!vif.resetn) break; end
                    while (!(vif.monitor_cb.axi_wvalid && vif.monitor_cb.axi_wready));
                    if (!vif.resetn) continue;
                    tr.data = vif.monitor_cb.axi_wdata;
                    // ── B 通道（捕获响应码）──
                    do begin @(vif.monitor_cb); if (!vif.resetn) break; end
                    while (!(vif.monitor_cb.axi_bvalid && vif.monitor_cb.axi_bready));
                    if (!vif.resetn) continue;
                    tr.error = vif.monitor_cb.axi_bresp != 2'b00;  // ★ 检查响应
                    ap.write(tr);
                end
            end

            // ── 读检测（AR → R）──
            if (vif.monitor_cb.axi_arvalid && vif.monitor_cb.axi_arready) begin
                a = vif.monitor_cb.axi_araddr;
                if (a >= base_addr && a <= (base_addr | addr_mask)) begin
                    tr = gpio_transaction::type_id::create("gpio");
                    tr.kind = 0;         // 读
                    tr.reg_sel = gpio_reg_e'(a[3:2]);
                    do begin @(vif.monitor_cb); if (!vif.resetn) break; end
                    while (!(vif.monitor_cb.axi_rvalid && vif.monitor_cb.axi_rready));
                    if (!vif.resetn) continue;
                    tr.data = vif.monitor_cb.axi_rdata;
                    tr.error = vif.monitor_cb.axi_rresp != 2'b00;
                    ap.write(tr);
                end
            end
        end
    endtask

endclass

// ═══════════════════════════════════════════════════════════════════════════════
//  gpio_agent — GPIO 被动 Agent 容器
//
//  ★ is_active = UVM_PASSIVE：不创建 driver（GPIO 不需要主动驱动）
//  ★ 只包含 monitor + analysis_port
// ═══════════════════════════════════════════════════════════════════════════════
class gpio_agent extends uvm_agent;
    `uvm_component_utils(gpio_agent)
    uvm_analysis_port#(gpio_transaction) ap;
    gpio_monitor monitor;

    function new(string n, uvm_component p);
        super.new(n, p);
        is_active = UVM_PASSIVE;         // ★ 被动模式
    endfunction

    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        monitor = gpio_monitor::type_id::create("monitor", this);
        ap = new("ap", this);
    endfunction

    virtual function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
        monitor.ap.connect(ap);          // ★ Monitor → Agent AP
    endfunction

endclass
