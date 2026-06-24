// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  riscv_vseq_lib.sv — Virtual Sequence 库（AXI 驱动型测试）                     ║
// ║  ★ Virtual Sequencer + Base VSeq + Smoke VSeq + Stress VSeq                  ║
// ╚══════════════════════════════════════════════════════════════════════════════════╝
//
// 【Virtual Sequence 是什么】
//   当验证环境有多个 Agent（各自有独立的 Sequencer），需要协调它们的激励顺序时，
//   用 Virtual Sequence 在 Virtual Sequencer 上统一调度。
//
//   例：先让 AXI Agent 写 GPIO 配置，再等 UART 收到数据，最后读 Timer 计数——
//   这三个操作需要按顺序执行，Virtual Sequence 负责这个协调。
//
// 【Virtual Sequencer 的作用】
//   Virtual Sequencer 本身**不连接任何 Driver**——它只是一个"句柄容器"，
//   持有各个子 Sequencer 的引用：
//     riscv_vsequencer.axi_sqr → axi_agent.sequencer
//   当 Virtual Sequence 需要在 AXI 上发送事务时，通过 p_sequencer.axi_sqr
//   路由到正确的 Sequencer。
//
// 【`uvm_declare_p_sequencer 宏】
//   这个宏在 Sequence 中声明 p_sequencer 句柄，类型为指定的 Sequencer。
//   它让 Sequence 能通过 p_sequencer 访问 Sequencer 中的成员（如 axi_sqr）。
//
//   等价手写代码：
//     riscv_vsequencer p_sequencer;
//     $cast(p_sequencer, m_sequencer);  // 在 pre_start 中自动执行
//
// 【start_item / finish_item 握手协议】
//   这是 Sequence 发送 transaction 的标准流程：
//     1. start_item(tr, priority, sequencer) — 向 sequencer 申请发送权限
//     2. randomize() / 手动赋值 — 填充 transaction 内容
//     3. finish_item(tr) — 通知 sequencer 发送完成
//
//   与 `uvm_do 宏的区别：
//     `uvm_do 自动完成 create + start_item + randomize + finish_item
//     start_item/finish_item 手动控制——允许在 start_item 和 finish_item 之间
//     手动修改字段值（而不完全依赖 randomize）
// ═══════════════════════════════════════════════════════════════════════════════

`include "uvm_macros.svh"
import uvm_pkg::*;

// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  riscv_vsequencer — Virtual Sequencer                                        ║
// ║  ★ 持有 axi_sequencer 的引用，供 Virtual Sequence 调度                        ║
// ╚══════════════════════════════════════════════════════════════════════════════╝
class riscv_vsequencer extends uvm_sequencer;
    `uvm_component_utils(riscv_vsequencer)

    axi_sequencer axi_sqr;               // ★ AXI Sequencer 引用

    function new(string n, uvm_component p);
        super.new(n, p);
    endfunction
endclass

// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  riscv_base_vseq — Virtual Sequence 基类                                     ║
// ║  ★ 从 config_db 获取寄存器模型 + Virtual Interface                           ║
// ║  ★ 复位释放 + 寄存器模型初始化                                                ║
// ╚══════════════════════════════════════════════════════════════════════════════╝
class riscv_base_vseq extends uvm_sequence#(axi_transaction);
    `uvm_object_utils(riscv_base_vseq)
    `uvm_declare_p_sequencer(riscv_vsequencer)  // ★ 声明 p_sequencer 为 riscv_vsequencer 类型

    riscv_reg_block regmodel;            // ★ RAL 寄存器模型
    virtual riscv_if vif;                // ★ Virtual Interface

    function new(string name = "riscv_base_vseq");
        super.new(name);
    endfunction

    // ── pre_start()：Sequence 启动前的准备工作 ──
    task pre_start();
        // ★ 从 config_db 获取寄存器模型（由 test 在 build_phase 中 set）
        if (!uvm_config_db#(riscv_reg_block)::get(null, "*", "regmodel", regmodel))
            `uvm_fatal("NOREG", "Register model not found")
        // ★ 获取 virtual interface
        if (!uvm_config_db#(virtual riscv_if)::get(null, "", "vif", vif))
            `uvm_fatal("NOVIF", "Virtual interface not found")
    endtask

    // ── body()：复位释放 + 寄存器模型复位 ──
    task body();
        vif.wait_reset_release();        // ★ 等 DUT 就绪
        regmodel.reset();                // ★ RAL 镜像复位
        `uvm_info("BASE", "Reset released, starting sequence", UVM_MEDIUM)
    endtask
endclass

// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  axi_smoke_vseq — AXI 冒烟测试（一次写 + 一次读回比对）                        ║
// ║  ★ 验证基本读写通路正确                                                        ║
// ╚══════════════════════════════════════════════════════════════════════════════╝
class axi_smoke_vseq extends riscv_base_vseq;
    `uvm_object_utils(axi_smoke_vseq)

    function new(string name = "axi_smoke_vseq");
        super.new(name);
    endfunction

    task body();
        axi_transaction wr, rd;          // ★ VCS 2018: 声明在 super 前

        super.body();                    // ★ 复位释放 + 模型复位

        // ── 写：Timer COMPARE 寄存器 ← 0xDEAD_BEEF ──
        wr = axi_transaction::type_id::create("wr");
        start_item(wr, , p_sequencer.axi_sqr);  // ★ 向 axi_sqr 申请发送权限
        assert(wr.randomize() with {
            addr == 'h40000004;           // Timer COMPARE 寄存器
            kind == 1;                    // 写
            size == 2;                    // 4 字节
        });
        wr.data = 32'hDEAD_BEEF;         // ★ 手动覆盖随机数据
        finish_item(wr);                 // ★ 通知 driver 发送
        `uvm_info("SMOKE", $sformatf("WR: %s", wr.convert2string()), UVM_MEDIUM)

        // ── 读：Timer COMPARE 寄存器 → 比对 ──
        rd = axi_transaction::type_id::create("rd");
        start_item(rd, , p_sequencer.axi_sqr);
        assert(rd.randomize() with {
            addr == 'h40000004;
            kind == 0;                    // 读
            size == 2;
        });
        finish_item(rd);
        `uvm_info("SMOKE", $sformatf("RD: %s", rd.convert2string()), UVM_MEDIUM)

        // ── 比对：读回的值 == 写入的值？──
        if (rd.data == wr.data)
            `uvm_info("SMOKE", "PASSED!", UVM_NONE)
        else
            `uvm_error("SMOKE",
                $sformatf("Data mismatch: wr=0x%08h rd=0x%08h", wr.data, rd.data))
    endtask
endclass

// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  axi_stress_vseq — AXI 压力测试（随机事务）                                    ║
// ║  ★ 发送 N 个随机 AXI 事务，随机地址/读写/大小                                  ║
// ║  ★ Scoreboard 自动完成比对（不需要 Sequence 手动检查）                         ║
// ╚══════════════════════════════════════════════════════════════════════════════╝
class axi_stress_vseq extends riscv_base_vseq;
    `uvm_object_utils(axi_stress_vseq)

    function new(string name = "axi_stress_vseq");
        super.new(name);
    endfunction

    rand int num_trans = 100;            // ★ 随机事务数（可配置）
    constraint c_num {
        num_trans inside { [10:500] };   // 10~500 次
    }

    task body();
        axi_transaction tr;              // ★ VCS 2018: 声明在 super 前
        super.body();
        repeat (num_trans) begin
            tr = axi_transaction::type_id::create("tr");
            start_item(tr, , p_sequencer.axi_sqr);
            assert(tr.randomize());      // ★ 全部随机（addr/kind/size 按约束生成）
            finish_item(tr);
        end
        `uvm_info("STRESS",
            $sformatf("Completed %0d random transactions", num_trans), UVM_NONE)
    endtask
endclass
