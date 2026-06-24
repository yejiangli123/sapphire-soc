// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  riscv_test_lib.sv — AXI 驱动型 UVM 测试库                                      ║
// ║  ★ riscv_base_test + axi_smoke_test + axi_stress_test + error_injection_test  ║
// ╚══════════════════════════════════════════════════════════════════════════════════╝
//
// 【UVM 角色 — uvm_test（AXI 驱动型）】
//   本文件定义了基于 AXI Agent 主动驱动的测试。
//   与 firmware_test_lib.sv（固件驱动型）的区别：
//     AXI 型：Test → Sequence → Driver → 直接控制 AXI 信号 → 读写外设
//     固件型：Test → 等固件执行 → Scoreboard 监听 GPIO 输出
//
//   AXI 驱动型适合验证外设寄存器（GPIO/Timer/UART/PLIC），
//   固件驱动型适合验证 CPU 流水线（M 扩展等指令级测试）。
//
// 【run_seq() 超时机制】
//   riscv_base_test 提供了一个通用的 run_seq() 方法——
//   用 fork...join_any 实现 sequence 执行 + 超时监控。
//   如果 sequence 在 timeout_ns 内未完成 → UVM_FATAL 立即终止仿真。
//   防止某个测试永久卡死在等待握手（如 Slave 未响应）。
//
//   ★ 为什么用 join_any + disable fork 而不是 join + 单独的超时监控？
//     join_any：谁先完成听谁的
//     disable fork：确保另一个线程也被杀死（不残留）
//     如果只用 join（等两个都完成再继续），可能会等超时到了 sequence 才完成——
//     浪费时间且可能产生假阳性（timeout 出现了但 sequence 碰巧也完成了）。
//
// 【report_phase 自动判定】
//   基类 riscv_base_test 的 report_phase 自动检查 env.scbd.failed：
//     failed > 0 → UVM_ERROR → 测试失败
//     failed = 0 → 测试通过
//   这样每个子 test 不需要单独写 report_phase——继承基类的判定逻辑即可。
//
//   注意：这是"自动判定"，但前提是 Scoreboard 的 failed 计数器确实被更新。
//   如果 Scoreboard 有 bug（如未捕获到 Monitor 的事务），fake=0 ≠ PASS——
//   实际上什么都没测。
// ═══════════════════════════════════════════════════════════════════════════════

`include "uvm_macros.svh"
import uvm_pkg::*;

// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  riscv_base_test — AXI 驱动测试基类                                           ║
// ║  ★ 创建 RAL + Env + Virtual Sequencer                                        ║
// ║  ★ run_seq() 超时封装 + report_phase 自动判定                                 ║
// ╚══════════════════════════════════════════════════════════════════════════════╝
class riscv_base_test extends uvm_test;
    `uvm_component_utils(riscv_base_test)

    riscv_env         env;              // ★ 主验证环境
    riscv_reg_block   regmodel;         // ★ RAL 寄存器模型
    riscv_vsequencer  vseqr;            // ★ Virtual Sequencer

    function new(string n, uvm_component p);
        super.new(n, p);
    endfunction

    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);

        // ── 创建 RAL 寄存器模型 ──
        regmodel = riscv_reg_block::type_id::create("regmodel");
        regmodel.build();              // ★ 必须先 build() 再使用
        // ★ 将 RAL 注入 config_db——所有 sequence 可通过 get() 获取
        uvm_config_db#(riscv_reg_block)::set(null, "*", "regmodel", regmodel);

        // ── 创建验证环境 + Virtual Sequencer ──
        env   = riscv_env::type_id::create("env", this);
        vseqr = riscv_vsequencer::type_id::create("vseqr", this);
    endfunction

    // ── connect_phase：连接 Virtual Sequencer 到实际 Sequencer ──
    virtual function void connect_phase(uvm_phase phase);
        vseqr.axi_sqr = env.axi_agt.sequencer;   // ★ 虚→实映射
    endfunction

    // ═══════════════════════════════════════════════════════════════════════
    //  run_seq() — 带超时保护的 Sequence 启动封装
    //
    //  参数：
    //    phase      — UVM phase（用于 raise/drop objection）
    //    vseq       — 要启动的 Virtual Sequence
    //    timeout_ns — 超时（纳秒），默认 1,000,000 ns = 1 ms
    //
    //  流程：
    //    1. raise_objection → UVM 知道测试开始
    //    2. fork：sequence 执行 + 超时计时
    //    3. join_any：谁先完成都继续
    //    4. disable fork → kill 另一个线程
    //    5. drop_objection → UVM 知道测试结束
    //
    //  ★ 超时不是"预期行为"——它是"最后防线"。
    //    如果 sequence 频繁超时，说明 DUT 或环境有死锁 bug。
    // ═══════════════════════════════════════════════════════════════════════
    task run_seq(uvm_phase phase, uvm_sequence_base vseq,
                 time timeout_ns = 1000000);    // ★ 默认 1ms 超时
        phase.raise_objection(this);             // ★ 测试开始
        fork
            begin
                vseq.start(vseqr);              // ★ 在 Virtual Sequencer 上启动
            end
            begin
                #(timeout_ns * 1ns);            // ★ 超时计时
                `uvm_fatal("TIMEOUT", "Sequence timeout!")
            end
        join_any
        disable fork;                            // ★ Kill 未完成的线程
        phase.drop_objection(this);              // ★ 测试结束
    endtask

    // ── 默认 run_phase：启动一个空的 base vseq ──
    virtual task run_phase(uvm_phase phase);
        riscv_base_vseq vseq;
        vseq = riscv_base_vseq::type_id::create("vseq");
        run_seq(phase, vseq);
    endtask

    // ═══════════════════════════════════════════════════════════════════════
    //  report_phase — 自动判定测试结果
    //
    //  ★ 直接从 Scoreboard 的统计计数器读取结果：
    //    failed > 0 → FAIL
    //    failed = 0 → PASS
    //
    //    这是"数据驱动判定"——Test 不需要知道每个测试的预期结果，
    //    只需要检查 Scoreboard 是否报过错。
    // ═══════════════════════════════════════════════════════════════════════
    virtual function void report_phase(uvm_phase phase);
        super.report_phase(phase);
        if (env.scbd.failed > 0) begin
            `uvm_error("TEST",
                $sformatf("FAILED: %0d mismatches!", env.scbd.failed))
        end
        else begin
            `uvm_info("TEST",
                $sformatf("PASSED: %0d checked, %0d passed",
                          env.scbd.checked, env.scbd.passed), UVM_NONE)
        end
    endfunction

endclass


// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  riscv_axi_smoke_test — AXI 冒烟测试                                          ║
// ║  ★ 一次写 + 一次读回比对（验证基本读写通路）                                    ║
// ╚══════════════════════════════════════════════════════════════════════════════╝
class riscv_axi_smoke_test extends riscv_base_test;
    `uvm_component_utils(riscv_axi_smoke_test)

    function new(string n, uvm_component p);
        super.new(n, p);
    endfunction

    virtual task run_phase(uvm_phase phase);
        axi_smoke_vseq vseq;
        vseq = axi_smoke_vseq::type_id::create("vseq");
        run_seq(phase, vseq);            // ★ 默认 1ms 超时
    endtask
endclass


// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  riscv_axi_stress_test — AXI 压力测试                                         ║
// ║  ★ 500 次随机 AXI 事务（随机地址/读写/大小）                                    ║
// ║  ★ num_trans 可在 test 层覆盖（未随机化，硬编码为 500）                        ║
// ╚══════════════════════════════════════════════════════════════════════════════╝
class riscv_axi_stress_test extends riscv_base_test;
    `uvm_component_utils(riscv_axi_stress_test)

    function new(string n, uvm_component p);
        super.new(n, p);
    endfunction

    virtual task run_phase(uvm_phase phase);
        axi_stress_vseq vseq;
        vseq = axi_stress_vseq::type_id::create("vseq");
        vseq.num_trans = 500;            // ★ 覆盖 Sequence 的默认值(100)
        run_seq(phase, vseq);
    endtask
endclass


// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  riscv_error_injection_test — 错误注入测试                                     ║
// ║  ★ 7 类错误场景：非对齐/只读保护/越界/背靠背/RAW/非法size/UART flood          ║
// ║  ★ 每个场景使用 check_resp() 验证响应码，失败自动计数                           ║
// ╚══════════════════════════════════════════════════════════════════════════════╝
class riscv_error_injection_test extends riscv_base_test;
    `uvm_component_utils(riscv_error_injection_test)

    function new(string n, uvm_component p);
        super.new(n, p);
    endfunction

    virtual task run_phase(uvm_phase phase);
        axi_error_injection_seq vseq;
        vseq = axi_error_injection_seq::type_id::create("vseq");
        run_seq(phase, vseq);
    endtask
endclass
