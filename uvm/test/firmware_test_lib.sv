// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  firmware_test_lib.sv — 固件驱动型 UVM 测试库                                  ║
// ║  ★ firmware_base_test + firmware_m_smoke_test + firmware_cache_stress_test     ║
// ╚══════════════════════════════════════════════════════════════════════════════════╝
//
// 【UVM 角色 — uvm_test】
//   Test 是 UVM 验证环境的顶层——它决定"跑哪个测试、怎么配置、跑多久"。
//
//   Test 的工作流程：
//     1. build_phase：创建 env + scoreboard + monitor（补全组件树）
//     2. connect_phase：连接 GPIO monitor → firmware scoreboard
//     3. run_phase：raise_objection → 启动 sequence → 等固件执行 → drop_objection
//     4. report_phase：UV 自动调用 scoreboard.report_phase（不需要手动写）
//
// 【固件驱动测试 vs AXI 驱动测试】
//
//   固件驱动（firmware_base_test）：
//     · 加载 firmware.hex 到 BRAM（在 RTL initial 块中通过 $readmemh 完成）
//     · 释放复位 → RISC-V core 自动从地址 0 开始执行固件
//     · Monitor 监听 GPIO/UART 输出 → Scoreboard 比对
//     · Test 只负责"等待足够长的时间"→ 简单
//
//   AXI 驱动（riscv_base_test）：
//     · Test 启动 AXI sequence → Driver 直接控制总线信号
//     · 需要精确的时序控制和 transaction 级验证
//     · 适合验证外设寄存器读写，不适合验证 CPU 流水线
//
//   本文件使用固件驱动方式——更适合验证 RISC-V core 的指令执行。
//
// 【Objection 机制 — 为什么必须 raise/drop】
//   UVM 的 run_phase 在所有 test 完成 objection 后才结束。
//   如果不 raise_objection，run_phase 可能立刻结束（仿真时间不够）。
//   raise_objection(this) → 告诉 UVM "我的测试还没结束"
//   #100us（等待固件执行）  → 消耗仿真时间
//   drop_objection(this) → 告诉 UVM "我的测试完成了"
//
//   如果忘记 drop → UVM 会一直等到全局超时（timeout）→ UVM_FATAL
//   如果忘记 raise → UVM 在 0 时刻就结束了 run_phase → 什么都没测
//
// 【为什么不直接用 #delay 而用 raise/drop？】
//   #delay 只是消耗时间，不提供"测试是否完成"的语义。
//   objection 机制让 UVM 知道哪些 test 还在运行——当所有 test 都 drop 了，
//   UVM 才能安全结束 run_phase 并进入 check/report phase。
// ═══════════════════════════════════════════════════════════════════════════════

`include "uvm_macros.svh"
import uvm_pkg::*;

// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  firmware_base_test — 固件驱动测试基类                                        ║
// ║  ★ 创建 env + firmware_scoreboard + gpio_output_monitor                      ║
// ║  ★ 连接 GPIO Monitor → firmware Scoreboard                                  ║
// ║  ★ run_phase 中 raise objection → 等 100us → drop                            ║
// ╚══════════════════════════════════════════════════════════════════════════════════╝
class firmware_base_test extends uvm_test;
    `uvm_component_utils(firmware_base_test)

    riscv_env            env;        // ★ 主验证环境（AXI Agent + Slave VIP + Scoreboard）
    firmware_scoreboard  fw_sb;      // ★ 固件专用 Scoreboard（GPIO 输出比对）
    gpio_output_monitor  gpio_mon;   // ★ GPIO 输出引脚监控器（边沿检测）
    virtual riscv_if     vif;        // ★ Virtual Interface

    function new(string name = "firmware_base_test", uvm_component parent = null);
        super.new(name, parent);
    endfunction

    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);

        if (!uvm_config_db#(virtual riscv_if)::get(this, "", "vif", vif))
            `uvm_fatal("NOVIF", "Virtual interface not found")

        // ★ 创建组件（test 是 UVM 树的最顶层，所有组件都在其下）
        env     = riscv_env::type_id::create("env", this);
        fw_sb   = firmware_scoreboard::type_id::create("fw_sb", this);
        gpio_mon = gpio_output_monitor::type_id::create("gpio_mon", this);
    endfunction

    virtual function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
        // ★ GPIO 输出引脚 Monitor → firmware Scoreboard
        //   这是固件驱动测试的核心连接：
        //     CPU 执行固件 → 写 GPIO OUT → gpio_out 引脚变化
        //     → gpio_monitor 检测变化 → 发 transaction → fw_sb 比对
        gpio_mon.ap.connect(fw_sb.gpio_export);
    endfunction

    virtual task run_phase(uvm_phase phase);
        phase.raise_objection(this);           // ★ 测试开始
        `uvm_info("FW_TEST", "Starting firmware-driven test...", UVM_NONE)
        #100us;                                // ★ 等固件执行（100 微秒）
        phase.drop_objection(this);            // ★ 测试完成
    endtask
endclass


// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  firmware_m_smoke_test — M 扩展冒烟测试                                       ║
// ║  ★ 预期：MUL(32,3)=96, DIV(100,7)=14, REM(100,7)=2                           ║
// ║  ★ 在固件执行前注册期望值，固件执行后比对                                      ║
// ╚══════════════════════════════════════════════════════════════════════════════════╝
//
//  ★ 测试流程：
//    1. build_phase：继承 firmware_base_test 的所有组件
//    2. run_phase 开始 → raise objection
//    3. 注册期望值（fw_sb.expect_result）——在固件执行前！
//       · test_id=0 → 期望 MUL(32,3)=96(0x0060)
//       · test_id=1 → 期望 DIV(100,7)=14(0x000E)
//       · test_id=2 → 期望 REM(100,7)=2(0x0002)
//    4. 启动 sequence（firmware_m_verify_seq，等待 3000 周期）
//    5. 额外等 500 周期（GPIO 写入传播延迟）
//    6. drop objection → UV 进入 check/report → firmware_scoreboard 报告结果
//
//  ★ 为什么提前注册期望值？
//    "先定目标，再验证" —— 杜绝后验偏差。
//    如果等到固件执行完再"看看输出是什么然后说那是期望"——
//    那不是验证，是作弊。
//
class firmware_m_smoke_test extends firmware_base_test;
    `uvm_component_utils(firmware_m_smoke_test)

    function new(string name = "firmware_m_smoke_test", uvm_component parent = null);
        super.new(name, parent);
    endfunction

    task run_phase(uvm_phase phase);
        firmware_m_verify_seq seq;
        phase.raise_objection(this);

        // ★ 提前注册期望值（固件执行前）★
        fw_sb.expect_result(0, 32'h0060);    // MUL: 32*3 = 96
        fw_sb.expect_result(1, 32'h000E);    // DIV: 100/7 = 14
        fw_sb.expect_result(2, 32'h0002);    // REM: 100%7 = 2

        `uvm_info("FW_TEST", "M-Smoke: Expecting MUL=96, DIV=14, REM=2", UVM_NONE)

        // ★ 启动 sequence（等待固件执行窗口）★
        seq = firmware_m_verify_seq::type_id::create("seq");
        seq.start(null);

        // ★ 额外等待：GPIO 写入传播到 Monitor 需要几个周期 ★
        repeat (500) @(posedge vif.clk);

        phase.drop_objection(this);
    endtask
endclass


// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  firmware_cache_stress_test — Cache 压力测试                                   ║
// ║  ★ 加载 cache stress 固件，运行 5000 周期                                      ║
// ║  ★ 不注册期望值——只验证不 crash（稳定性测试）                                   ║
// ╚══════════════════════════════════════════════════════════════════════════════════╝
class firmware_cache_stress_test extends firmware_base_test;
    `uvm_component_utils(firmware_cache_stress_test)

    function new(string name = "firmware_cache_stress_test", uvm_component parent = null);
        super.new(name, parent);
    endfunction

    task run_phase(uvm_phase phase);
        firmware_cache_stress_seq seq;
        phase.raise_objection(this);

        seq = firmware_cache_stress_seq::type_id::create("seq");
        // ★ randomize() 为 sequence 的 wait_cycles 赋随机值 ★
        assert(seq.randomize() with { wait_cycles == 5000; });
        seq.start(null);

        phase.drop_objection(this);
    endtask
endclass
