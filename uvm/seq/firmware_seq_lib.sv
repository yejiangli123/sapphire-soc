// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  firmware_seq_lib.sv — 固件驱动型测试序列库                                     ║
// ║  ★ firmware_base_seq + firmware_m_verify_seq + firmware_cache_stress_seq       ║
// ╚══════════════════════════════════════════════════════════════════════════════════╝
//
// 【UVM 角色 — uvm_sequence（固件驱动型）】
//   固件驱动型 Sequence 不发送 AXI transaction——它只是"等待固件执行完成"。
//
//   与 AXI 驱动型 Sequence 的区别：
//     axi_smoke_vseq：主动发 AXI 读写，Driver 驱动 DUT
//     firmware_m_verify_seq：被动等待，RISC-V core 自己执行固件
//
//   固件驱动型测试的典型流程：
//     1. RTL initial 块中通过 $readmemh 加载 firmware.hex 到 BRAM
//     2. Test 在 run_phase 中启动 firmware sequence
//     3. Sequence 等复位释放 → 等 N 个时钟周期（固件执行时间）
//     4. 期间，Monitor 监听 GPIO/UART → Scoreboard 自动比对
//     5. Sequence 结束 → Test 结束
//
// 【为什么 Sequence 只等待，不干活】
//   因为固件是 RISC-V core 自己执行的——Sequence 控制不了 CPU 的 PC。
//   固件的加载（firmware.hex）是在 RTL 的 initial 块中完成的——
//   这是仿真时的特殊手段（$readmemh），实际芯片中需要 bootloader。
//
//   所以固件驱动的 Sequence 非常简单——它的唯一职责是"等够时间"。
//   ★ 反向思考：正因为 Sequence 如此简单，Scoreboard 的设计才格外重要——
//     所有的验证逻辑（怎么比对、怎么判定 PASS/FAIL）都在 Scoreboard 中。
//
// 【参数化的 uvm_sequence】
//   firmware_base_seq 的参数是 uvm_sequence_item（而非 axi_transaction），
//   因为固件驱动型 Sequence 不产生任何 transaction。
//   它只是消耗时间，不发送数据。
// ═══════════════════════════════════════════════════════════════════════════════

`include "uvm_macros.svh"
import uvm_pkg::*;

// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  firmware_base_seq — 固件测试基类 Sequence                                   ║
// ║  ★ 只做一件事：等复位释放 → 等固件执行                                        ║
// ╚══════════════════════════════════════════════════════════════════════════════╝
class firmware_base_seq extends uvm_sequence #(uvm_sequence_item);
    `uvm_object_utils(firmware_base_seq)

    virtual riscv_if vif;                // ★ Virtual Interface

    function new(string name = "firmware_base_seq");
        super.new(name);
    endfunction

    // ── pre_start()：获取 virtual interface ──
    task pre_start();
        if (!uvm_config_db#(virtual riscv_if)::get(null, "", "vif", vif))
            `uvm_fatal("NOVIF", "Virtual interface not found")
    endtask

    task body();
        `uvm_info("FW_SEQ", "Waiting for reset release...", UVM_MEDIUM)
        vif.wait_reset_release();        // ★ 等 DUT 就绪
        `uvm_info("FW_SEQ", "Firmware is executing — monitoring GPIO...", UVM_MEDIUM)
        // ★ 子类通过 repeat(n) @(posedge vif.clk) 来等待固件执行
    endtask
endclass


// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  firmware_m_verify_seq — M 扩展验证 Sequence（等待 3000 周期）                 ║
// ╚══════════════════════════════════════════════════════════════════════════════╝
class firmware_m_verify_seq extends firmware_base_seq;
    `uvm_object_utils(firmware_m_verify_seq)

    function new(string name = "firmware_m_verify_seq");
        super.new(name);
    endfunction

    task body();
        super.body();

        // ★ 固件执行的机器码：
        //    MUL(32, 3) = 96  → GPIO OUT = 0x00000060
        //    DIV(100, 7) = 14 → GPIO OUT = 0x0001000E
        //    REM(100, 7) = 2  → GPIO OUT = 0x00020002
        //
        //    ★ 期望值由 test 层注册（fw_sb.expect_result），
        //      不在 Sequence 中定义——职责分离。
        //      Test：定义"什么是正确"
        //      Sequence：定义"什么时候结束"
        //      Scoreboard：定义"怎么比对"

        // ★ 等待 3000 个时钟周期（远超固件执行所需时间）
        repeat (3000) @(posedge vif.clk);
        `uvm_info("FW_SEQ", "M-extension verification window complete", UVM_NONE)
    endtask
endclass


// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  firmware_cache_stress_seq — Cache 压力测试 Sequence                          ║
// ║  ★ 可配置等待周期（随机 1000~10000）                                          ║
// ╚══════════════════════════════════════════════════════════════════════════════╝
class firmware_cache_stress_seq extends firmware_base_seq;
    `uvm_object_utils(firmware_cache_stress_seq)

    rand int wait_cycles;                // ★ 随机等待周期数

    constraint c_wait {
        wait_cycles inside { [1000:10000] };  // 1000~10000 周期
    }

    function new(string name = "firmware_cache_stress_seq");
        super.new(name);
    endfunction

    task body();
        super.body();
        repeat (wait_cycles) @(posedge vif.clk);  // ★ 按随机时长等待
        `uvm_info("FW_SEQ",
            $sformatf("Cache stress: completed %0d cycles", wait_cycles), UVM_NONE)
    endtask
endclass


// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  firmware_m_smoke_seq — M 扩展冒烟 Sequence（等待 2000 周期）                  ║
// ║  ★ 比 firmware_m_verify_seq 更短（适合快速冒烟）                               ║
// ╚══════════════════════════════════════════════════════════════════════════════╝
class firmware_m_smoke_seq extends firmware_base_seq;
    `uvm_object_utils(firmware_m_smoke_seq)

    function new(string name = "firmware_m_smoke_seq");
        super.new(name);
    endfunction

    task body();
        super.body();
        repeat (2000) @(posedge vif.clk);    // ★ 2000 周期（冒烟测试）
        `uvm_info("FW_SEQ", "Firmware execution window complete", UVM_NONE)
    endtask
endclass
