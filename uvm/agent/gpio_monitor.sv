// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  gpio_monitor.sv — GPIO 输出监测器（Passive Monitor）                         ║
// ║  ★ 边沿检测 + pin change 事件触发 + 采样握手                                  ║
// ╚══════════════════════════════════════════════════════════════════════════════════╝
//
// 【UVM 角色 — uvm_monitor (PASSIVE)】
//   本 Monitor 是**被动组件**——只监听 gpio_out 引脚的信号变化，不驱动任何信号。
//   当 gpio_out 值发生变化时，打包成 gpio_transaction 通过 analysis_port 广播。
//
//   与 AXI Monitor 的区别：
//     axi_monitor：需要在 5 个通道的握手信号中提取事务（多阶段跟踪）
//     gpio_monitor：简单边沿检测——前值 != 当前值 → 发送 transaction（单拍采样）
//
// 【边沿检测策略】
//   使用 prev_gpio 寄存器保存上一拍的 gpio_out 值。
//   每拍比较：`vif.gpio_out !== prev_gpio`
//
//   ★ 为什么用 !== 而不是 != ：
//     !== 是 case-inequality（全等判断），X 和 Z 也能正确比较。
//     !=  是 logic-inequality，X/Z 比较结果可能是 X（阻止 if 分支执行）。
//     在仿真中，信号可能存在 X 态（上电未初始化），用 !== 更安全。
//
// 【为什么需要这个 Monitor】
//   固件执行 MUL/DIV/REM 后，通过写 GPIO OUT 寄存器输出结果。
//   本 Monitor 捕获 gpio_out 的每次变化 → 发给 firmware_scoreboard 比对。
//   这是"固件驱动测试"数据通路的关键环节：
//     CPU 写 GPIO → gpio_out 变化 → gpio_monitor 采样 → 发给 scoreboard
//
//   ★ 故障排查技巧：
//     如果 scoreboard 显示 0 PASS 0 FAIL（没有收到任何 GPIO 写），第一个要检查的
//     就是这个 monitor——看 gpio_out 是否有变化（通过 log 中的 "GPIO changed" 消息）。
//     如果 gpio_out 一直是 0，说明固件没有成功写 GPIO → 问题在 CPU 流水线。
// ═══════════════════════════════════════════════════════════════════════════════

`include "uvm_macros.svh"
import uvm_pkg::*;

class gpio_output_monitor extends uvm_monitor;
    `uvm_component_utils(gpio_output_monitor)

    virtual riscv_if        vif;           // ★ 通过 config_db 获取的 virtual interface
    uvm_analysis_port #(gpio_transaction) ap;  // ★ 广播端口：发给 firmware_scoreboard

    bit [31:0] prev_gpio;                  // ★ 上一拍的 gpio_out 值（边沿检测用）

    function new(string name, uvm_component parent);
        super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        ap = new("ap", this);              // ★ 创建 analysis_port
        if (!uvm_config_db#(virtual riscv_if)::get(this, "", "vif", vif))
            `uvm_fatal("NOVIF", "Virtual interface not found")
    endfunction

    // ═══════════════════════════════════════════════════════════════════════
    //  run_phase — 边沿检测主循环
    //
    //  ★ 设计要点：
    //    1. 先等复位释放（vif.wait_reset_release()）——确保上电前不误触发
    //    2. 每拍采样 gpio_out，与前值比较
    //    3. 变化时：创建 transaction → 赋值 data/kind → ap.write() 广播
    //    4. 更新 prev_gpio（下一次比较的基准）
    //
    //  ★ 为什么用 !== 而不是 !=：
    //    case-inequality 可以正确处理 X/Z 态。在仿真的前几个周期，
    //    信号可能处于 X 态——用 != 可能导致 X != 0 的结果为 X，if 分支不执行。
    //
    //  ★ 局限性（已知问题）：
    //    连续两次相同的 GPIO 写法不会被检测到（只检测变化）。
    //    例如：固件先写 0x0060，再写 0x0060——第二次不会触发。
    //    对于当前固件（每次写不同 test_id）来说不是问题，但如果将来需
    //    检测"写相同值"的场景，需要改为每次都采样（去掉边沿检测）。
    // ═══════════════════════════════════════════════════════════════════════
    task run_phase(uvm_phase phase);
        gpio_transaction tx;
        prev_gpio = 32'h0;

        vif.wait_reset_release();          // ★ 等复位释放

        forever begin
            @(posedge vif.clk);
            // ★ 边沿检测：gpio_out 变化时触发 ★
            if (vif.gpio_out !== prev_gpio) begin
                tx = gpio_transaction::type_id::create("gpio_tx");
                tx.data = vif.gpio_out;    // ★ 记录当前 gpio_out 值
                tx.kind = 1;               // write（GPIO 输出 = 相当于"写"）
                `uvm_info("GPIO_MON", $sformatf("GPIO changed: 0x%08h -> 0x%08h",
                    prev_gpio, vif.gpio_out), UVM_HIGH)
                ap.write(tx);              // ★ 广播给 scoreboard
                prev_gpio = vif.gpio_out;  // ★ 更新基准值
            end
        end
    endtask

endclass
