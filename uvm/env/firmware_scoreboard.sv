// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  firmware_scoreboard.sv — 固件驱动型 Scoreboard（Firmware-Driven）             ║
// ║  ★ 监听 GPIO 输出 → 提取 test_id + result → 与预期值比对                     ║
// ╚══════════════════════════════════════════════════════════════════════════════════╝
//
// 【UVM 角色 — uvm_scoreboard】
//   Scoreboard 是 UVM 验证的"裁判"——负责判定测试通过还是失败。
//   工作流程：
//     1. 测试开始前，注册期望值（expect_result）
//     2. 监听 DUT 的输出（通过 TLM analysis_fifo 接收 GPIO transaction）
//     3. 收到输出后，与期望值比对 → PASS 或 FAIL
//     4. 在 report_phase 中汇总结果
//
// 【固件驱动测试策略（Firmware-Driven Test）】
//   这种策略与 AXI Agent 驱动的传统方式不同：
//
//   传统 AXI Agent 方式：
//     Test → Sequence → Driver → 发 AXI 读写 → Scoreboard 比对 AXI 事务
//
//   固件驱动方式（本 Scoreboard 的策略）：
//     Test → 加载 firmware.hex 到 BRAM → RISC-V core 执行固件
//     → 固件计算 MUL/DIV/REM → 写结果到 GPIO
//     → gpio_output_monitor 监听 GPIO 变化 → 发送 transaction
//     → 本 Scoreboard 比对 GPIO 输出值
//
//   这种方式的好处是：CPU 执行"真实"指令，能测试整个流水线通路。
//   缺点是：依赖于固件的正确性——如果固件本身有 bug，验证结果会受影响。
//
// 【GPIO 输出协议】
//   固件和 Scoreboard 之间约定一个简单的通信协议：
//
//     GPIO 输出的 32-bit 数据分为：
//       [31:16] = test_id（标识是哪个测试）
//       [15:0]  = result（测试结果值）
//
//   例：test_id=0 → 期望值=96(0x0060)，固件应写 0x00000060 到 GPIO OUT 寄存器
//
//   这种"test_id + result"的打包方式简单但有效——一个 GPIO 写就完成一次校验。
//
// 【expected_q（期望队列）的设计】
//   expected_q 是一个 SystemVerilog 队列（queue），存储 {test_id, expected_val} 对。
//   每当 check_gpio 找到匹配的 test_id → 比对 → 从队列中删除（释放内存）。
//   未被匹配到的 test_id 在 report_phase 中报 FAIL（说明固件没输出预期结果）。
//
// 【为什么用 uvm_tlm_analysis_fifo 而不直接接 analysis_port】
//   analysis_fifo 提供了缓冲能力——GPIO Monitor 可能快速连续发送多个 transaction，
//   而 Scoreboard 的 run_phase 通过 get() 逐条消费。没有 fifo 缓冲的话，
//   如果 Scoreboard 处理速度跟不上 Monitor 发送速度，transaction 会丢失。
// ═══════════════════════════════════════════════════════════════════════════════

`include "uvm_macros.svh"
import uvm_pkg::*;

class firmware_scoreboard extends uvm_scoreboard;
    `uvm_component_utils(firmware_scoreboard)

    // ★ TLM 端口：接收 GPIO 输出
    uvm_analysis_export #(gpio_transaction) gpio_export;   // export（被动接收）
    uvm_tlm_analysis_fifo  #(gpio_transaction) gpio_fifo;   // ★ 缓冲队列（防丢失）

    // ═══════════════════════════════════════════════════════════════════════
    //  期望结果管理
    //
    //  expected_t：期望结果结构体
    //    test_id      — 测试编号（对应固件的 test_id）
    //    expected_val — 期望的 32-bit 值（比较时只用低 16 位）
    //
    //  expected_q：期望结果队列（$ 表示队列）
    //    expect_result() 将新期望插入队列
    //    check_gpio() 匹配后从队列删除
    //    report_phase() 检查队列是否为空（遗留 = 有未匹配期望）
    // ═══════════════════════════════════════════════════════════════════════
    typedef struct {
        int unsigned test_id;          // 测试编号
        bit [31:0]  expected_val;      // 期望的 GPIO 输出值
    } expected_t;
    expected_t expected_q[$];          // ★ 队列（$）：动态管理期望值

    int pass_cnt, fail_cnt;            // 通过/失败计数器

    function new(string name, uvm_component parent);
        super.new(name, parent);
    endfunction

    // ── build_phase：创建 TLM 端口 ──
    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        gpio_export = new("gpio_export", this);
        gpio_fifo   = new("gpio_fifo", this);
    endfunction

    // ── connect_phase：连接 export → fifo ──
    function void connect_phase(uvm_phase phase);
        gpio_export.connect(gpio_fifo.analysis_export);
    endfunction

    // ═══════════════════════════════════════════════════════════════════════
    //  expect_result() — 注册一个期望结果
    //
    //  在测试开始前（固件执行前）调用。
    //  例：expect_result(0, 32'h0060) → 期望 test_id=0 的 GPIO 输出 = 0x0060(96)
    //
    //  为什么在测试开始前注册？
    //    确保"先定目标，再验证"——杜绝后验偏差（测出来什么就说什么是对的）。
    //    这是验证方法学的基本原则。
    // ═══════════════════════════════════════════════════════════════════════
    function void expect_result(int unsigned test_id, bit [31:0] val);
        expected_t e;
        e.test_id = test_id;
        e.expected_val = val;
        expected_q.push_back(e);               // ★ 插入队列尾部
        `uvm_info("FW_SB", $sformatf("Expect test%0d = 0x%08h", test_id, val), UVM_MEDIUM)
    endfunction

    // ═══════════════════════════════════════════════════════════════════════
    //  run_phase — 持续消费 GPIO transaction 并比对
    //
    //  ★ 标准 Scoreboard 模式：
    //    forever + get() → 阻塞等待下一个 transaction → 比对 → 循环
    //
    //  ★ 为什么用 get() 而不是 try_get()：
    //    get() 阻塞等待——没有 transaction 时 Scoreboard 挂起（不影响仿真性能，
    //    因为 run_phase 是独立线程）。try_get() 不阻塞但需要外部轮询。一般 Scoreboard
    //    用 get() 更简洁。
    // ═══════════════════════════════════════════════════════════════════════
    task run_phase(uvm_phase phase);
        gpio_transaction gpio_tx;
        forever begin
            gpio_fifo.get(gpio_tx);            // ★ 阻塞等待下一个 GPIO 写
            check_gpio(gpio_tx);               // 比对
        end
    endtask

    // ═══════════════════════════════════════════════════════════════════════
    //  check_gpio() — 比对单个 GPIO 输出
    //
    //  ★ 比对逻辑：
    //    1. 拆包：高 16 位 = test_id，低 16 位 = result
    //    2. 查找匹配的 test_id：遍历 expected_q，找 test_id 匹配的期望值
    //    3. 比对低 16 位：result vs expected_val[15:0]
    //    4. 比对后删除：从队列删除已匹配期望（防止重复匹配）
    //
    //  ★ 只比对低 16 位的原因：
    //    firmware_scoreboard 只关心结果值，test_id 只用于标识测试。
    //    期望值的 [31:16] 部分是任意值（可以放扩展信息）。
    //
    //  ★ delete(i) 后 return：
    //    删除当前匹配项后立即返回——每个 transaction 只应该匹配一个期望。
    //    如果删除后继续遍历，可能匹配到重复的 test_id（bug）。
    // ═══════════════════════════════════════════════════════════════════════
    function void check_gpio(gpio_transaction tx);
        bit [31:0] test_id = tx.data[31:16];    // ★ 高 16 位 = 测试编号
        bit [31:0] result  = tx.data[15:0];     // ★ 低 16 位 = 实际计算结果

        foreach (expected_q[i]) begin
            if (expected_q[i].test_id == test_id) begin
                // ★ 找到匹配的 test_id — 比对结果 ★
                if (expected_q[i].expected_val[15:0] == result) begin
                    `uvm_info("FW_SB", $sformatf(
                        "PASS test%0d: got=0x%04h exp=0x%04h",
                        test_id, result[15:0], expected_q[i].expected_val[15:0]), UVM_NONE)
                    pass_cnt++;
                end
                else begin
                    `uvm_error("FW_SB", $sformatf(
                        "FAIL test%0d: got=0x%04h exp=0x%04h",
                        test_id, result[15:0], expected_q[i].expected_val[15:0]))
                    fail_cnt++;
                end
                expected_q.delete(i);            // ★ 删除已匹配项，释放内存
                return;                          // ★ 已匹配，不再继续查找
            end
        end
        // ★ test_id 未在期望队列中找到 → 意外的 GPIO 写入
        `uvm_warning("FW_SB", $sformatf("Unexpected GPIO write: id=%0d val=0x%04h",
            test_id, result))
    endfunction

    // ═══════════════════════════════════════════════════════════════════════
    //  report_phase — 汇总验证结果
    //
    //  ★ 判定标准：
    //    - fail_cnt > 0 → 有比对失败 → UVM_ERROR → 测试 FAIL
    //    - expected_q.size() > 0 → 有期望值未被匹配 → UVM_ERROR → 测试 FAIL
    //      （说明固件没有输出期望的 test_id——可能是固件卡死或跳过了某个测试）
    //    - 两者都为 0 → 全部通过
    //
    //  ★ UVM_ERROR vs UVM_FATAL：
    //    UVM_ERROR 不终止仿真（继续检查后续结果）
    //    UVM_FATAL 立即终止仿真（用于不可恢复的错误）
    //    Scoreboard 中用 ERROR 更合适——一次比对失败不影响后续比对。
    // ═══════════════════════════════════════════════════════════════════════
    function void report_phase(uvm_phase phase);
        super.report_phase(phase);
        `uvm_info("FW_SB", $sformatf(
            "Firmware Scoreboard: %0d PASS, %0d FAIL, %0d unchecked",
            pass_cnt, fail_cnt, expected_q.size()), UVM_NONE)
        if (fail_cnt > 0 || expected_q.size() > 0)
            `uvm_error("FW_SB", "Test FAILED — see above for details")
    endfunction

endclass
