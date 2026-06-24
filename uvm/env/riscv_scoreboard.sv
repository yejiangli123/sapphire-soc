// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  riscv_scoreboard.sv — AXI 事务比对 Scoreboard                                   ║
// ║  ★ 参考模型预测 + 全字段比对 + 复位安全 + 统计报告                              ║
// ╚══════════════════════════════════════════════════════════════════════════════════╝
//
// 【UVM 角色 — uvm_scoreboard】
//   本 Scoreboard 是 AXI 驱动型验证的"裁判"——它接收 Monitor 捕获的 AXI 事务，
//   用参考模型预测期望值，逐字段比对，判定 PASS/FAIL。
//
//   与 firmware_scoreboard 的区别：
//     firmware_scoreboard：固件驱动——监听 GPIO 输出，比对预注册期望值
//     riscv_scoreboard：AXI 驱动——监听 AXI 总线，用 ref_model 动态预测
//
// 【工作流程】
//   Monitor 捕获 AXI 事务 → ap.write(tr)
//     → uvm_analysis_imp 自动调用本 Scoreboard 的 write(tr) 函数
//     → ref_model.predict(tr) 产生预测 transaction（pred）
//     → match(pred, tr) 逐字段比对
//     → 统计 checked/passed/failed
//     → report_phase 汇总
//
// 【TLM 通信机制】
//   uvm_analysis_imp#(axi_transaction, riscv_scoreboard) axi_imp;
//   ★ imp（implementation port）是 TLM 连接的终端——
//     必须在本类中实现 write(axi_transaction) 函数。
//     当 Monitor 调用 ap.write(tr) 时，UVM 自动路由到本类的 write() 函数。
//
//   ★ 为什么 imp 的回调函数必须叫 write()？
//     uvm_analysis_imp 内部通过宏约定回调函数名为 write。
//     如果改名为 write_axi()，TLM 连接会失败（编译不报错，但运行时不调用）。
//     这是 UVM 中常见的"无声失败"（silent failure）。
//
// 【复位安全】
//   resp == 2'b11（DECERR + 特定编码）表示事务被复位中止。
//   此时跳过预测和比对（因为 DUT 状态不可靠），但计入 checked。
//   避免"假阳性"FAIL——被复位中断的事务不算 bug。
//
// 【match() 比对策略】
//   逐字段比对（kind / addr / size / data / resp）。
//   任何字段不匹配 → return 0。
//   ★ 不比对 wstrb：因为 ref_model 使用 Monitor 采样的实际 wstrb
//     （而非 Driver 生成的），两者始终相同——比对 wstrb 无意义。
// ═══════════════════════════════════════════════════════════════════════════════

`include "uvm_macros.svh"
import uvm_pkg::*;

class riscv_scoreboard extends uvm_scoreboard;
    `uvm_component_utils(riscv_scoreboard)

    // ★ TLM 接收端口：Monitor → Scoreboard
    //   imp 参数：(transaction类型, 接收类类型)
    //   回调函数必须是 write(axi_transaction tr)
    uvm_analysis_imp#(axi_transaction, riscv_scoreboard) axi_imp;

    // ★ 参考模型（字节可写内存镜像）
    riscv_ref_model ref_model;

    // ★ 统计计数器
    int checked;    // 总检查数
    int passed;     // 通过数
    int failed;     // 失败数

    // ═══════════════════════════════════════════════════════════════════════
    //  reset() — 重置统计 + 清空参考模型
    //   每次新测试开始前由 env.reset() 调用
    // ═══════════════════════════════════════════════════════════════════════
    virtual function void reset();
        checked = 0;
        passed  = 0;
        failed  = 0;
        ref_model.reset();                 // ★ 清空参考模型的内存镜像
    endfunction

    function new(string n, uvm_component p);
        super.new(n, p);
    endfunction

    // ── build_phase：创建 TLM 端口 + 参考模型 ──
    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        axi_imp   = new("axi_imp", this);  // ★ 创建 imp（必须传 this，否则回调不到本类）
        ref_model = riscv_ref_model::type_id::create("ref_model", this);
    endfunction

    // ═══════════════════════════════════════════════════════════════════════
    //  match() — 全字段比对
    //
    //  返回值：1 = 匹配，0 = 不匹配
    //
    //  ★ 比对所有影响功能正确性的字段：
    //    kind  — 读写类型
    //    addr  — 目标地址
    //    size  — 传输宽度
    //    data  — 数据值（最关键的比对项）
    //    resp  — AXI 响应码
    //
    //  ★ 不比对 wstrb：参考模型的 wstrb 就是从 Monitor 复制的
    //    （copy_fields），比对它没有意义。
    // ═══════════════════════════════════════════════════════════════════════
    function bit match(axi_transaction pred, axi_transaction obs);
        if (pred.kind != obs.kind) return 0;    // 读写类型
        if (pred.addr != obs.addr) return 0;    // 地址
        if (pred.size != obs.size) return 0;    // 传输宽度
        if (pred.data != obs.data) return 0;    // ★ 数据（最重要）
        if (pred.resp != obs.resp) return 0;    // 响应码
        return 1;                                // ★ 所有字段匹配
    endfunction

    // ═══════════════════════════════════════════════════════════════════════
    //  write() — TLM 回调函数（Monitor 的 ap.write(tr) 触发）
    //
    //  ★ 这是 UVM TLM 的核心机制：
    //    Monitor 调用 ap.write(tr)
    //    → analysis_port 广播
    //    → uvm_analysis_imp 接收
    //    → 自动调用本 scoreboard 的 write(tr)
    //
    //  ★ 函数名必须是 write——这是 UVM 约定，不可改名
    //
    //  处理流程：
    //    1. 检查 resp==2'b11（复位中止）→ 跳过比对
    //    2. ref_model.predict(tr) → 获得预测 transaction
    //    3. match(pred, tr) → 逐字段比对
    //    4. 统计 PASS/FAIL
    // ═══════════════════════════════════════════════════════════════════════
    virtual function void write(axi_transaction tr);
        axi_transaction pred;              // ★ VCS 2018: 声明必须在前

        // ── 复位中止：跳过比对 ──
        if (tr.resp == 2'b11) begin
            checked++;
            `uvm_info("SCBD",
                $sformatf("SKIP reset-aborted: %s", tr.convert2string()), UVM_MEDIUM)
            return;
        end

        // ── 参考模型预测 ──
        pred = ref_model.predict(tr);
        checked++;

        // ── 比对 ──
        if (match(pred, tr)) begin
            passed++;
            `uvm_info("SCBD",
                $sformatf("MATCH: %s", tr.convert2string()), UVM_HIGH)
        end
        else begin
            failed++;
            `uvm_error("SCBD",
                $sformatf("MISMATCH\n  Exp: %s\n  Got: %s",
                          pred.convert2string(), tr.convert2string()))
        end
    endfunction

    // ═══════════════════════════════════════════════════════════════════════
    //  report_phase() — 汇总报告
    //
    //  ★ 判定标准：
    //    failed > 0 → UVM_ERROR + 测试失败
    //    failed = 0 → 测试通过
    //
    //  ★ 同时报告 checked 数量——如果 checked=0，说明 Monitor 没捕获到
    //    任何事务（环境连接问题）
    // ═══════════════════════════════════════════════════════════════════════
    virtual function void report_phase(uvm_phase phase);
        super.report_phase(phase);
        `uvm_info("SCBD",
            $sformatf("Scoreboard: %0d checked, %0d passed, %0d failed",
                      checked, passed, failed), UVM_NONE)
        if (failed > 0)
            `uvm_error("SCBD", $sformatf("%0d mismatches!", failed))
    endfunction

endclass
