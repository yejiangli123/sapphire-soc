// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  enhanced_scoreboard.sv — 增强型 Scoreboard（多 Agent + Golden Model）         ║
// ║  ★ AXI 事务比对 + 指令写入自动喂给 Golden Model 执行                            ║
// ╚══════════════════════════════════════════════════════════════════════════════════╝
//
// 【UVM 角色 — uvm_scoreboard（增强版）】
//   本 Scoreboard 是 riscv_scoreboard 的增强版本——除了标准的 AXI 事务比对，
//   还集成了 RISC-V Golden Model（指令级 ISA 模拟器）。
//
//   与 riscv_scoreboard 的区别：
//     riscv_scoreboard：只用 ref_model（内存模型）比对
//     enhanced_scoreboard：ref_model 比对 + Golden Model 指令执行
//
// 【核心增强：指令写入自动喂给 Golden Model】
//   当 AXI Monitor 捕获到一次"指令写入"操作时（kind=1 且 addr >= 0x40000000，
//   即写到了 CPU 的指令区域），Scoreboard 自动将写入的数据（机器码）喂给
//   Golden Model 执行（golden.execute()）。
//
//   这样做的意义：
//     · 固件加载过程（firmware.hex → BRAM）通过 AXI 写入完成
//     · 每写入一条指令 → Golden Model 执行它 → 维护"理想"的寄存器/内存状态
//     · 固件执行完毕后，Golden Model 的状态 = 预期状态
//     · 将 DUT 实际状态 vs Golden Model 预期状态比对 → PASS/FAIL
//
//   ★ 这是"在线比对"（online checking）的雏形——验证和 DUT 执行同时进行
//
// 【当前状态与限制】
//   ★ Golden Model 已实现（riscv_golden_model.sv），execute() 支持全部 RV32I 指令
//   ★ 但 DUT 的实际寄存器/内存状态难以从外部读取（需要通过 debug 接口或 GPIO 输出）
//   ★ 当前版本只执行了"喂给 Golden Model"这一步，未实现"读取 DUT 状态并比对"
//   ★ 完整实现需要增加 DUT 状态回读机制（如通过 AXI 读寄存器映射的调试接口）
//
// 【报告格式】
//   本 Scoreboard 使用 $display 直接打印格式化的表格报告
//   （而非 `uvm_info）——使报告在 log 中更显眼
// ═══════════════════════════════════════════════════════════════════════════════

class enhanced_scoreboard extends uvm_scoreboard;
    `uvm_component_utils(enhanced_scoreboard)

    // ── TLM 端口 + 子组件 ──
    uvm_analysis_imp#(axi_transaction, enhanced_scoreboard) axi_imp;
    riscv_ref_model    mem_ref;       // ★ 字节级内存参考模型
    riscv_golden_model golden;        // ★ RISC-V 指令级黄金模型

    // ── 统计计数器（AXI / GPIO / UART 分别统计）──
    int ax_c, ax_p, ax_f;             // AXI：checked / passed / failed
    int gp_c, gp_p, gp_f;             // GPIO（预留）
    int ua_c, ua_p, ua_f;             // UART（预留）

    function new(string n, uvm_component p);
        super.new(n, p);
    endfunction

    // ── build_phase：创建 TLM 端口 + 参考模型 + Golden Model ──
    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        axi_imp = new("axi_imp", this);
        mem_ref = riscv_ref_model::type_id::create("mem_ref", this);
        golden  = riscv_golden_model::type_id::create("golden", this);
    endfunction

    // ═══════════════════════════════════════════════════════════════════════
    //  write_axi() — AXI 事务回调
    //
    //  ★ 两步处理：
    //    1. 标准比对——ref_model.predict() → 比对 data 字段
    //    2. 指令喂给——如果是写操作且地址在指令区域（>= 0x40000000）
    //       → golden.execute() 执行这条指令
    //
    //  ★ 地址阈值 0x40000000 是示例值，实际使用时应根据 SoC 地址映射调整
    //    （BRAM 指令空间在 0x00000000 开始，此处可能应改为 >= 32'h00000000）
    // ═══════════════════════════════════════════════════════════════════════
    virtual function void write_axi(axi_transaction tr);
        ax_c++;

        // ★ 步骤 1：标准比对（和 riscv_scoreboard 相同）
        axi_transaction pred = mem_ref.predict(tr);
        if (pred.data != tr.data) begin
            `uvm_error("SCBD/AXI",
                $sformatf("MISMATCH Exp:%h Got:%h", pred.data, tr.data))
            ax_f++;
        end
        else begin
            ax_p++;
        end

        // ★ 步骤 2：指令写入喂给 Golden Model
        //   如果是写操作且地址 >= 0x40000000（指令区域）→ 执行该机器码
        if (tr.kind == 1 && tr.addr >= 'h40000000) begin
            bit [31:0] res, ma, mwd;     // 结果 / 访存地址 / 访存数据
            bit [4:0]  rd;               // 目的寄存器
            bit        mw;               // 是否 store
            golden.execute(tr.data, tr.addr, res, rd, mw, ma, mwd);
        end
    endfunction

    // ═══════════════════════════════════════════════════════════════════════
    //  report_phase() — 格式化报告
    //
    //  ★ 使用 $display 直接打印表格（而非 `uvm_info），
    //    使报告在 log 中更显眼、更易读
    //
    //  ★ 同时调用 golden.report_stats() 打印指令分类统计
    //    （帮助评估固件的指令覆盖率）
    // ═══════════════════════════════════════════════════════════════════════
    virtual function void report_phase(uvm_phase phase);
        super.report_phase(phase);
        golden.report_stats();               // ★ 指令统计报告

        $display("╔══════════════════════════════════╗");
        $display("║  ENHANCED SCOREBOARD REPORT      ║");
        $display("║  AXI:  %3d checked/%3d passed/%3d failed ║",
                 ax_c, ax_p, ax_f);
        $display("╚══════════════════════════════════╝");

        if (ax_f > 0)
            `uvm_error("SCBD", $sformatf("%0d TOTAL FAILURES!", ax_f))
        else
            `uvm_info("SCBD", "ALL PASSED", UVM_NONE)
    endfunction

endclass
