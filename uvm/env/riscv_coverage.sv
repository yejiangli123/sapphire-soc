// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  riscv_coverage.sv — 功能覆盖率收集器（Functional Coverage）                    ║
// ║  ★ AXI 事务交叉覆盖：外设地址 × 读写类型 × 传输大小                            ║
// ╚══════════════════════════════════════════════════════════════════════════════════╝
//
// 【UVM 角色 — uvm_subscriber】
//   本类继承自 uvm_subscriber#(axi_transaction)，这是一个参数化的"订阅者"类。
//   它通过 analysis_export 接收 Monitor 广播的 AXI transaction（不用自己写 imp）。
//
//   uvm_subscriber 的本质：带 analysis_export 的 uvm_component。
//   它内置了 analysis_export，子类只需实现 write(T tr) 即可接收数据。
//
// 【Functional Coverage 是什么】
//   Functional Coverage（功能覆盖率）是工程师**手工指定**的"哪些场景被验证过"。
//   它与 Code Coverage（工具自动收集：行/条件/翻转/FSM）互补：
//
//     Code Coverage：告诉我"哪些代码被执行过"（工具自动）
//     Functional Coverage：告诉我"哪些功能场景被验证过"（人工定义）
//
//   举例：
//     Code Coverage 说：GPIO 的写数据路径 100% 覆盖
//     Functional Coverage 说：GPIO 的 1/2/4 字节写都验证过了 ✓
//
// 【本 covergroup 的设计】
//   axi_cg 有三个 coverpoint：
//     addr_cp：外设地址空间（GPIO / UART / Timer / PLIC）
//     kind_cp：读写类型（读 / 写）
//     size_cp：传输宽度（1B / 2B / 4B，对应 AXI SIZE 编码 0/1/2）
//
//   cross：三者交叉 → 确保每种外设 × 每类操作 × 每种宽度的组合都被测试到
//     例：(GPIO, WR, 1B) + (GPIO, WR, 2B) + ... + (PLIC, RD, 4B)
//     共 4×2×3 = 24 个交叉仓（cross bin）
//
// 【为什么用 function sample 而非 event 触发】
//   covergroup 可以用 event（@(posedge clk)）或 function sample 来采样。
//   本设计用 function sample——当 Monitor 发出 transaction 时手动调用 sample()。
//   好处：采样时机精确，不受时钟影响。
//
//   ★ 对比：
//     @(posedge clk)：每拍自动采样（适合持续监控的信号）
//     function sample()：手动调用（适合事件驱动的采样）
//
// 【复位事务过滤】
//   tr.resp == 2'b11 → 复位中止事务 → 不完整，跳过采样。
//   如果在复位期间采样，会把"半截"事务计入覆盖率——污染统计数据。
// ═══════════════════════════════════════════════════════════════════════════════

`include "uvm_macros.svh"
import uvm_pkg::*;

class riscv_coverage extends uvm_subscriber#(axi_transaction);
    `uvm_component_utils(riscv_coverage)

    // ═══════════════════════════════════════════════════════════════════════
    //  axi_cg — AXI 事务覆盖组
    //
    //  ★ function sample(addr, kind, size)：
    //    外部调用 axi_cg.sample(tr.addr, tr.kind, tr.size)
    //    会自动更新所有 coverpoint 和 cross
    //
    //  ★ 交叉覆盖 cross addr_cp, kind_cp, size_cp：
    //    生成的 bin = 外设(4) × 读写(2) × 大小(3) = 24 个交叉仓
    //    每个交叉仓都被命中 → 100% functional coverage
    // ═══════════════════════════════════════════════════════════════════════
    covergroup axi_cg with function sample(
        bit [31:0] addr,       // AXI 地址
        bit        kind,       // 0=读, 1=写
        bit [2:0]  size        // AXI SIZE 编码：0=1B, 1=2B, 2=4B
    );
        // ── coverpoint 1：外设地址空间 ──
        addr_cp: coverpoint addr {
            bins gpio  = {['h20000000:'h2000FFFF]};    // GPIO (64KB)
            bins uart  = {['h30000000:'h3000FFFF]};    // UART (64KB)
            bins timer = {['h40000000:'h4000FFFF]};    // Timer (64KB)
            bins plic  = {['h50000000:'h5000FFFF]};    // PLIC (64KB)
            // ★ 地址落在范围之外 → 不计入覆盖率（不是非法访问）
            //    如果需要覆盖非法地址访问，应加 illegal_bins
        }

        // ── coverpoint 2：读写类型 ──
        kind_cp: coverpoint kind {
            bins rd = {0};      // 读事务
            bins wr = {1};      // 写事务
        }

        // ── coverpoint 3：传输宽度（AXI SIZE 编码）──
        size_cp: coverpoint size {
            bins b8       = {0};      // 1 字节（size=0）
            bins halfword = {1};      // 2 字节（size=1）
            bins word     = {2};      // 4 字节（size=2）
        }

        // ★ 关键交叉：外设 × 读写 × 大小 ★
        //   目标：24/24 bins 命中
        cross addr_cp, kind_cp, size_cp;
    endgroup

    function new(string n, uvm_component p);
        super.new(n, p);
        axi_cg = new();          // ★ covergroup 必须用 new() 实例化
    endfunction

    // ── write() — uvm_subscriber 的回调函数 ──
    virtual function void write(axi_transaction tr);
        if (tr.resp == 2'b11) return;        // ★ 过滤复位中止事务
        axi_cg.sample(tr.addr, tr.kind, tr.size);
    endfunction

endclass
