// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  enhanced_coverage.sv — 增强型覆盖率收集器（4 组 Covergroup）                    ║
// ║  ★ AXI 事务 + RISC-V 指令类型 + Hazard 场景 + UART 数据                        ║
// ╚══════════════════════════════════════════════════════════════════════════════════╝
//
// 【UVM 角色 — uvm_subscriber（增强版）】
//   本类是 riscv_coverage 的增强版，收集 4 个维度的功能覆盖率：
//
//   CG1 (axi_cg)     — AXI 事务：外设地址 × 读写类型 × 大小 × 数据值
//   CG2 (instr_cg)   — RISC-V 指令：指令类型 × funct3 交叉
//   CG3 (hazard_cg)  — 流水线冒险：冒险类型 × stall 周期数
//   CG4 (uart_cg)    — UART 通信：波特率 × 数据字节
//
//   ★ 与 riscv_coverage 的区别：
//     riscv_coverage：只收集 AXI 事务覆盖率（1 组 covergroup）
//     enhanced_coverage：收集 4 组 covergroup——覆盖 DUT 的多个维度
//
// 【为什么需要 4 个 Covergroup】
//   每组 covergroup 针对不同的验证目标：
//     CG1 — 总线层（AXI 协议覆盖）
//     CG2 — 指令层（RISC-V ISA 覆盖）
//     CG3 — 微架构层（流水线冒险覆盖——面试加分项）
//     CG4 — 外设层（UART 功能覆盖）
//
//   分层覆盖率让验证工程师能精确定位"哪层的哪些场景没测到"。
//
// 【★ 当前状态与已知 gap】
//   CG2/CG3/CG4 的 sample_*() 函数已定义，但**没有任何组件调用它们**。
//   只有 CG1（axi_cg）通过 write() 回调被自动采样。
//   要使 CG2~CG4 生效，需要在 Monitor 或 Scoreboard 中调用这些 sample 函数。
//   这是环境集成层面的 gap——覆盖率收集器的"架子"搭好了，但"数据源"未接入。
//
// 【面试要点】
//   Q: 你收集了哪些覆盖率？
//   A: AXI 事务（外设×读写×大小×数据值交叉）、RISC-V 指令类型、
//      流水线 Hazard 场景、UART 通信数据。但后三者的采样函数需要
//      在 Monitor 中调用——这是当前版本的已知 gap，已规划后续完善。
// ═══════════════════════════════════════════════════════════════════════════════

`include "uvm_macros.svh"
import uvm_pkg::*;

class enhanced_coverage extends uvm_subscriber#(axi_transaction);
    `uvm_component_utils(enhanced_coverage)

    virtual riscv_if vif;               // ★ Virtual Interface（用于访问 DUT 信号）
    axi_transaction ax;                 // ★ 当前 AXI 事务（CG1 采样用）

    // ╔══════════════════════════════════════════════════════════════════════════╗
    // ║  CG1: axi_cg — AXI 事务覆盖率                                             ║
    // ║  ★ 外设地址 × 读写类型 × 传输大小 × 数据值 × 交叉                          ║
    // ║  ★ option.per_instance = 1：每个实例独立收集（非全局合并）                 ║
    // ║  ★ option.goal = 95：目标 95% 覆盖率（出口标准）                          ║
    // ╚══════════════════════════════════════════════════════════════════════════════╝
    covergroup axi_cg;
        option.per_instance = 1;        // ★ 每个实例独立统计
        option.goal = 95;               // ★ 目标覆盖率 95%

        // ── coverpoint 1：外设地址空间 ──
        addr_cp: coverpoint ax.addr {
            bins gpio  = {['h20000000:'h2000FFFF]};   // GPIO (64KB)
            bins uart  = {['h30000000:'h3000FFFF]};   // UART (64KB)
            bins timer = {['h40000000:'h4000FFFF]};   // Timer (64KB)
            bins plic  = {['h50000000:'h5000FFFF]};   // PLIC (64KB)
            illegal_bins reserved = default;            // ★ 非外设地址 → 非法 bin
            //   非法 bin 表示"不应该出现的地址访问"——如果命中，覆盖率工具报 warning
        }

        // ── coverpoint 2：读写类型 ──
        kind_cp: coverpoint ax.kind {
            bins rd = {0};               // 读
            bins wr = {1};               // 写
        }

        // ── coverpoint 3：传输宽度（AXI SIZE 编码）──
        size_cp: coverpoint ax.size {
            bins b8       = {0};         // 1 字节（size=0 → 2^0 = 1B）
            bins halfword = {1};         // 2 字节（size=1 → 2^1 = 2B）
            bins word     = {2};         // 4 字节（size=2 → 2^2 = 4B）
        }

        // ── coverpoint 4：数据值 ──
        data_cp: coverpoint ax.data {
            bins zero = {0};             // 全零（常见于复位值/清零）
            bins ones = {'1};            // 全 1（常见于掩码/配置）
            bins w1   = {1,2,4,8,16,32,64,128};  // 2 的幂（常见于位操作）
            bins db   = {32'hDEADBEEF};  // ★ 标志性测试值
            bins cb   = {32'hCAFEBABE};  // ★ 标志性测试值
        }

        // ── 交叉覆盖 ──
        c1: cross addr_cp, kind_cp;      // 外设 × 读写（共 4×2=8 bins）
        c2: cross kind_cp, size_cp;      // 读写 × 大小（共 2×3=6 bins）
    endgroup

    // ╔══════════════════════════════════════════════════════════════════════════╗
    // ║  CG2: instr_cg — RISC-V 指令覆盖率                                        ║
    // ║  ★ 指令类型 × funct3 交叉                                                  ║
    // ║  ★ 采样函数：sample_instr(itype, funct3)                                   ║
    // ║  ★ 当前状态：采样函数未接入（需要 Monitor/Scoreboard 调用）                ║
    // ╚══════════════════════════════════════════════════════════════════════════════╝
    int itype;                           // ★ 指令类型（0=ALU, 2=Load, 3=Store, 4=Branch, 5/6/7=Jump）
    bit [2:0] if3;                       // ★ funct3 字段
    covergroup instr_cg;
        option.per_instance = 1;

        op_cp: coverpoint itype {
            bins alu    = {0, 1};        // R-type + I-type ALU + LUI/AUIPC
            bins load   = {2};           // Load（LB/LH/LW/LBU/LHU）
            bins store  = {3};           // Store（SB/SH/SW）
            bins branch = {4};           // Branch（BEQ/BNE/BLT/BGE/BLTU/BGEU）
            bins jump   = {5, 6, 7};     // JAL + JALR + 保留
        }

        f3_cp: coverpoint if3 {
            bins add  = {0};             // ADD/ADDI/SUB
            bins sll  = {1};             // SLL/SLLI
            bins slt  = {2};             // SLT/SLTI
            bins xor_ = {4};             // XOR/XORI
            bins sr   = {5};             // SRL/SRA
            bins or_  = {6};             // OR/ORI
            bins and_ = {7};             // AND/ANDI
        }

        c3: cross op_cp, f3_cp;          // ★ 指令类型 × 操作子类型交叉
    endgroup

    // ╔══════════════════════════════════════════════════════════════════════════╗
    // ║  CG3: hazard_cg — 流水线冒险覆盖率                                         ║
    // ║  ★ 冒险类型 × stall 周期数                                                  ║
    // ║  ★ 采样函数：sample_hazard(htype, stall_c)                                 ║
    // ║  ★ 当前状态：采样函数未接入                                                ║
    // ╚══════════════════════════════════════════════════════════════════════════════╝
    int htype;                           // ★ 冒险类型
    int stall_c;                         // ★ stall 周期数
    covergroup hazard_cg;
        option.per_instance = 1;

        h_cp: coverpoint htype {
            bins raw       = {0};        // RAW（真数据冒险——R-type → I-type 依赖）
            bins load_use  = {1};        // Load-Use（唯一需要 stall 的冒险）
            bins ctrl      = {2};        // 控制冒险（分支预测失败）
            bins none      = {4};        // 无冒险（正常流水）
        }

        s_cp: coverpoint stall_c {
            bins s0 = {0};               // 无 stall
            bins s1 = {1};               // 1 周期 stall（Load-Use）
            bins s2 = {2};               // 2 周期 stall
            bins sm = {[3:10]};          // 3~10 周期 stall（多周期除法/M-unit）
        }
        // ★ 没有 cross——htype 和 stall_c 的交叉覆盖待添加
    endgroup

    // ╔══════════════════════════════════════════════════════════════════════════╗
    // ║  CG4: uart_cg — UART 通信覆盖率                                            ║
    // ║  ★ 波特率 × 发送数据字节                                                    ║
    // ║  ★ 采样函数：sample_uart(baud, data)                                       ║
    // ║  ★ 当前状态：采样函数未接入                                                ║
    // ╚══════════════════════════════════════════════════════════════════════════════╝
    int baud;                            // ★ 波特率
    bit [7:0] ud;                        // ★ UART 数据字节
    covergroup uart_cg;
        option.per_instance = 1;

        b_cp: coverpoint baud {
            bins b9600   = {9600};       // 9600 bps
            bins b115200 = {115200};     // 115200 bps（最常用）
        }

        d_cp: coverpoint ud {
            bins printable = {[32:126]}; // 可打印 ASCII 字符
            bins ctrl      = {[0:31]};   // 控制字符
        }
    endgroup

    // ── 构造函数：初始化 4 组 covergroup ──
    function new(string n, uvm_component p);
        super.new(n, p);
        axi_cg    = new();               // ★ covergroup 必须用 new() 实例化
        instr_cg  = new();
        hazard_cg = new();
        uart_cg   = new();
    endfunction

    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db#(virtual riscv_if)::get(this, "", "vif", vif))
            `uvm_fatal("COV/NOVIF", "Virtual interface not found")
    endfunction

    // ── write() — AXI 事务回调（CG1 自动采样）──
    virtual function void write(axi_transaction tr);
        if (tr.resp == 2'b11) return;    // ★ 过滤复位中止事务
        ax = tr;                          // ★ 更新 CG1 的采样变量
        axi_cg.sample();                  // ★ 触发 CG1 采样
    endfunction

    // ═══════════════════════════════════════════════════════════════════════
    //  手动采样函数（★ 当前未被调用——待集成到 Monitor/Scoreboard）
    //
    //  这些函数提供"窗口"让外部组件注入采样数据。
    //  CG1 通过 write() 自动采样；CG2~CG4 需要外部主动调用。
    //
    //  集成方案：
    //    sample_instr()  → 在 Scoreboard 中每条 AXI 指令写入后调用
    //    sample_hazard() → 在 Monitor 中检测到 stall 时调用
    //    sample_uart()   → 在 UART Monitor 中收到字节时调用
    // ═══════════════════════════════════════════════════════════════════════

    function void sample_instr(int t, bit [2:0] f);
        itype = t;
        if3   = f;
        instr_cg.sample();
    endfunction

    function void sample_hazard(int t, int s);
        htype   = t;
        stall_c = s;
        hazard_cg.sample();
    endfunction

    function void sample_uart(int b, bit [7:0] d);
        baud = b;
        ud   = d;
        uart_cg.sample();
    endfunction

    // ── report_phase：报告各组覆盖率 ──
    virtual function void report_phase(uvm_phase phase);
        super.report_phase(phase);
        `uvm_info("COV",
            $sformatf("AXI=%.1f%% Instr=%.1f%% Hazard=%.1f%% UART=%.1f%%",
                      axi_cg.get_coverage(),
                      instr_cg.get_coverage(),
                      hazard_cg.get_coverage(),
                      uart_cg.get_coverage()), UVM_NONE)
    endfunction

endclass
