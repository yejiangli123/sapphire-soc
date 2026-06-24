// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  riscv_env.sv — SoC UVM 验证环境顶层（Environment）                            ║
// ║  ★ 整合 4 个 Agent + Scoreboard + Coverage + Virtual Sequencer              ║
// ╚══════════════════════════════════════════════════════════════════════════════════╝
//
// 【UVM 角色 — uvm_env】
//   Environment（env）是 UVM 验证环境的**顶层容器**。
//   它负责：
//     1. 在 build_phase 中创建所有子组件（agent / scoreboard / coverage 等）
//     2. 在 connect_phase 中连接各组件之间的 TLM 端口
//     3. 提供 reset() 钩子供 test 层调用
//
//   env 本身不包含测试逻辑——它只是把组件"组装"起来。
//   真正的测试逻辑在 test 层（哪个 sequence 跑、什么配置）。
//
// 【本 env 的组件清单】
//   ┌─────────────────────────────────────────────────────────┐
//   │                     riscv_env                            │
//   │  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐   │
//   │  │axi_agent │ │gpio_agent│ │uart_agent│ │axi_slave │   │
//   │  │(ACTIVE)  │ │(PASSIVE) │ │(PASSIVE) │ │(PASSIVE) │   │
//   │  │driver+   │ │monitor   │ │monitor   │ │VIP       │   │
//   │  │sequencer │ │          │ │          │ │          │   │
//   │  └────┬─────┘ └──────────┘ └──────────┘ └──────────┘   │
//   │       │ ap                                               │
//   │       ├────→ riscv_scoreboard (scbd)                     │
//   │       └────→ riscv_coverage   (cov)                      │
//   │                                                          │
//   │  ┌──────────────────────┐                                │
//   │  │  riscv_vsequencer    │ ← 协调多个 sequencer           │
//   │  │  (virtual sequencer) │   （当前版本仅连接 axi_sqr）    │
//   │  └──────────────────────┘                                │
//   └─────────────────────────────────────────────────────────┘
//
// 【ACTIVE vs PASSIVE Agent】
//   ACTIVE agent（axi_agent）：包含 driver + sequencer，能**主动发起**总线事务
//   PASSIVE agent（gpio/uart）：只包含 monitor，只**被动监听**信号变化
//
//   axi_agent 设置为 UVM_ACTIVE 后，uvm_agent 内部会自动实例化 driver 和 sequencer
//
// 【TLM 连接策略】
//   axi_agent.ap（analysis_port）
//     ├──→ scbd.axi_imp      — Scoreboard 接收 AXI 事务进行比对
//     └──→ cov.analysis_export — Coverage 收集 AXI 事务信息
//
//   ★ analysis_port 是广播端口——一个发送方可以连接多个接收方。
//     连接后，每次 ap.write(tr) 都会自动调用所有 subscriber 的 write() 方法。
//
// 【Virtual Sequencer 的作用】
//   v_sqr 持有所有 agent 的 sequencer 句柄。
//   当需要协调多个 agent 的激励时（如：先发 AXI 写，等 GPIO 中断后再读），
//   通过 virtual sequence 在 v_sqr 上统一调度。
//   当前版本仅连接 axi_agt.sequencer，gpio/uart 因为是 passive agent 没有 sequencer。
//
// 【reset() 钩子】
//   提供 reset() 函数供 test 调用——通常用于清除 scoreboard 的内部状态。
//   在每次新测试开始时调用，确保前一次测试的残留数据不会影响当前结果。
// ═══════════════════════════════════════════════════════════════════════════════

`include "uvm_macros.svh"
import uvm_pkg::*;

class riscv_env extends uvm_env;
    // ★ UVM Factory 注册
    `uvm_component_utils(riscv_env)

    // ═══════════════════════════════════════════════════════════════════════
    //  子组件声明
    // ═══════════════════════════════════════════════════════════════════════
    axi_agent         axi_agt;     // AXI4-Lite 主动 Agent（Driver + Monitor + Sequencer）
    gpio_agent        gpio_agt;    // GPIO 被动 Agent（仅 Monitor）
    uart_agent        uart_agt;    // UART 被动 Agent（仅 Monitor）
    axi_slave_agent   axi_slv;     // ★ AXI Slave VIP —— 模拟从设备行为（内存镜像）
    riscv_scoreboard  scbd;        // 主 Scoreboard（参考模型比对）
    riscv_coverage    cov;         // 覆盖率收集器
    riscv_vsequencer  v_sqr;       // ★ Virtual Sequencer（协调多 agent 时序）

    function new(string n, uvm_component p);
        super.new(n, p);
    endfunction

    // ═══════════════════════════════════════════════════════════════════════
    //  build_phase — 创建所有子组件
    //
    //  ★ 执行顺序：自顶向下。
    //    UVM 先调用 env 的 build_phase，在 build_phase 中调用 create() 创建子组件。
    //    然后 UVM 自动递归调用每个子组件的 build_phase。
    //
    //  ★ create() vs new()：
    //    create() 是 UVM Factory 方法——创建出来的组件可以被 override。
    //    new() 是 SystemVerilog 构造函数——绕过 Factory，无法被 override。
    //    UVM 中所有 component 必须用 create() 创建。
    //
    //  ★ is_active = UVM_ACTIVE：
    //    axi_agent 设为 ACTIVE → 内部自动创建 driver + sequencer
    //    其他 agent 默认 PASSIVE → 仅有 monitor
    // ═══════════════════════════════════════════════════════════════════════
    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);

        // ★ 创建组件（参数：name, parent）
        axi_agt  = axi_agent::type_id::create("axi_agt", this);
        axi_agt.is_active = UVM_ACTIVE;          // ★ 设为主动模式（需要 driver）

        axi_slv  = axi_slave_agent::type_id::create("axi_slv", this);  // ★ Slave VIP

        gpio_agt = gpio_agent::type_id::create("gpio_agt", this);
        uart_agt = uart_agent::type_id::create("uart_agt", this);

        scbd = riscv_scoreboard::type_id::create("scbd", this);
        cov  = riscv_coverage::type_id::create("cov", this);
        v_sqr = riscv_vsequencer::type_id::create("v_sqr", this);
    endfunction

    // ── reset() — 重置 scoreboard 状态（每次新 test 前调用）──
    virtual function void reset();
        scbd.reset();
    endfunction

    // ═══════════════════════════════════════════════════════════════════════
    //  connect_phase — 连接 TLM 端口
    //
    //  ★ 执行顺序：自底向上（子组件先连接，父组件后连接）。
    //    只有子组件的 TLM port 已经就绪，父组件才能连接它们。
    //
    //  ★ TLM 连接 = 数据通路铺设：
    //    连接完成后，Monitor 发出 ap.write(tr) → 自动流入 Scoreboard 和 Coverage。
    //    不需要 Monitor 显式调用 Scoreboard 的函数 → 松耦合。
    //
    //  ★ 当前连接情况：
    //    axi_agt.ap → scbd.axi_imp（写/读事务比对）
    //    axi_agt.ap → cov.analysis_export（覆盖率采样）
    //    v_sqr.axi_sqr = axi_agt.sequencer（virtual sequencer 持有 axi sqr 句柄）
    //
    //    ★ 尚未连接的：
    //    gpio_agt.ap / uart_agt.ap → 在 firmware 测试中由 firmware_scoreboard 单独连接
    //    （不在主 env connect_phase 中——这是设计的不对称性，后续可统一）
    // ═══════════════════════════════════════════════════════════════════════
    virtual function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
        // ★ Monitor → Scoreboard（事务比对）
        axi_agt.ap.connect(scbd.axi_imp);
        // ★ Monitor → Coverage（覆盖率采样）
        axi_agt.ap.connect(cov.analysis_export);
        // ★ 连接 axi sequencer 到 virtual sequencer（供 virtual sequence 调度）
        v_sqr.axi_sqr = axi_agt.sequencer;
    endfunction

endclass
