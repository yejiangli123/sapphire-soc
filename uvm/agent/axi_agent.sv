// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  axi_agent.sv — AXI4-Lite 总线 Agent（容器组件）                               ║
// ║  ★ 封装 Driver + Monitor + Sequencer + Analysis Port                          ║
// ╚══════════════════════════════════════════════════════════════════════════════════╝
//
// 【UVM 角色 — uvm_agent】
//   Agent 是 UVM 中的**容器组件**——它把 Driver、Monitor、Sequencer 打包在一起，
//   形成对 DUT 某个接口的完整验证逻辑。
//
//   一个 Agent 包含：
//     - Driver（ACTIVE 时有）：驱动接口信号的时序逻辑
//     - Monitor：采样接口信号，打包 transaction
//     - Sequencer（ACTIVE 时有）：管理 sequence 的调度
//     - Analysis Port：Mon → 外部的广播端口
//
// 【ACTIVE vs PASSIVE 模式】
//   Agent 通过 is_active 控制是否创建 driver 和 sequencer：
//     - UVM_ACTIVE（默认）：创建 driver + monitor + sequencer（能主动发事务）
//     - UVM_PASSIVE：只创建 monitor（只能被动监听）
//
//   ★ 为什么需要 PASSIVE 模式？
//     有些接口你只监听不驱动——比如 GPIO/UART。用 PASSIVE 模式
//     可以复用同一个 agent 代码，不创建无用的 driver。
//
//   模式由 test 或 env 在 build_phase 中通过设置 is_active 控制：
//     axi_agt.is_active = UVM_ACTIVE;
//
// 【Analysis Port 的作用】
//   ap（analysis_port）是 Monitor 的对外广播端口。
//   在 connect_phase 中，monitor.ap 连接到 ap → 外部通过 ap 接收 transaction。
//
//   设计原理：
//     Agent 内部的 ap 是"包装端口"——Monitor 只连到 AP，外部也只连到 AP。
//     如果将来 Monitor 换掉了，只要新的 Monitor 也连到 AP，外部无需修改。
//     这是典型的"依赖倒置"设计（面向接口而非实现）。
//
// 【connect_phase 中的握手连接】
//   driver.seq_item_port.connect(sequencer.seq_item_export);
//   ★ 这是 Driver ↔ Sequencer 的唯一通信通道。
//     连接后，Driver 的 get_next_item/item_done 才能正常工作。
//     如果不连接，Driver 会在 get_next_item 处永久阻塞。
//
//     seq_item_port（在 Driver 侧）← 连接 → seq_item_export（在 Sequencer 侧）
//     它们是一对 TLM 端口：port 是发起方（Driver），export 是接收方（Sequencer）。
//     实际上数据流是双向的：Driver 向 Sequencer "请求" item → Sequencer "返回" item。
// ═══════════════════════════════════════════════════════════════════════════════

`include "uvm_macros.svh"
import uvm_pkg::*;

class axi_agent extends uvm_agent;
    `uvm_component_utils(axi_agent)

    // ★ 子组件
    uvm_analysis_port#(axi_transaction) ap;  // ★ 广播端口（Mon → Scoreboard/Coverage）
    axi_driver    driver;                     // ★ Driver：驱动 AXI 信号（ACTIVE 模式）
    axi_monitor   monitor;                    // ★ Monitor：采样 AXI 信号（始终创建）
    axi_sequencer sequencer;                  // ★ Sequencer：调度 sequence（ACTIVE 模式）

    function new(string n, uvm_component p);
        super.new(n, p);
    endfunction

    // ═══════════════════════════════════════════════════════════════════════
    //  build_phase — 条件创建子组件
    //
    //  ★ Monitor 始终创建——无论 ACTIVE 还是 PASSIVE
    //  ★ Driver + Sequencer 仅在 ACTIVE 模式下创建
    //
    //  is_active 的默认值是 UVM_ACTIVE（在 uvm_agent 基类中定义）。
    //  如需 PASSIVE 模式，在 env 的 build_phase 中显式设置：
    //    agent.is_active = UVM_PASSIVE;
    // ═══════════════════════════════════════════════════════════════════════
    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        // ★ Monitor 始终创建（PASSIVE 模式下也需要监控）
        monitor = axi_monitor::type_id::create("monitor", this);
        ap = new("ap", this);                  // ★ 创建分析端口
        // ★ ACTIVE 模式：额外创建 driver 和 sequencer
        if (get_is_active() == UVM_ACTIVE) begin
            driver    = axi_driver::type_id::create("driver", this);
            sequencer = axi_sequencer::type_id::create("sequencer", this);
        end
    endfunction

    // ═══════════════════════════════════════════════════════════════════════
    //  connect_phase — 内部连接
    //
    //  ★ Monitor.ap → Agent.ap（包装）
    //    外部通过 agent.ap 接收 Monitor 发出的 transaction
    //
    //  ★ Driver.seq_item_port → Sequencer.seq_item_export（ACTIVE 模式）
    //    这是 Driver 和 Sequencer 之间的标准 TLM 连接
    //    不连接的话，Driver 无法从 Sequence 获取 transaction
    //
    //  ★ 为什么 connect_phase 中连接而非 build_phase？
    //    build_phase 只创建组件，connect_phase 才建立内部通信。
    //    分离确保了所有组件先创建好，再连接（避免连到 null 端口）。
    // ═══════════════════════════════════════════════════════════════════════
    virtual function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
        // ★ Monitor → 外部（通过 AP 包装）
        monitor.ap.connect(ap);
        // ★ Driver ↔ Sequencer（仅在 ACTIVE 模式）
        if (get_is_active() == UVM_ACTIVE)
            driver.seq_item_port.connect(sequencer.seq_item_export);
    endfunction

endclass
