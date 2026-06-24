// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  axi_sequencer.sv — AXI4-Lite 事务调度器（Sequencer）                          ║
// ║  ★ 管理 Sequence 的调度，不包含激励生成逻辑                                     ║
// ╚══════════════════════════════════════════════════════════════════════════════════╝
//
// 【UVM 角色 — uvm_sequencer】
//   Sequencer 是 Sequence 和 Driver 之间的"调度器"。
//   它的核心职责：
//     1. 接收 Sequence 发送的 transaction
//     2. 管理多个 Sequence 的仲裁（哪个 Sequence 先发）
//     3. 将 transaction 传递给 Driver
//     4. 接收 Driver 的完成通知（item_done），返回给 Sequence
//
// 【为什么需要 Sequencer 作为中间层】
//   不能让 Sequence 直接给 Driver 发数据——那样 Sequence 和 Driver 就耦合了。
//
//   有 Sequencer 之后：
//     · Sequence 只关心"生成什么数据"，不关心 Driver 的时序
//     · Driver 只关心"怎么驱动信号"，不关心数据是怎么生成的
//     · 多个 Sequence 可以同时跑，Sequencer 负责仲裁（先到先服务）
//
//   这是典型的"生产者-消费者解耦"模式。
//
// 【参数化的 uvm_sequencer】
//   `uvm_sequencer#(axi_transaction)` —— 参数指定了 transaction 类型。
//   这意味着本 Sequencer 只能调度 axi_transaction 类型的 sequence。
//   如果要用其他类型的 transaction，需要不同的 sequencer。
//
// 【当前版本】
//   本 Sequencer 的代码极少——只声明了一个空的 class。
//   这是因为 uvm_sequencer 基类已经实现了所有调度逻辑（仲裁、传递、握手）。
//   只有当需要自定义仲裁策略时，才需要覆写基类方法（如 user_priority_arbitration）。
//   默认的 FIFO 仲裁（先到先服务）对简单验证环境来说完全足够。
//
// 【面试要点】
//   Q: Sequence 和 Sequencer 的区别？
//   A: Sequence 是 object（动态），负责"生成什么数据"；
//      Sequencer 是 component（静态），负责"调度和传递"。
//      Sequence 调用 start(sequencer) 在指定 sequencer 上启动。
// ═══════════════════════════════════════════════════════════════════════════════

`include "uvm_macros.svh"
import uvm_pkg::*;

// ★ 参数化基类：uvm_sequencer#(axi_transaction)
//   指定本 Sequencer 调度 axi_transaction 类型的 sequence
class axi_sequencer extends uvm_sequencer#(axi_transaction);
    `uvm_component_utils(axi_sequencer)

    function new(string n, uvm_component p);
        super.new(n, p);
    endfunction
endclass
