// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  axi_transaction.sv — AXI4-Lite 总线事务类（Transaction）                      ║
// ╚══════════════════════════════════════════════════════════════════════════════════╝
//
// 【UVM 角色】
//   本类是 AXI4-Lite 总线的一次读/写操作的抽象表示，属于 UVM 中的 uvm_sequence_item。
//   它是 Sequence → Sequencer → Driver → Monitor → Scoreboard 全链路的数据载体。
//
//   在整个 UVM 验证环境中，Transaction 处于数据流的中心位置：
//     Sequence  负责"生成什么 transaction"   —— 随机化 addr/data/kind
//     Driver    负责"怎么把 transaction 发出去" —— 按 AXI 协议时序驱动信号
//     Monitor   负责"采样信号还原 transaction"  —— 从 DUT 接口重新组装 transaction
//     Scoreboard负责"比对 transaction"          —— 期望值 vs 实际值
//
// 【AXI4-Lite 协议背景】
//   AXI4-Lite 是 AXI4 协议的简化版，用于寄存器级的单次读写：
//     - 5 个独立通道：AW(写地址), W(写数据), B(写响应), AR(读地址), R(读数据+响应)
//     - 不支持 Burst 传输（每次只传 1 个数据）
//     - Size 编码：0=1 字节, 1=2 字节, 2=4 字节（对应 2^size 字节）
//     - 地址必须对齐到数据宽度（半字对齐到 bit0=0，字对齐到 bit1:0=0）
//
// 【字段说明】
//   addr[31:0]  — AXI 字节地址，映射到 SoC 外设地址空间（GPIO/UART/Timer/PLIC）
//   data[31:0]  — 读/写数据
//   kind        — 操作类型：0=Read, 1=Write
//   size[2:0]   — 传输宽度编码（AXI 标准 AxSIZE）:
//                   0 = 1 字节 (LB/SB)
//                   1 = 2 字节 (LH/SH)
//                   2 = 4 字节 (LW/SW)
//   wstrb[3:0]  — 字节写掩码（写操作时有效），每位对应 1 个字节通道
//                   Monitor 采样 DUT 实际使用的 wstrb 存入此字段
//   resp[1:0]   — AXI 响应码：00=OKAY, 10=SLVERR, 11=DECERR
//   done        — 事务完成标志（driver 在事务结束时置 1）
//   start_time  — 事务发起时间戳（$time）
//   end_time    — 事务完成时间戳（用于性能分析）
//
// 【约束设计思路】
//   1. legal_addr_c：限址到外设地址空间（避免越界访问产生未定义行为）
//   2. legal_size_c：AXI4-Lite 仅支持 0/1/2（1/2/4 字节），禁止 size=3+（保留）
//   3. addr_align_c：确保地址对齐——半字地址 bit0=0，字地址 bit1:0=0
//      AXI4-Lite 协议规定非对齐访问应返回 SLVERR/DECERR（但本 DUT 未全实现）
//
// 【uvm_sequence_item 继承链】
//   uvm_object → uvm_transaction → uvm_sequence_item → axi_transaction
//   继承 uvm_sequence_item 意味着本类可以被 sequencer 调度、被 driver 消费
//
// 【注】本 transaction 用于 AXI 主动驱动模式（AXI Agent 作为 Master）。
//       在固件驱动模式下，AXI 总线由 RISC-V core 主动发起，此时不使用本 transaction。
// ═══════════════════════════════════════════════════════════════════════════════

`include "uvm_macros.svh"
import uvm_pkg::*;

class axi_transaction extends uvm_sequence_item;
    // ★ 用 `uvm_object_utils 注册到 UVM Factory（非 component，故用 object_utils）
    `uvm_object_utils(axi_transaction)

    // ═══════════════════════════════════════════════════════════════════════
    //  数据字段
    // ═══════════════════════════════════════════════════════════════════════
    rand bit [31:0] addr;          // 目标地址（字节寻址，32-bit 全地址）
    rand bit [31:0] data;          // 读写数据（写=发送，读=接收）
    rand bit        kind;          // 操作类型：0=Read, 1=Write
    rand bit [2:0]  size;          // 传输宽度（AXI AxSIZE）：0=1B, 1=2B, 2=4B
    bit  [3:0]      wstrb;         // 实际字节写掩码（Monitor 捕获，非随机化）
    bit  [1:0]      resp;          // AXI 响应（Driver/Scoreboard 使用）
    bit             done;          // 事务完成标志
    time            start_time,    // 事务开始时间（$time 赋值）
                    end_time;      // 事务结束时间

    // ═══════════════════════════════════════════════════════════════════════
    //  随机约束（constraint）
    //
    //  约束在 Sequence 调用 randomize() 时自动生效。
    //  约束设计需平衡"覆盖全面"和"避免非法组合"。
    // ═══════════════════════════════════════════════════════════════════════

    // ── 约束 1：合法地址范围 ──
    //  将随机地址限制在 SoC 已实现的外设地址空间内。
    //  BRAM(0x0000_xxxx) 不在范围中——因为它由 RISC-V core 直接访问，
    //  不走 AXI Agent 通路。
    constraint legal_addr_c {
        addr inside {
            ['h20000000:'h2000FFFF],  // GPIO  (64KB 空间)
            ['h30000000:'h3000FFFF],  // UART  (64KB 空间)
            ['h40000000:'h4000FFFF],  // Timer (64KB 空间)
            ['h50000000:'h5000FFFF]   // PLIC  (64KB 空间)
        };
    }

    // ── 约束 2：合法传输宽度 ──
    //  AXI4-Lite 仅支持 1/2/4 字节（编码 0/1/2）。
    //  size≥3 为保留值，综合出的 DUT 不会实现对应逻辑。
    constraint legal_size_c {
        size inside {0, 1, 2};
    }

    // ── 约束 3：地址对齐 ──
    //  AXI4-Lite 协议要求地址对齐到传输宽度：
    //    size=0(1B)  → 无对齐要求
    //    size=1(2B)  → addr[0] == 0（半字对齐）
    //    size=2(4B)  → addr[1:0] == 0（字对齐）
    //  非对齐访问在 AXI4 规范中应返回 SLVERR/DECERR。
    constraint addr_align_c {
        (size == 1) -> (addr[0] == 0);         // 半字：bit0=0
        (size == 2) -> (addr[1:0] == 0);       // 字：bit1:0=0
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  构造函数与工具函数
    // ═══════════════════════════════════════════════════════════════════════
    function new(string name = "axi_transaction");
        super.new(name);
        kind = 0;
        size = 2;       // 默认 4 字节（覆盖大多数 LW/SW 场景）
        done = 0;
    endfunction

    // ── 格式化输出（方便 log 查看）──
    //  示例输出: "AXI: addr=0x40000000 data=0xDEADBEEF WR size=4B wstrb=0b1111 resp=00 done=1 t=1000-3000"
    virtual function string convert2string();
        int unsigned sz = 1 << size;          // size 编码转为实际字节数（0→1, 1→2, 2→4）
        return $sformatf("AXI: addr=0x%08h data=0x%08h %s size=%0dB wstrb=0b%04b resp=%0b done=%0b t=%0t-%0t",
                         addr, data, kind ? "WR" : "RD", sz, wstrb, resp, done, start_time, end_time);
    endfunction

endclass
