// ============================================================
//  axi4_interconnect.v — 多主设备 AXI4-Lite 互连模块 (Interconnect)
//
//  【模块用途】
//  本模块替代原有的 system_bus.v，实现 SoC 内部总线互连。
//  在 2 个 AXI4-Lite 主设备（Manager）和 5 个 AXI4-Lite
//  从设备（Subordinate）之间完成地址解码、仲裁和通道路由。
//
//  【主设备】
//    - Manager 0 (M0): CPU 指令缓存 (I-Cache / 取指)
//    - Manager 1 (M1): CPU 数据缓存 (D-Cache / 访存)
//
//  【从设备】
//    - Port 0: BRAM   (片上块 RAM，用于指令/数据存储)
//    - Port 1: GPIO   (通用输入输出)
//    - Port 2: UART   (串口)
//    - Port 3: Timer  (定时器)
//    - Port 4: PLIC   (平台级中断控制器)
//
//  【地址映射 (Address Map)】
//    0x0000_0000 ~ 0x0FFF_FFFF — BRAM   (实际解码覆盖低 256MB)
//    0x2000_0000 ~ 0x20FF_FFFF — GPIO
//    0x3000_0000 ~ 0x30FF_FFFF — UART
//    0x4000_0000 ~ 0x40FF_FFFF — Timer
//    0x5000_0000 ~ 0x50FF_FFFF — PLIC
//
//  【顶层设计决策 — 为什么是全组合逻辑？】
//  本模块不包含任何时序元件（无 reg、无状态机），所有信号
//  均为 wire 类型，通过 assign 连续赋值实现。这不是疏忽，
//  而是针对 AXI4-Lite 场景的有意设计取舍：
//
//  1. 低延迟: 全组合路径意味着 valid/ready 握手可以在
//     单周期内从主设备穿透到从设备，无需等待流水线寄存器。
//     对于 CPU 取指路径（M0 → BRAM），这是关键时序瓶颈，
//     但避免了额外的指令延迟周期。
//
//  2. 无状态管理: AXI4-Lite 协议本身不要求 burst 计数、
//     ID 跟踪等复杂状态。写事务是 AW → W → B 三阶段，
//     读事务是 AR → R 两阶段。每阶段由一个 valid/ready
//     握手完成，无需跨周期状态保持。
//
//  3. 面积最小化: 无寄存器 = 无触发器开销。
//
//  设计代价：
//    - 无法处理需要跨周期仲裁的复杂场景（如 outstanding
//      transactions）。
//    - 对从设备要求"零等待"或接近零等待，因为主设备
//      ready 信号直接取决于从设备 ready。
//
//  【与 SoC 的集成】
//  - CPU 核通过 I-Cache 和 D-Cache 接口分别连接到 M0 和 M1。
//    I-Cache 取指走 M0 的 AR/R 通道；D-Cache 读/写走 M1 的
//    全部五通道。
//  - 外部中断通过 PLIC (Port 4) 汇聚，CPU 通过 M1 访问 PLIC
//    寄存器来响应中断。
//  - UART 和 GPIO 提供基本的调试和外设交互能力。
//  - Timer 提供系统节拍（tick）用于 OS 调度。
//
//  ============================================================
//  【通道架构】
//
//  AXI4-Lite 有五条独立通道：
//    AW  — 写地址 (Write Address)
//    W   — 写数据  (Write Data)
//    B   — 写响应  (Write Response)
//    AR  — 读地址  (Read Address)
//    R   — 读数据  (Read Data)
//
//  本模块对读通道和写通道**独立仲裁**。这意味着 M0 可以
//  同时执行对 BRAM 的读操作，而 M1 执行对 UART 的写操作。
//  读写互不阻塞。
//
//  ============================================================
//  【仲裁机制 (Arbitration) — 静态优先级】
//
//  采用固定的静态优先级方案：M0 > M1。
//
//  代码注释提到"round-robin arbitration"，但实际实现是
//  简单优先级。原因：
//    - I-Cache (M0) 取指是延迟敏感的关键路径，必须优先。
//    - D-Cache (M1) 访问频率通常低于取指，在大多数基准
//      测试中不会出现饿死（starvation）问题。
//    - 纯组合的轮询仲裁需要维护一个状态寄存器来记录
//      上一拍的仲裁结果，与本模块"零寄存器"的设计哲学
//      冲突。
//
//  仲裁表达式（以写通道为例；读通道完全对称）：
//    w_grant_m0 = m0_awvalid;
//    w_grant_m1 = m1_awvalid && !m0_awvalid;
//
//  解读：如果 M0 发起请求，则 M0 无条件获胜；仅当 M0 空闲时
//  M1 才获得授权。
//
//  ============================================================
//  【地址解码 (Address Decoding)】
//
//  通过 Verilog function addr_decode 完成。这是一个纯组合
//  函数，无时钟依赖。
//
//  解码策略：
//    - BRAM 使用 addr[31:28] 判断（4 位，解码低 256MB）
//    - 其他外设使用 addr[31:24] 判断（8 位，各占 16MB）
//    - 未命中任何地址的访问默认回退到 BRAM（Port 0）
//
//  注意：BRAM 的解码粒度为 256MB（addr[31:28]==4'h0），
//  比注释所述的"64KB"大得多。在 32 位地址空间中，
//  0x0000_0000 ~ 0x0FFF_FFFF 的所有地址都会命中 BRAM。
//  如果实际 BRAM 只有 64KB，高位地址会回绕。
//
//  ============================================================
//  【关键路径分析 (Critical Path)】
//
//  本模块的关键时序路径（以 M0 读 BRAM 为例）：
//
//  路径 1 — 前向地址路径 (AR):
//    m0_araddr → addr_decode() → r_sel_m0 (2 级比较器)
//    → r_target 选择 (2:1 mux)
//    → r_target == 3'd0 比较 (3 bit 比较器)
//    → s0_arvalid 输出
//    组合延迟约：函数调用(~0.3ns) + mux + 比较器 ≈ 0.8~1.5ns
//
//  路径 2 — Ready 回传路径:
//    s0_arready → (r_sel_m0 == 3'd0) && ... 多路选择 (5:1 mux)
//    → m0_arready 输出
//    组合延迟约：5:1 mux ≈ 0.5~1.0ns
//
//  路径 3 — 读数据回传路径:
//    s*_rdata[*], s*_rvalid → 5:1 mux (按 r_sel_m* 选择)
//    → m*_rdata, m*_rvalid
//    组合延迟约：5:1 mux ≈ 0.5~1.0ns
//
//  综合关键路径: m0_araddr → s0_arvalid → s0_arready → m0_arready
//  （前向 + 从设备 ready + 回传）
//  在 50~100MHz 典型 SoC 频率下，此路径通常在 3~6ns 内闭合，
//  即可满足 10~20ns 的时钟周期要求。若频率超过 200MHz，
//  可能需要插入流水线寄存器（Pipeline Register）。
//
//  ============================================================
//  【时序与停顿处理 (Timing & Stall Handling)】
//
//  本模块**不引入额外 stall 周期**。所有的背压（backpressure）
//  直接传递：
//
//  场景 A — 从设备忙：
//    sX_awready = 0 ⇒ m*_awready = 0（如果目标为该从设备）
//    主设备握手失败，必须保持 awvalid/awaddr 不变，
//    在下一个周期重试。这是 AXI4 协议的标准背压行为。
//
//  场景 B — 写数据跟随写地址：
//    W 通道的 w_target 信号复用了 AW 通道的 w_sel_m* 信号。
//    这意味着 W 数据与 AW 地址在**同一周期**被路由。
//    如果从设备的 AW 通道 ready 但 W 通道未 ready，
//    写事务会暂停在 W 阶段，直到从设备接收数据。
//    ——注意：W 通道未使用独立的地址解码，而是沿用 AW 的
//    w_sel 结果。这是合理的，因为 AW 和 W 的 valid 可以
//    不同周期断言（但 W 不能早于 AW）。
//
//  场景 C — 写响应（B 通道）独立完成：
//    B 通道的 bvalid/bready 握手完全独立于 AW/W 的时序。
//    从设备可以在收到写数据之后任意周期给出 B 响应。
//    本模块通过 w_sel_m*（基于 AW 地址）选择对应的
//    从设备 B 通道并路由回主设备。
//
//  ============================================================
//  【已知问题与局限 (Known Issues & Limitations)】
//
//  1.  [严重] 无地址冲突隔离（No Address Isolation）:
//      所有从设备的 awaddr/araddr 被广播（broadcast）了
//      同一个地址（取自已授权主设备的地址）。
//      例如，即使 M0 的目标是 BRAM，GPIO/UART/Timer/PLIC
//      的 awaddr 也会收到相同的地址值。虽然这些端口因
//      awvalid=0 不会锁存该地址，但会产生无用的信号跳变，
//      额外消耗 4 条 32 位总线的动态功耗。
//      改进方案：使用 AND 门将 awaddr 在 awvalid 无效时
//      强制拉到 0。
//
//  2.  [中等] 未映射地址静默回退（Silent Fallback）:
//      addr_decode 对未命中地址默认返回 BRAM（Port 0）。
//      如果 CPU 访问 0x6000_0000（无外设），事务会被
//      路由到 BRAM，可能导致数据损坏或非预期的读取结果。
//      AXI 规范通常要求此类访问返回 DECERR（解码错误），
//      但 AXI4-Lite 的 DECERR 是可选的。
//      改进方案：增加一个默认从设备返回 SLVERR 或 DECERR。
//
//  3.  [中等] BRAM 地址空间过大:
//      BRAM 使用 addr[31:28]==4'h0 作为判断条件，
//      实际占用了整个低 256MB 地址空间。如果 BRAM 实
//      例仅为 64KB，地址 [27:16] 可能被忽略或导致回绕。
//      建议修改为 addr[31:16]==16'h0000（64KB 精确匹配）。
//
//  4.  [轻微] 读/写通道仲裁独立可能导致顺序问题:
//      虽然 AMBA AXI4-Lite 规范保证同 ID 的有序性，
//      但读通道和写通道的独立仲裁意味着：
//        - 如果 M0 正在做写操作（AW 通道授权给 M0），
//          同时 M1 发起读请求（AR 通道），读通道可能
//          立即授权给 M1（因为 AR 和 AW 通道独立仲裁）。
//        - 这在 AXI4-Lite 中通常是合法的（读写独立），
//          但在某些需要强顺序的场景（如 MMIO）可能不会
//          完全符合预期。
//
//  5.  [轻微] w_sel 在 B 通道阶段的可靠性:
//      写响应（B 通道）的路由使用 w_sel_m*，而 w_sel_m*
//      是从 AW 地址解码得到的（连续组合信号）：
//        assign w_sel_m0 = addr_decode(m0_awaddr);
//      如果主设备在 AW 握手完成前撤除 awaddr 或改变了
//      awaddr（违反 AXI 规范），w_sel 会立即变化，
//      导致 B 通道的 mux 选择错误的从设备，可能丢失响应。
//      这要求主设备严格遵守 AXI 规范——在 awvalid 和 awready
//      同时为高之前，awaddr 必须保持稳定。
//      改进方案：在 AW 握手完成的拍寄存 w_sel，
//      后续 W/B 通道使用寄存后的值。
//
//  6.  [轻微] 无性能监控/调试接口:
//      没有事务计数器、延迟测量等调试辅助信号，
//      在集成调试时缺少可观测性。
//
//  ============================================================

module axi4_interconnect (
    input  wire clk,
    input  wire reset,

    // =======================================================
    //  Manager 0 (M0): I-Cache / 指令取指
    //  指令缓存通过此端口读取指令。
    //  写通道也在端口定义中（虽然 I-Cache 极少写），
    //  这是为了接口统一性，可能用于 cache 初始化或测试。
    // =======================================================

    // --- 写地址通道 (AW) ---
    input  wire [31:0] m0_awaddr,  input  wire m0_awvalid, output wire m0_awready,
    // --- 写数据通道 (W) ---
    input  wire [31:0] m0_wdata,   input  wire [3:0] m0_wstrb,
    input  wire m0_wvalid,         output wire m0_wready,
    // --- 写响应通道 (B) ---
    output wire [1:0] m0_bresp,    output wire m0_bvalid,  input  wire m0_bready,
    // --- 读地址通道 (AR) ---
    input  wire [31:0] m0_araddr,  input  wire m0_arvalid, output wire m0_arready,
    // --- 读数据通道 (R) ---
    output wire [31:0] m0_rdata,   output wire [1:0] m0_rresp,
    output wire m0_rvalid,         input  wire m0_rready,

    // =======================================================
    //  Manager 1 (M1): D-Cache / 数据访存
    //  数据缓存通过此端口进行 load/store 操作。
    //  与 I-Cache 不同，D-Cache 会频繁使用写通道。
    // =======================================================

    // --- 写地址通道 (AW) ---
    input  wire [31:0] m1_awaddr,  input  wire m1_awvalid, output wire m1_awready,
    // --- 写数据通道 (W) ---
    input  wire [31:0] m1_wdata,   input  wire [3:0] m1_wstrb,
    input  wire m1_wvalid,         output wire m1_wready,
    // --- 写响应通道 (B) ---
    output wire [1:0] m1_bresp,    output wire m1_bvalid,  input  wire m1_bready,
    // --- 读地址通道 (AR) ---
    input  wire [31:0] m1_araddr,  input  wire m1_arvalid, output wire m1_arready,
    // --- 读数据通道 (R) ---
    output wire [31:0] m1_rdata,   output wire [1:0] m1_rresp,
    output wire m1_rvalid,         input  wire m1_rready,

    // =======================================================
    //  Subordinate Ports: 5 个从设备接口
    //
    //  每个从设备拥有完整的 AXI4-Lite 五通道接口。
    //  从设备端（Subordinate side）的信号方向与
    //  主设备端（Manager side）相反：
    //    - address/data/valid:  互连模块 → 从设备 (output)
    //    - ready/resp:          从设备 → 互连模块 (input)
    // =======================================================

    // --- Port 0: BRAM (块 RAM) ---
    output wire [31:0] s0_awaddr,  output wire s0_awvalid, input wire s0_awready,
    output wire [31:0] s0_wdata,   output wire [3:0] s0_wstrb,
    output wire s0_wvalid,         input  wire s0_wready,
    input  wire [1:0] s0_bresp,    input  wire s0_bvalid,  output wire s0_bready,
    output wire [31:0] s0_araddr,  output wire s0_arvalid, input wire s0_arready,
    input  wire [31:0] s0_rdata,   input  wire [1:0] s0_rresp,
    input  wire s0_rvalid,         output wire s0_rready,

    // --- Port 1: GPIO ---
    output wire [31:0] s1_awaddr,  output wire s1_awvalid, input wire s1_awready,
    output wire [31:0] s1_wdata,   output wire [3:0] s1_wstrb,
    output wire s1_wvalid,         input  wire s1_wready,
    input  wire [1:0] s1_bresp,    input  wire s1_bvalid,  output wire s1_bready,
    output wire [31:0] s1_araddr,  output wire s1_arvalid, input wire s1_arready,
    input  wire [31:0] s1_rdata,   input  wire [1:0] s1_rresp,
    input  wire s1_rvalid,         output wire s1_rready,

    // --- Port 2: UART ---
    output wire [31:0] s2_awaddr,  output wire s2_awvalid, input wire s2_awready,
    output wire [31:0] s2_wdata,   output wire [3:0] s2_wstrb,
    output wire s2_wvalid,         input  wire s2_wready,
    input  wire [1:0] s2_bresp,    input  wire s2_bvalid,  output wire s2_bready,
    output wire [31:0] s2_araddr,  output wire s2_arvalid, input wire s2_arready,
    input  wire [31:0] s2_rdata,   input  wire [1:0] s2_rresp,
    input  wire s2_rvalid,         output wire s2_rready,

    // --- Port 3: Timer ---
    output wire [31:0] s3_awaddr,  output wire s3_awvalid, input wire s3_awready,
    output wire [31:0] s3_wdata,   output wire [3:0] s3_wstrb,
    output wire s3_wvalid,         input  wire s3_wready,
    input  wire [1:0] s3_bresp,    input  wire s3_bvalid,  output wire s3_bready,
    output wire [31:0] s3_araddr,  output wire s3_arvalid, input wire s3_arready,
    input  wire [31:0] s3_rdata,   input  wire [1:0] s3_rresp,
    input  wire s3_rvalid,         output wire s3_rready,

    // --- Port 4: PLIC (平台级中断控制器) ---
    output wire [31:0] s4_awaddr,  output wire s4_awvalid, input wire s4_awready,
    output wire [31:0] s4_wdata,   output wire [3:0] s4_wstrb,
    output wire s4_wvalid,         input  wire s4_wready,
    input  wire [1:0] s4_bresp,    input  wire s4_bvalid,  output wire s4_bready,
    output wire [31:0] s4_araddr,  output wire s4_arvalid, input wire s4_arready,
    input  wire [31:0] s4_rdata,   input  wire [1:0] s4_rresp,
    input  wire s4_rvalid,         output wire s4_rready
);

    // ============================================================
    //  地址解码函数 (Address Decode Function)
    //
    //  纯组合逻辑函数，根据 AXI 地址的高位判断目标从设备。
    //
    //  解码表:
    //    addr[31:28] == 4'h0        → Port 0 (BRAM)
    //    addr[31:24] == 8'h20       → Port 1 (GPIO)
    //    addr[31:24] == 8'h30       → Port 2 (UART)
    //    addr[31:24] == 8'h40       → Port 3 (Timer)
    //    addr[31:24] == 8'h50       → Port 4 (PLIC)
    //    其他                       → Port 0 (BRAM, 默认)
    //
    //  设计决策：
    //  - 使用高位地址比较（而非完整的地址范围检查），
    //    以最小化比较器逻辑的级数（减少组合路径深度）。
    //  - BRAM 使用 4 位比较以覆盖整个低 256MB，但实际
    //    BRAM 通常只有数 KB 到数十 KB。高位地址回绕
    //    到 BRAM 的基址（由 BRAM 内部处理）。
    // ============================================================
    function [2:0] addr_decode;
        input [31:0] addr;
        begin
            if (addr[31:28] == 4'h0)       addr_decode = 3'd0;  // BRAM
            else if (addr[31:24] == 8'h20) addr_decode = 3'd1;  // GPIO
            else if (addr[31:24] == 8'h30) addr_decode = 3'd2;  // UART
            else if (addr[31:24] == 8'h40) addr_decode = 3'd3;  // Timer
            else if (addr[31:24] == 8'h50) addr_decode = 3'd4;  // PLIC
            else                           addr_decode = 3'd0;  // 默认: BRAM
        end
    endfunction

    // ============================================================
    //  写通道路由 (Write Channel Routing)
    //
    //  AXI 写事务分三个阶段: AW → W → B
    //  本模块对此三个阶段分别路由。
    //
    //  仲裁策略: 静态优先级 M0 > M1
    //  当 M0 和 M1 同时请求时，M0 获得授权，
    //  M1 的 ready 信号保持为低，迫使 M1 等待。
    // ============================================================

    // --- 写地址解码：根据各自主设备的 AW 地址确定目标从设备 ---
    wire [2:0] w_sel_m0 = addr_decode(m0_awaddr);
    wire [2:0] w_sel_m1 = addr_decode(m1_awaddr);

    // --- 写通道仲裁（AW 阶段） ---
    // M0 只要发起请求即获胜（静态优先级最高）
    // M1 仅在 M0 空闲时获得授权
    wire w_grant_m0 = m0_awvalid;
    wire w_grant_m1 = m1_awvalid && !m0_awvalid;

    // --- AW 地址广播 ---
    // 将获胜主设备的 awaddr 广播给所有 5 个从设备。
    // 注意：所有从设备的 awaddr 接收相同的值。
    // 只有目标从设备的 awvalid 被置位（见下文），
    // 因此非目标从设备不会采样 awaddr。
    // 代价：非目标端口的总线仍在跳变，增加动态功耗。
    assign s0_awaddr  = w_grant_m0 ? m0_awaddr : m1_awaddr;
    assign s1_awaddr  = s0_awaddr;  // 广播，从设备通过 awvalid 选择
    assign s2_awaddr  = s0_awaddr;
    assign s3_awaddr  = s0_awaddr;
    assign s4_awaddr  = s0_awaddr;

    // --- AW 目标选择 ---
    // 根据仲裁结果和目标解码，确定当前事务的从设备编号
    wire [2:0] w_target = w_grant_m0 ? w_sel_m0 : w_sel_m1;

    // --- AW valid 门控 ---
    // 只有目标从设备的 awvalid 被置位；其他从设备的 awvalid 保持为 0
    assign s0_awvalid = (w_target == 3'd0) && (w_grant_m0 || w_grant_m1);
    assign s1_awvalid = (w_target == 3'd1) && (w_grant_m0 || w_grant_m1);
    assign s2_awvalid = (w_target == 3'd2) && (w_grant_m0 || w_grant_m1);
    assign s3_awvalid = (w_target == 3'd3) && (w_grant_m0 || w_grant_m1);
    assign s4_awvalid = (w_target == 3'd4) && (w_grant_m0 || w_grant_m1);

    // --- AW ready 回传 ---
    // 根据解码结果，将目标从设备的 awready 回传给获胜的主设备。
    // 未获胜的主设备 awready 恒为 0（因 grant 为 0）。
    // 多路选择器在每个主设备的 ready 路径上形成 5:1 mux。
    assign m0_awready = w_grant_m0 && (
        (w_sel_m0 == 3'd0 && s0_awready) || (w_sel_m0 == 3'd1 && s1_awready) ||
        (w_sel_m0 == 3'd2 && s2_awready) || (w_sel_m0 == 3'd3 && s3_awready) ||
        (w_sel_m0 == 3'd4 && s4_awready));
    assign m1_awready = w_grant_m1 && (
        (w_sel_m1 == 3'd0 && s0_awready) || (w_sel_m1 == 3'd1 && s1_awready) ||
        (w_sel_m1 == 3'd2 && s2_awready) || (w_sel_m1 == 3'd3 && s3_awready) ||
        (w_sel_m1 == 3'd4 && s4_awready));

    // --- W 数据通道路由 ---
    // W 通道的地址采用与 AW 相同的 w_target（即沿用 AW 阶段
    // 选中的从设备编号）。路由器假设 W 通道事务跟随 AW 通道，
    // 因此 w_target 在 W valid 期间是有效的。
    //
    // 注意：W 数据同样广播给所有从设备（仅目标从设备的 wvalid 有效）。
    assign s0_wdata   = w_grant_m0 ? m0_wdata : m1_wdata;
    assign s0_wstrb   = w_grant_m0 ? m0_wstrb : m1_wstrb;
    assign {s1_wdata,s2_wdata,s3_wdata,s4_wdata} = {4{s0_wdata}};
    assign {s1_wstrb,s2_wstrb,s3_wstrb,s4_wstrb} = {4{s0_wstrb}};
    assign s0_wvalid  = (w_target == 3'd0) && (w_grant_m0 || w_grant_m1);
    assign s1_wvalid  = (w_target == 3'd1) && (w_grant_m0 || w_grant_m1);
    assign s2_wvalid  = (w_target == 3'd2) && (w_grant_m0 || w_grant_m1);
    assign s3_wvalid  = (w_target == 3'd3) && (w_grant_m0 || w_grant_m1);
    assign s4_wvalid  = (w_target == 3'd4) && (w_grant_m0 || w_grant_m1);

    // --- W ready 回传（与 AW ready 逻辑对称） ---
    assign m0_wready  = w_grant_m0 && (
        (w_sel_m0 == 3'd0 && s0_wready) || (w_sel_m0 == 3'd1 && s1_wready) ||
        (w_sel_m0 == 3'd2 && s2_wready) || (w_sel_m0 == 3'd3 && s3_wready) ||
        (w_sel_m0 == 3'd4 && s4_wready));
    assign m1_wready  = w_grant_m1 && (
        (w_sel_m1 == 3'd0 && s0_wready) || (w_sel_m1 == 3'd1 && s1_wready) ||
        (w_sel_m1 == 3'd2 && s2_wready) || (w_sel_m1 == 3'd3 && s3_wready) ||
        (w_sel_m1 == 3'd4 && s4_wready));

    // --- B 写响应通道路由 ---
    // 写响应从目标从设备回传到发起写事务的主设备。
    // 路由选择基于 w_sel_m*（AW 地址解码结果），
    // 因为 B 响应必须来自与 AW/W 相同的从设备。
    //
    // 关键假设：在 B 通道握手完成前，w_sel_m* 保持稳定。
    // 如果主设备在 AW 完成前改变 awaddr（违反 AXI 协议），
    // w_sel_m* 会立即变化，导致 B 响应被错误路由。
    //
    // bresp/bvalid: 5:1 mux（从 5 个从设备中选择）
    assign m0_bresp   = (w_sel_m0 == 3'd0) ? s0_bresp : (w_sel_m0 == 3'd1) ? s1_bresp :
                         (w_sel_m0 == 3'd2) ? s2_bresp : (w_sel_m0 == 3'd3) ? s3_bresp : s4_bresp;
    assign m0_bvalid  = (w_sel_m0 == 3'd0) ? s0_bvalid : (w_sel_m0 == 3'd1) ? s1_bvalid :
                         (w_sel_m0 == 3'd2) ? s2_bvalid : (w_sel_m0 == 3'd3) ? s3_bvalid : s4_bvalid;
    assign m1_bresp   = (w_sel_m1 == 3'd0) ? s0_bresp : (w_sel_m1 == 3'd1) ? s1_bresp :
                         (w_sel_m1 == 3'd2) ? s2_bresp : (w_sel_m1 == 3'd3) ? s3_bresp : s4_bresp;
    assign m1_bvalid  = (w_sel_m1 == 3'd0) ? s0_bvalid : (w_sel_m1 == 3'd1) ? s1_bvalid :
                         (w_sel_m1 == 3'd2) ? s2_bvalid : (w_sel_m1 == 3'd3) ? s3_bvalid : s4_bvalid;

    // --- B ready 前向路由 ---
    // 从主设备的 bready 路由到对应从设备的 bready。
    // 这里的逻辑是"反向查找"——检查哪个主设备的目标是此从设备，
    // 然后转发该主设备的 bready。
    // 如果没有任何主设备的目标是此从设备，bready 保持为 0。
    assign s0_bready   = (w_sel_m0 == 3'd0) ? m0_bready : (w_sel_m1 == 3'd0) ? m1_bready : 1'b0;
    assign s1_bready   = (w_sel_m0 == 3'd1) ? m0_bready : (w_sel_m1 == 3'd1) ? m1_bready : 1'b0;
    assign s2_bready   = (w_sel_m0 == 3'd2) ? m0_bready : (w_sel_m1 == 3'd2) ? m1_bready : 1'b0;
    assign s3_bready   = (w_sel_m0 == 3'd3) ? m0_bready : (w_sel_m1 == 3'd3) ? m1_bready : 1'b0;
    assign s4_bready   = (w_sel_m0 == 3'd4) ? m0_bready : (w_sel_m1 == 3'd4) ? m1_bready : 1'b0;

    // ============================================================
    //  读通道路由 (Read Channel Routing)
    //
    //  AXI 读事务分两个阶段: AR → R
    //  读通道与写通道**完全独立**地仲裁和路由。
    //
    //  独立性好处：
    //    - M0 读 BRAM 的同时，M1 可以写 UART，互不阻塞。
    //    - 避免读操作被写操作阻塞（尤其是写响应慢的设备）。
    //
    //  独立性代价：
    //    - 可能导致 M0/M1 的读写操作在时间线上交错，
    //      在需要强顺序的场景（如 MMIO）需要注意。
    // ============================================================

    // --- 读地址解码：根据各自主设备的 AR 地址确定目标从设备 ---
    wire [2:0] r_sel_m0 = addr_decode(m0_araddr);
    wire [2:0] r_sel_m1 = addr_decode(m1_araddr);

    // --- 读通道仲裁（AR 阶段） ---
    // 与写通道相同的静态优先级逻辑：M0 > M1
    wire r_grant_m0 = m0_arvalid;
    wire r_grant_m1 = m1_arvalid && !m0_arvalid;

    // --- AR 地址广播（与 AW 广播逻辑对称） ---
    assign s0_araddr  = r_grant_m0 ? m0_araddr : m1_araddr;
    assign {s1_araddr,s2_araddr,s3_araddr,s4_araddr} = {4{s0_araddr}};

    // --- AR 目标选择 ---
    wire [2:0] r_target = r_grant_m0 ? r_sel_m0 : r_sel_m1;

    // --- AR valid 门控 ---
    assign s0_arvalid = (r_target == 3'd0) && (r_grant_m0 || r_grant_m1);
    assign s1_arvalid = (r_target == 3'd1) && (r_grant_m0 || r_grant_m1);
    assign s2_arvalid = (r_target == 3'd2) && (r_grant_m0 || r_grant_m1);
    assign s3_arvalid = (r_target == 3'd3) && (r_grant_m0 || r_grant_m1);
    assign s4_arvalid = (r_target == 3'd4) && (r_grant_m0 || r_grant_m1);

    // --- AR ready 回传（与 AW ready 逻辑对称） ---
    assign m0_arready = r_grant_m0 && (
        (r_sel_m0 == 3'd0 && s0_arready) || (r_sel_m0 == 3'd1 && s1_arready) ||
        (r_sel_m0 == 3'd2 && s2_arready) || (r_sel_m0 == 3'd3 && s3_arready) ||
        (r_sel_m0 == 3'd4 && s4_arready));
    assign m1_arready = r_grant_m1 && (
        (r_sel_m1 == 3'd0 && s0_arready) || (r_sel_m1 == 3'd1 && s1_arready) ||
        (r_sel_m1 == 3'd2 && s2_arready) || (r_sel_m1 == 3'd3 && s3_arready) ||
        (r_sel_m1 == 3'd4 && s4_arready));

    // --- R 读数据回传 ---
    // 根据读地址解码结果（r_sel_m*），将目标从设备的
    // rdata/rresp/rvalid 通过 5:1 mux 回传给对应主设备。
    //
    // 关键点：r_sel 基于 AR 地址，在整个读事务期间必须保持
    // 稳定。如果 CPU 在 AR 握手完成前改变 araddr（违反协议），
    // 读数据可能被错误路由。
    assign m0_rdata  = (r_sel_m0 == 3'd0) ? s0_rdata : (r_sel_m0 == 3'd1) ? s1_rdata :
                        (r_sel_m0 == 3'd2) ? s2_rdata : (r_sel_m0 == 3'd3) ? s3_rdata : s4_rdata;
    assign m0_rresp  = (r_sel_m0 == 3'd0) ? s0_rresp : (r_sel_m0 == 3'd1) ? s1_rresp :
                        (r_sel_m0 == 3'd2) ? s2_rresp : (r_sel_m0 == 3'd3) ? s3_rresp : s4_rresp;
    assign m0_rvalid = (r_sel_m0 == 3'd0) ? s0_rvalid : (r_sel_m0 == 3'd1) ? s1_rvalid :
                        (r_sel_m0 == 3'd2) ? s2_rvalid : (r_sel_m0 == 3'd3) ? s3_rvalid : s4_rvalid;
    assign m1_rdata  = (r_sel_m1 == 3'd0) ? s0_rdata : (r_sel_m1 == 3'd1) ? s1_rdata :
                        (r_sel_m1 == 3'd2) ? s2_rdata : (r_sel_m1 == 3'd3) ? s3_rdata : s4_rdata;
    assign m1_rresp  = (r_sel_m1 == 3'd0) ? s0_rresp : (r_sel_m1 == 3'd1) ? s1_rresp :
                        (r_sel_m1 == 3'd2) ? s2_rresp : (r_sel_m1 == 3'd3) ? s3_rresp : s4_rresp;
    assign m1_rvalid = (r_sel_m1 == 3'd0) ? s0_rvalid : (r_sel_m1 == 3'd1) ? s1_rvalid :
                        (r_sel_m1 == 3'd2) ? s2_rvalid : (r_sel_m1 == 3'd3) ? s3_rvalid : s4_rvalid;

    // --- R ready 前向路由（与 B ready 逻辑对称） ---
    assign s0_rready  = (r_sel_m0 == 3'd0) ? m0_rready : (r_sel_m1 == 3'd0) ? m1_rready : 1'b0;
    assign s1_rready  = (r_sel_m0 == 3'd1) ? m0_rready : (r_sel_m1 == 3'd1) ? m1_rready : 1'b0;
    assign s2_rready  = (r_sel_m0 == 3'd2) ? m0_rready : (r_sel_m1 == 3'd2) ? m1_rready : 1'b0;
    assign s3_rready  = (r_sel_m0 == 3'd3) ? m0_rready : (r_sel_m1 == 3'd3) ? m1_rready : 1'b0;
    assign s4_rready  = (r_sel_m0 == 3'd4) ? m0_rready : (r_sel_m1 == 3'd4) ? m1_rready : 1'b0;

endmodule
