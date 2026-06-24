// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  bus_arbiter.v — 双主设备总线仲裁器（Dual-Master Bus Arbiter）                 ║
// ║                                                                            ║
// ║  【模块用途】                                                               ║
// ║  本模块是 system_bus.v 的升级替代版本，将原来的单主设备（CPU 直连）总线       ║
// ║  扩展为支持两个独立主设备的总线互连单元：                                     ║
// ║    - Master 0 (M0): I-Cache（指令缓存），只读设备，用于取指令                 ║
// ║    - Master 1 (M1): D-Cache（数据缓存），读写设备，用于 load/store            ║
// ║                                                                            ║
// ║  模块在两者之间进行仲裁，确保同一时刻只有一个主设备访问共享的从设备资源       ║
// ║  （BRAM、GPIO、UART、Timer、PLIC），同时为各自生成独立的应答 (ack) 和         ║
// ║  读数据返回通路。                                                            ║
// ║                                                                            ║
// ║  【设计动机 —— 为什么需要仲裁器】                                            ║
// ║  在原始 system_bus.v 中，CPU 核直接通过总线访问所有外设，只有一个主设备。    ║
// ║  引入 I-Cache 和 D-Cache 后，两个 Cache 各自拥有独立的总线主接口，可能       ║
// ║  在同一周期同时发起内存/外设访问请求。如果直接将两者并联到 system_bus，      ║
// ║  会产生多驱动器冲突（multiple drivers）和地址/数据竞争。因此需要一个          ║
// ║  仲裁器来：                                                                  ║
// ║    1. 决定哪个主设备获得总线使用权                                           ║
// ║    2. 将被授权主设备的地址/数据/控制信号路由到共享从设备                      ║
// ║    3. 将从设备的读数据和应答信号正确返回给对应主设备                          ║
// ║                                                                            ║
// ║  【存储器组织与地址映射】                                                     ║
// ║  与 system_bus.v 保持一致，但译码针对两个主设备分别进行：                     ║
// ║    0x00000000 — BRAM (块RAM)    m0_sel_bram / m1_sel_bram  高4位=0x0       ║
// ║    0x20000000 — GPIO            m0_sel_gpio / m1_sel_gpio  高8位=0x20      ║
// ║    0x30000000 — UART            m0_sel_uart / m1_sel_uart  高8位=0x30      ║
// ║    0x40000000 — Timer           m0_sel_timer/m1_sel_timer  高8位=0x40      ║
// ║    0x50000000 — PLIC            m0_sel_plic / m1_sel_plic  高8位=0x50      ║
// ║                                                                            ║
// ║  【仲裁策略：轮转优先级（Round-Robin）】                                      ║
// ║  设计采用带状态记忆的轮转仲裁，而非固定优先级。具体规则：                     ║
// ║    - 仅一方请求时：直接授权该方，无需切换状态                                  ║
// ║    - 双方同时请求时（conflict）：交替授权 —— 上次授权 M0 则本次授权 M1，     ║
// ║      反之亦然。通过 last_grant 寄存器记录上一次的授权结果                     ║
// ║    - 复位后首次冲突：last_grant 初始化为 1，使 M0 优先获得授权               ║
// ║                                                                            ║
// ║  为什么用轮转而不是固定优先级（如 Always I$ > D$）？                          ║
// ║    - 固定优先级可能导致 D$ 饥饿（starvation）：如果 I$ 持续有请求            ║
// ║      （如 CPU 在紧凑循环中取指），D$ 的 load/store 永远得不到响应             ║
// ║    - 轮转仲裁保证公平性：任意一方在最差情况下只需等待一个事务即可获得总线     ║
// ║    - 虽然指令获取在理论上更紧急（没有指令 CPU 无法执行），但 Cache 的         ║
// ║      命中率通常很高（>90%），实际冲突概率低，轮转仲裁的公平性优于             ║
// ║      固定优先级的确定性                                                        ║
// ║                                                                            ║
// ║  【读写时序与协议差异 —— BRAM vs 外设】                                      ║
// ║  本模块存在两种不同的时序路径，这是设计的核心复杂性所在：                     ║
// ║                                                                            ║
// ║  1. BRAM 访问（1 周期寄存器响应）：                                          ║
// ║     - BRAM 的读数据需要 1 个时钟周期才能返回（registered read）              ║
// ║     - 总线在第 N 周期发出地址和读请求 → BRAM 在第 N+1 周期返回数据           ║
// ║     - 因此需要一个寄存器流水级：bram_req_d, bram_rdata_reg, is_bram_d       ║
// ║     - ack 信号也在第 N+1 周期才有效（bram_ack = bram_req_d && !bram_we_d）  ║
// ║     - 写操作同理：bram_we 在当前周期有效，bram_write_ack 在下一周期有效      ║
// ║                                                                            ║
// ║  2. 外设访问（组合逻辑响应，0 等待）：                                        ║
// ║     - GPIO/UART/Timer/PLIC 均为组合读，地址有效后数据立即返回                ║
// ║     - ack 信号组合生成：m0_ack = m0_req（同周期）                            ║
// ║     - 无流水线延迟，请求与应答在同一周期完成                                  ║
// ║                                                                            ║
// ║  这种混合时序设计的原因是：                                                   ║
// ║    - BRAM 使用 FPGA 的 Block RAM 原语，其读操作必然是寄存输出的             ║
// ║      （BRAM 的 READ_FIRST 模式需要一个读时钟沿后才能输出数据）               ║
// ║    - 外设（GPIO/UART/Timer/PLIC）由寄存器构成，读路径是纯组合逻辑，          ║
// ║      可以在同一周期内完成                                                     ║
// ║                                                                            ║
// ║  【状态机说明】                                                               ║
// ║  本模块没有传统意义上的多状态 FSM，而是用以下控制机制组合实现仲裁：           ║
// ║                                                                            ║
// ║  (A) 仲裁状态寄存器：last_grant                                              ║
// ║      - 0 = 上一周期授权了 M0（I-Cache）                                     ║
// ║      - 1 = 上一周期授权了 M1（D-Cache）                                     ║
// ║      - 仅在双方同时请求（conflict）时翻转，实现轮转                           ║
// ║      - 单调请求（仅一方 req）时保持不变，减少不必要的翻转功耗                ║
// ║                                                                            ║
// ║  (B) 授权组合逻辑：grant_m0 / grant_m1                                       ║
// ║      - grant_m0 = m0_req && (!m1_req || last_grant == 1'b1)                ║
// ║        翻译：M0 有请求，且（M1 无请求 或 上次授权了 M1 → 该轮到 M0）         ║
// ║      - grant_m1 = m1_req && (!m0_req || last_grant == 1'b0)                ║
// ║        翻译：M1 有请求，且（M0 无请求 或 上次授权了 M0 → 该轮到 M1）         ║
// ║      - 当双方均无请求时，grant_m0 = grant_m1 = 0，总线空闲                   ║
// ║                                                                            ║
// ║  (C) BRAM 延迟流水线：bram_req_d, bram_we_d, bram_rdata_reg, is_bram_d     ║
// ║      - 这是隐含的一级流水状态，将 BRAM 请求延迟一个周期以匹配 BRAM 的        ║
// ║        寄存输出延迟                                                           ║
// ║      - is_bram_d 记录了延迟后的访问目标是否为 BRAM，用于 ack 和读数据        ║
// ║        多路复用的判断                                                         ║
// ║                                                                            ║
// ║  【关键路径分析】                                                             ║
// ║  1. 地址译码路径：m0_addr/m1_addr → sel → is_xxx → 外设地址端口             ║
// ║     延迟 = 地址比较器 + 多路选择器 + 扇出                                     ║
// ║                                                                            ║
// ║  2. 仲裁路径：m0_req/m1_req → last_grant → grant_mx → sel_xxx              ║
// ║     延迟 = 少量组合逻辑门（AND/OR/NOT），非关键路径                            ║
// ║                                                                            ║
// ║  3. 读数据返回路径（最长的组合路径）：                                        ║
// ║     外设 → gpio_rdata/uart_rdata/timer_rdata/plic_rdata → 多路复用器       ║
// ║     → rdata_comb → m0_rdata/m1_rdata                                        ║
// ║     这是潜在的时序瓶颈，尤其当外设数据路径有较长组合延迟时                    ║
// ║                                                                            ║
// ║  4. BRAM 读路径（寄存路径，非关键）：                                         ║
// ║     bram_rdata → bram_rdata_reg → rdata_comb → master rdata                ║
// ║     有一级寄存器打断，时序宽松                                                ║
// ║                                                                            ║
// ║  5. Ack 路径：                                                                ║
// ║     - BRAM ack：寄存器路径（bram_req_d → m0_ack/m1_ack），时序宽松          ║
// ║     - 外设 ack：组合路径（m0_req/m1_req → m0_ack/m1_ack），但仅经过        ║
// ║       always @(*) 中的 if-else，延迟可忽略                                  ║
// ║                                                                            ║
// ║  【已知问题与局限】                                                           ║
// ║                                                                            ║
// ║  1. 外设组合 ack 可能导致 Cache 端时序问题                                   ║
// ║     对于 GPIO/UART/Timer/PLIC 的读操作，ack 在同一周期返回（m0_ack =        ║
// ║     m0_req）。如果 I-Cache 或 D-Cache 的 FSM 期望 ack 在请求后至少 1       ║
// ║     个周期才有效，这种同周期 ack 可能导致 Cache 状态机跳转异常。             ║
// ║     实际上，ICache 和 DCache 的 miss FSM 在发出请求后等待 mem_ack，         ║
// ║     它们从 IDLE → REFILL/WRITE 已有寄存器跳转，所以下一周期看到 ack = 1    ║
// ║     是正常的。但如果 Cache 中存在同周期采样 ack 的路径，需注意此问题。      ║
// ║                                                                            ║
// ║  2. BRAM 读写冲突时的数据一致性                                              ║
// ║     当 D-Cache 写 BRAM 的某个地址后，I-Cache 立即读同一地址时：             ║
// ║     - BRAM 内部通常为 READ_FIRST 模式，读出的可能是旧数据                    ║
// ║     - 本模块没有实现写转发（write forwarding）或数据一致性维护逻辑          ║
// ║     - 上层软件/硬件需要保证 I-Cache 和 D-Cache 的一致性（如在修改代码后     ║
// ║       执行 fence.i 指令刷新 I-Cache）                                        ║
// ║                                                                            ║
// ║  3. 调试用的 $display 语句                                                  ║
// ║     第 133-139 行的 $display 在综合时会被忽略，但在仿真中会大量打印。        ║
// ║     在大规模仿真或回归测试中可能显著拖慢仿真速度。建议在正式发布前用         ║
// ║     `ifdef DEBUG 包裹或移除。                                                ║
// ║                                                                            ║
// ║  4. 无超时或死锁保护机制                                                      ║
// ║     如果某个主设备发出请求后一直得不到 ack（理论上不会发生，但如果 BRAM      ║
// ║     未正确连接或外设挂起），总线没有超时检测。请求将无限期等待。             ║
// ║     对于简单的 SoC 这不是问题，但在复杂系统中可能需要添加看门狗超时。       ║
// ║                                                                            ║
// ║  5. 地址空洞的默认行为                                                        ║
// ║     未映射的地址空间（如 0x10000000 ~ 0x1FFFFFFF）访问时：                  ║
// ║       - 所有 is_xxx 均为 0，rdata_comb = 32'h0                              ║
// ║       - 对外设的写使能也全为 0，不会误写入                                    ║
// ║       - 但 ack 仍然会返回（组合 ack = m0_req/m1_req），主设备认为访问成功  ║
// ║       - 这种静默失败（silent failure）可能导致软件难以调试非法地址访问      ║
// ║                                                                            ║
// ║  6. 仲裁公平性的边界情况                                                      ║
// ║     当双方持续请求且从不间断时，轮转仲裁严格交替授权，完全公平。             ║
// ║     但如果 M0 请求模式为"请求1周期 → 停1周期 → 请求1周期…"（即每两周期     ║
// ║     请求一次），而 M1 持续请求，则 M0 因为每次都在 M1 被授权后的下一个      ║
// ║     周期发出请求，可能总能抓住 last_grant==1 的条件而持续获得授权。          ║
// ║     这不是设计缺陷，而是轮转仲裁在非持续请求模式下的固有特性。               ║
// ║                                                                            ║
// ║  【与 SoC 的集成方式】                                                       ║
// ║  在 SoC 顶层（参见 sim/Makefile 中的 RTL 文件列表），bus_arbiter 取代了     ║
// ║  原来的 system_bus.v。它在以下层次中处于核心位置：                            ║
// ║                                                                            ║
// ║    I-Cache ──┬── bus_arbiter ──┬── BRAM                                    ║
// ║    D-Cache ──┘                 ├── GPIO                                    ║
// ║                                ├── UART                                    ║
// ║                                ├── Timer                                   ║
// ║                                └── PLIC                                    ║
// ║                                                                            ║
// ║  bus_arbiter 的上游是 I-Cache (icache.v) 和 D-Cache (dcache.v) 的        ║
// ║  存储器侧接口（mem_addr/mem_req/mem_ack 等），下游是各个外设的接口。        ║
// ║  与 system_bus.v 相比，主要接口变化：                                       ║
// ║    - 将单一的 cpu_* 接口拆分为 m0_* 和 m1_* 两组独立的主设备接口            ║
// ║    - 增加了 m1_wstrb（字节写使能）支持，因为 D-Cache 需要字节/半字写入    ║
// ║    - BRAM 接口移除了 mem_ready 输入（被内部 ack 生成逻辑替代）              ║
// ║    - 外设接口增加了写数据（xxx_wdata），因为不同主设备可能有不同写数据      ║
// ║      （虽然目前写数据仅来自 M1/D-Cache）                                    ║
// ║                                                                            ║
// ║  【与 axi4_interconnect.v 的关系】                                          ║
// ║  bus_arbiter 处理的是 SoC 内部总线（BRAM + 低速外设），运行在与 CPU        ║
// ║  相同的时钟域。对于外部存储器（如 DDR），另有 axi4_interconnect.v 或        ║
// ║  类似的 AXI 互连模块处理 AXI4 总线事务，bus_arbiter 不涉及 AXI 协议。      ║
// ╚══════════════════════════════════════════════════════════════════════════════╝

module bus_arbiter (
    input  wire clk,
    input  wire reset,

    // ═══════════════════════════════════════════════════════════════════
    // Master 0: I-Cache 接口（只读）
    //   m0_addr  — 32 位地址，I-Cache 发起取指请求的目标地址
    //   m0_req   — 读请求，高有效。I-Cache 是只读设备，仅有读事务
    //   m0_rdata — 32 位读数据，返回给 I-Cache 的指令数据
    //   m0_ack   — 应答信号，高有效表示当前事务完成（BRAM=1周期延迟，外设=同周期）
    //   注意：无 m0_wdata/m0_we/m0_wstrb，因为 I-Cache 只读取指令，不写
    // ═══════════════════════════════════════════════════════════════════
    input  wire [31:0] m0_addr,
    input  wire        m0_req,
    output wire [31:0] m0_rdata,
    output reg         m0_ack,

    // ═══════════════════════════════════════════════════════════════════
    // Master 1: D-Cache 接口（读写）
    //   m1_addr  — 32 位地址，D-Cache 发起 load/store 的目标地址
    //   m1_wdata — 32 位写数据，store 指令要写入的数据
    //   m1_wstrb — 4 位字节写选通，每一位对应 32 位数据中的一个字节
    //              bit[0]=byte0, bit[1]=byte1, bit[2]=byte2, bit[3]=byte3
    //              支持 sb/sh/sw 等不同宽度的写操作
    //   m1_re    — 读使能（load），高有效
    //   m1_we    — 写使能（store），高有效
    //   m1_rdata — 32 位读数据，返回给 D-Cache 的 load 结果
    //   m1_ack   — 应答信号，高有效表示当前事务完成
    //   注意：m1_re 和 m1_we 在同一周期只能有一个为高（由 D-Cache FSM 保证）
    // ═══════════════════════════════════════════════════════════════════
    input  wire [31:0] m1_addr,
    input  wire [31:0] m1_wdata,
    input  wire [3:0]  m1_wstrb,
    input  wire        m1_re,
    input  wire        m1_we,
    output wire [31:0] m1_rdata,
    output reg         m1_ack,

    // ═══════════════════════════════════════════════════════════════════
    // BRAM 块 RAM 接口
    //   bram_addr  — 地址，来自被授权主设备的 sel_addr
    //   bram_wdata — 写数据，来自 M1 (D-Cache)，因为只有 D-Cache 会写
    //   bram_wstrb — 字节写选通，透传自 M1
    //   bram_we    — 写使能，仅在 M1 被授权、M1 写请求、且地址命中 BRAM 时有效
    //   bram_rdata — 读数据，来自 BRAM 的组合输出，在下个时钟沿被寄存
    //   时序：读地址 → BRAM 组合输出 → bram_rdata_reg（下一周期） → master
    // ═══════════════════════════════════════════════════════════════════
    output wire [31:0] bram_addr,
    output wire [31:0] bram_wdata,
    output wire [3:0]  bram_wstrb,
    output wire        bram_we,
    input  wire [31:0] bram_rdata,

    // ═══════════════════════════════════════════════════════════════════
    // GPIO 通用输入输出接口
    //   地址空间：0x20000000 ~ 0x20FFFFFF (16MB)
    //   gpio_we 仅在 M1 被授权、写请求、地址命中 GPIO 时有效
    //   读数据为组合逻辑输出，由多路复用器选择
    // ═══════════════════════════════════════════════════════════════════
    output wire [31:0] gpio_addr,
    output wire [31:0] gpio_wdata,
    output wire        gpio_we,
    input  wire [31:0] gpio_rdata,

    // ═══════════════════════════════════════════════════════════════════
    // UART 串口接口
    //   地址空间：0x30000000 ~ 0x30FFFFFF (16MB)
    // ═══════════════════════════════════════════════════════════════════
    output wire [31:0] uart_addr,
    output wire [31:0] uart_wdata,
    output wire        uart_we,
    input  wire [31:0] uart_rdata,

    // ═══════════════════════════════════════════════════════════════════
    // Timer 定时器接口
    //   地址空间：0x40000000 ~ 0x40FFFFFF (16MB)
    // ═══════════════════════════════════════════════════════════════════
    output wire [31:0] timer_addr,
    output wire [31:0] timer_wdata,
    output wire        timer_we,
    input  wire [31:0] timer_rdata,

    // ═══════════════════════════════════════════════════════════════════
    // PLIC 平台级中断控制器接口
    //   地址空间：0x50000000 ~ 0x50FFFFFF (16MB)
    // ═══════════════════════════════════════════════════════════════════
    output wire [31:0] plic_addr,
    output wire [31:0] plic_wdata,
    output wire        plic_we,
    input  wire [31:0] plic_rdata
);

    // ╔══════════════════════════════════════════════════════════════════╗
    // ║  第一级：地址译码（Address Decode）                                ║
    // ║                                                                    ║
    // ║  对每个主设备独立译码，判断其请求地址属于哪个从设备。              ║
    // ║  两套译码逻辑完全对称，只是输入地址不同。                          ║
    // ║                                                                    ║
    // ║  译码粒度：                                                         ║
    // ║    - BRAM:  高 4 位（addr[31:28]），范围 0x0xxx_xxxx，256MB       ║
    // ║    - 外设:  高 8 位（addr[31:24]），各 16MB                       ║
    // ║                                                                    ║
    // ║  部分译码的性质：未使用的地址区域不匹配任何 sel_xxx，所有译码      ║
    // ║  信号均为 0，读数据默认为 0，写使能均无效。                        ║
    // ╚══════════════════════════════════════════════════════════════════╝
    wire m0_sel_bram  = (m0_addr[31:28] == 4'h0);          // M0 地址命中 BRAM
    wire m0_sel_gpio  = (m0_addr[31:24] == 8'h20);         // M0 地址命中 GPIO
    wire m0_sel_uart  = (m0_addr[31:24] == 8'h30);         // M0 地址命中 UART
    wire m0_sel_timer = (m0_addr[31:24] == 8'h40);         // M0 地址命中 Timer
    wire m0_sel_plic  = (m0_addr[31:24] == 8'h50);         // M0 地址命中 PLIC

    wire m1_sel_bram  = (m1_addr[31:28] == 4'h0);          // M1 地址命中 BRAM
    wire m1_sel_gpio  = (m1_addr[31:24] == 8'h20);         // M1 地址命中 GPIO
    wire m1_sel_uart  = (m1_addr[31:24] == 8'h30);         // M1 地址命中 UART
    wire m1_sel_timer = (m1_addr[31:24] == 8'h40);         // M1 地址命中 Timer
    wire m1_sel_plic  = (m1_addr[31:24] == 8'h50);         // M1 地址命中 PLIC

    // ╔══════════════════════════════════════════════════════════════════╗
    // ║  第二级：轮转仲裁（Round-Robin Arbitration）                       ║
    // ║                                                                    ║
    // ║  【仲裁状态寄存器：last_grant】                                    ║
    // ║    - 1'b0：上一次授权了 M0（I-Cache）                             ║
    // ║    - 1'b1：上一次授权了 M1（D-Cache）                             ║
    // ║    - 复位初始值为 1'b1，使 M0 在首次冲突时优先获得授权             ║
    // ║    - 仅在双方同时请求（conflict）时翻转，单调请求保持不变          ║
    // ║                                                                    ║
    // ║  【授权组合逻辑】                                                   ║
    // ║    grant_m0 = m0_req && (!m1_req || last_grant == 1'b1)           ║
    // ║    grant_m1 = m1_req && (!m0_req || last_grant == 1'b0)           ║
    // ║                                                                    ║
    // ║  真值表分析（假设 m0_req=1, m1_req=1, last_grant=0→上次M0赢）：  ║
    // ║    last_grant=0: grant_m0 = 1 && (0 || 0) = 0    M0 输            ║
    // ║                  grant_m1 = 1 && (0 || 1) = 1    M1 赢 ✓          ║
    // ║    last_grant=1: grant_m0 = 1 && (0 || 1) = 1    M0 赢 ✓          ║
    // ║                  grant_m1 = 1 && (0 || 0) = 0    M1 输            ║
    // ║    → 交替授权，公平轮转                                             ║
    // ║                                                                    ║
    // ║  当仅一方请求时（如 m0_req=1, m1_req=0）：                         ║
    // ║    grant_m0 = 1 && (1 || X) = 1  ✓                                ║
    // ║    grant_m1 = 0 && anything = 0                                   ║
    // ║    → 直接授权请求方，无需仲裁竞争                                   ║
    // ║                                                                    ║
    // ║  【conflict 信号的生成】                                            ║
    // ║    conflict = m0_req && m1_req：双方同时请求时为 1                 ║
    // ║    仅在 conflict=1 时翻转 last_grant，避免不必要的状态切换          ║
    // ╚══════════════════════════════════════════════════════════════════╝

    // M1 的总请求信号：读请求或写请求任一个有效即为有请求
    // 注意：m1_re 和 m1_we 同一周期最多一个为 1（由 D-Cache FSM 保证互斥）
    wire m1_req = m1_re || m1_we;

    reg  last_grant;   // 仲裁状态寄存器：0=M0上次赢，1=M1上次赢

    // 授予 M0 的条件：M0 有请求 且 (M1 无请求 或者 上次 M1 赢了 → 轮到 M0)
    wire grant_m0 = m0_req && (!m1_req || last_grant == 1'b1);

    // 授予 M1 的条件：M1 有请求 且 (M0 无请求 或者 上次 M0 赢了 → 轮到 M1)
    wire grant_m1 = m1_req && (!m0_req || last_grant == 1'b0);

    // 冲突检测：双方同时请求。仅在冲突时翻转 last_grant 实现轮转
    wire conflict = m0_req && m1_req;

    // last_grant 状态更新：
    //   复位时初始化为 1（让 M0 首次获得授权）
    //   冲突时翻转（实现轮转：上次M0赢→下次M1赢，上次M1赢→下次M0赢）
    //   无冲突时保持不变（减少不必要的翻转功耗）
    always @(posedge clk or posedge reset) begin
        if (reset)        last_grant <= 1'b1;  // M0 在复位后首次冲突中优先
        else if (conflict) last_grant <= ~last_grant;
    end

    // ╔══════════════════════════════════════════════════════════════════╗
    // ║  第三级：信号路由（Signal Routing / Multiplexing）                 ║
    // ║                                                                    ║
    // ║  根据仲裁结果（grant_m0 / grant_m1），将被授权主设备的地址和        ║
    // ║  译码结果选择为有效信号，用于驱动下游从设备。                       ║
    // ║                                                                    ║
    // ║  sel_addr: 被选中的主设备的地址，驱动所有从设备的地址端口           ║
    // ║  is_xxx:   被选中的主设备的译码结果，用于控制读数据多路复用和       ║
    // ║            写使能门控                                                ║
    // ║                                                                    ║
    // ║  注意：写数据（m1_wdata）和字节选通（m1_wstrb）直接来自 M1，       ║
    // ║  不需要多路选择，因为 M0（I-Cache）没有写数据通路。                 ║
    // ╚══════════════════════════════════════════════════════════════════╝

    // 被授权主设备的地址
    wire [31:0] sel_addr  = grant_m0 ? m0_addr : m1_addr;

    // 被授权主设备命中的从设备类型（用于后续的读数据选择和写使能门控）
    wire        is_bram   = grant_m0 ? m0_sel_bram  : m1_sel_bram;
    wire        is_gpio   = grant_m0 ? m0_sel_gpio  : m1_sel_gpio;
    wire        is_uart   = grant_m0 ? m0_sel_uart  : m1_sel_uart;
    wire        is_timer  = grant_m0 ? m0_sel_timer : m1_sel_timer;
    wire        is_plic   = grant_m0 ? m0_sel_plic  : m1_sel_plic;

    // ╔══════════════════════════════════════════════════════════════════╗
    // ║  BRAM 接口信号生成（含 1 周期流水线寄存器）                        ║
    // ║                                                                    ║
    // ║  BRAM 的读时序要求：地址在当前周期给出 → 数据在下一周期返回。      ║
    // ║  为此，总线内部需要一级流水寄存器来对齐延迟：                      ║
    // ║                                                                    ║
    // ║  周期 N（请求发起）：                                               ║
    // ║    - bram_addr = sel_addr（被授权主设备的地址）                    ║
    // ║    - bram_we   = grant_m1 && m1_we && is_bram（写使能）           ║
    // ║    - bram_wdata / bram_wstrb 也在此周期有效                        ║
    // ║    - BRAM 内部：如果是读，地址被寄存，数据从存储器阵列读出          ║
    // ║                                                                    ║
    // ║  周期 N+1（数据返回）：                                             ║
    // ║    - bram_rdata 输出周期 N 请求的读数据                             ║
    // ║    - bram_rdata_reg <= bram_rdata（寄存 BRAM 输出）               ║
    // ║    - bram_req_d / bram_we_d / is_bram_d 延迟一级后用于生成 ack    ║
    // ║    - bram_ack = bram_req_d && !bram_we_d（读应答有效）            ║
    // ║                                                                    ║
    // ║  【为什么 bram_we 也用 grant_m1 限定？】                            ║
    // ║    M0（I-Cache）只读不写，BRAM 写操作只可能来自 M1（D-Cache）。    ║
    // ║    grant_m1 确保写使能仅在 M1 被授权时有效，防止 M0 获得授权时     ║
    // ║    由于某些意外原因产生 BRAM 写操作。                               ║
    // ║    bram_wdata 和 bram_wstrb 直接来自 m1_* —— 当 M0 被授权时，     ║
    // ║    bram_we=0，所以写数据和选通的值无关紧要（BRAM 不会采样）。      ║
    // ╚══════════════════════════════════════════════════════════════════╝

    // BRAM 组合信号：地址/写数据/写选通直接输出（地址在当前周期需要稳定）
    assign bram_addr  = sel_addr;                          // BRAM 地址 = 被授权主设备地址
    assign bram_wdata = m1_wdata;                          // 写数据仅来自 M1 (D-Cache)
    assign bram_wstrb = m1_wstrb;                          // 字节选通仅来自 M1
    assign bram_we    = grant_m1 && m1_we && is_bram;      // 仅 M1 被授权 + 写请求 + 命中 BRAM

    // BRAM 延迟流水线寄存器：将请求信息延迟一个周期以匹配 BRAM 的寄存输出延迟
    //   周期 N:    请求发起，这些寄存器捕获请求相关信息
    //   周期 N+1:  数据返回，用这些延迟信号生成 ack 和读数据选择
    reg        bram_req_d;       // 延迟的 BRAM 读请求（指示上一周期是否有 BRAM 读）
    reg        bram_we_d;        // 延迟的 BRAM 写请求（指示上一周期是否有 BRAM 写）
    reg [31:0] bram_rdata_reg;   // 寄存的 BRAM 读数据（BRAM 输出在下一周期被捕获）
    reg        is_bram_d;        // 延迟的目标指示（上一周期访问的目标是否为 BRAM）

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            bram_req_d     <= 1'b0;
            bram_we_d      <= 1'b0;
            bram_rdata_reg <= 32'h0;
            is_bram_d      <= 1'b0;
        end
        else begin
            // bram_req_d: 延迟一级的 BRAM 访问请求
            //   M0 被授权 + M0 有读请求 + 命中 BRAM → I-Cache 读 BRAM
            //   或 M1 被授权 + M1 有请求（读或写）+ 命中 BRAM → D-Cache 访问 BRAM
            bram_req_d <= (grant_m0 && m0_req && is_bram) ||
                          (grant_m1 && (m1_re || m1_we) && is_bram);

            // bram_we_d: 延迟一级的 BRAM 写指示
            bram_we_d  <= grant_m1 && m1_we && is_bram;

            // bram_rdata_reg: 在周期 N+1 捕获 BRAM 在周期 N 输出的读数据
            //   此时 bram_rdata 对应周期 N 发出的地址的读结果
            bram_rdata_reg <= bram_rdata;

            // is_bram_d: 延迟一级的目标类型，用于后续的 ack 和读数据多路复用
            is_bram_d  <= is_bram;
        end
    end

    // ╔══════════════════════════════════════════════════════════════════╗
    // ║  外设接口信号生成（GPIO / UART / Timer / PLIC）                   ║
    // ║                                                                    ║
    // ║  所有外设接口采用统一的信号生成模式：                              ║
    // ║    - 地址：广播 sel_addr（被授权主设备的地址）                     ║
    // ║    - 写数据：直接来自 M1（D-Cache），因为只有 D-Cache 会写外设    ║
    // ║    - 写使能：grant_m1 && m1_we && is_xxx                          ║
    // ║              仅 M1 被授权 + M1 写请求 + 地址命中该外设时有效      ║
    // ║    - 读数据：各自外设的组合输出，由多路复用器选择                  ║
    // ║                                                                    ║
    // ║  为什么外设没有读使能信号？                                        ║
    // ║    外设（GPIO/UART/Timer/PLIC）均为寄存器构成的组合读外设，       ║
    // ║    其数据输出始终有效（rdata 端口始终反映当前寄存器的值）。        ║
    // ║    读操作仅需通过多路复用器选择对应的 rdata 即可，不需要单独的    ║
    // ║    读使能来触发外设内部动作。这与 BRAM 不同（BRAM 有 mem_re 用于  ║
    // ║    内部读脉冲控制）。                                              ║
    // ║                                                                    ║
    // ║  为什么写使能三重限定（grant_m1 && m1_we && is_xxx）？             ║
    // ║    - grant_m1：确保只有 M1 被授权时才可能写（M0 被授权时写使能=0）║
    // ║    - m1_we：确保只有写操作时才有效（读操作时写使能=0）             ║
    // ║    - is_xxx：确保只有地址命中此外设时才有效（防止误写其他外设）   ║
    // ║    三重防护确保外设寄存器不会被意外修改。                          ║
    // ╚══════════════════════════════════════════════════════════════════╝

    assign gpio_addr  = sel_addr;
    assign gpio_wdata = m1_wdata;
    assign gpio_we    = grant_m1 && m1_we && is_gpio;

    // ★ 调试用的 $display：在仿真中打印 M0/M1 的每次读写操作
    //   注意：综合工具会忽略 $display，仅在仿真中生效。
    //   如果仿真日志量过大，可考虑用 `ifdef DEBUG 包裹或降低打印频率。
    always @(posedge clk) begin
        if (grant_m1 && m1_we)
            $display("[ARB] M1 WRITE addr=0x%08h data=0x%08h strb=%b is_gpio=%0b gpio_we=%0b grant=%0b",
                     m1_addr, m1_wdata, m1_wstrb, is_gpio, gpio_we, grant_m1);
        if (grant_m0 && m0_req)
            $display("[ARB] M0 READ  addr=0x%08h grant=%0b", m0_addr, grant_m0);
    end

    assign uart_addr  = sel_addr;
    assign uart_wdata = m1_wdata;
    assign uart_we    = grant_m1 && m1_we && is_uart;

    assign timer_addr = sel_addr;
    assign timer_wdata = m1_wdata;
    assign timer_we   = grant_m1 && m1_we && is_timer;

    assign plic_addr  = sel_addr;
    assign plic_wdata = m1_wdata;
    assign plic_we    = grant_m1 && m1_we && is_plic;

    // ╔══════════════════════════════════════════════════════════════════╗
    // ║  读数据多路复用（Read Data Multiplexer）                           ║
    // ║                                                                    ║
    // ║  rdata_comb: 从设备读数据的组合多路复用器                          ║
    // ║    优先级顺序：BRAM > GPIO > UART > Timer > PLIC > 0             ║
    // ║    BRAM 的数据使用寄存版本（bram_rdata_reg），因为 BRAM 需要      ║
    // ║    一个周期的延迟才能输出读数据                                    ║
    // ║    外设的数据使用组合版本（xxx_rdata），因为它们是组合读           ║
    // ║                                                                    ║
    // ║  注意：rdata_comb 同时包含了 BRAM 的寄存数据和外设的组合数据，    ║
    // ║  这是因为在周期 N+1（BRAM 返回数据时），外设的组合读数据仍然是    ║
    // ║  有效的（外设地址在周期 N+1 也给出）。但在实际使用中，is_bram    ║
    // ║  和 is_gpio 等互斥，只有一路数据有效。                            ║
    // ║                                                                    ║
    // ║  默认值 32'h0：未命中任何从设备时返回全 0（静默失败）             ║
    // ╚══════════════════════════════════════════════════════════════════╝
    wire [31:0] rdata_comb = is_bram  ? bram_rdata_reg :
                              is_gpio  ? gpio_rdata :
                              is_uart  ? uart_rdata :
                              is_timer ? timer_rdata :
                              is_plic  ? plic_rdata : 32'h0;

    // ╔══════════════════════════════════════════════════════════════════╗
    // ║  Ack 信号生成（Acknowledge Generation）                           ║
    // ║                                                                    ║
    // ║  【BRAM 的 ack 生成（1 周期延迟）】                                ║
    // ║    bram_ack       = bram_req_d && !bram_we_d                     ║
    // ║      上一周期有 BRAM 读请求 → 本周期 ack 有效                      ║
    // ║    bram_write_ack = bram_we_d                                     ║
    // ║      上一周期有 BRAM 写请求 → 本周期 ack 有效                      ║
    // ║    区分读写是为了在 M1 写 BRAM 时正确生成 ack                      ║
    // ║                                                                    ║
    // ║  【外设的 ack 生成（同周期，组合逻辑）】                           ║
    // ║    外设（GPIO/UART/Timer/PLIC）的读操作在同周期返回 ack：         ║
    // ║      m0_ack = m0_req（当 grant_m0 且不是 BRAM 时）               ║
    // ║      m1_ack = m1_req（当 grant_m1 且不是 BRAM 时）               ║
    // ║    这样做的假设是外设的读数据在组合逻辑延迟内已就绪。             ║
    // ║                                                                    ║
    // ║  【always @(*) 组合逻辑实现】                                      ║
    // ║    使用组合 always 块而非 assign，因为 ack 的生成涉及多个条件     ║
    // ║    分支（granted/not granted, BRAM/not BRAM, read/write）。       ║
    // ║    使用 always @(*) 可以避免多个 assign 语句之间的优先级歧义。    ║
    // ║    注意：default 赋值为 0，确保未覆盖的分支中 ack 不会被锁存。    ║
    // ║                                                                    ║
    // ║  【关键时序注意】                                                   ║
    // ║    - BRAM 读：周期 N 发请求 → 周期 N+1 收 ack 和数据              ║
    // ║    - BRAM 写：周期 N 发请求 → 周期 N+1 收 ack                     ║
    // ║    - 外设读：周期 N 发请求 → 周期 N 同周期收 ack 和数据            ║
    // ║    - 外设写：周期 N 发请求 → 周期 N 同周期收 ack（数据在          ║
    // ║              下一时钟沿被外设采样）                                ║
    // ║    这种 BRAM 和 外设之间 ack 时序的不一致是设计的核心权衡：       ║
    // ║    简单性（统一的接口）vs 性能（BRAM 必须寄存读）。               ║
    // ║    I-Cache 和 D-Cache 的 FSM 通过 mem_ack 信号等待，无论是        ║
    // ║    1 周期还是同周期 ack，Cache 都能正确响应（Cache FSM 在请求     ║
    // ║    后进入等待状态，等到 ack=1 后进入下一状态）。                  ║
    // ╚══════════════════════════════════════════════════════════════════╝

    // BRAM ack 信号（基于延迟一级的请求信息）
    wire bram_ack       = bram_req_d && !bram_we_d;   // BRAM 读应答：上一周期有读请求
    wire bram_write_ack = bram_we_d;                   // BRAM 写应答：上一周期有写请求

    // ╔══════════════════════════════════════════════════════════════════╗
    // ║  读数据返回到主设备                                                  ║
    // ║                                                                    ║
    // ║  m0_rdata (I-Cache 读数据)：                                        ║
    // ║    - M0 被授权 + 访问 BRAM：返回 bram_rdata_reg（寄存的 BRAM 数据） ║
    // ║    - M0 被授权 + 访问外设：返回 rdata_comb（组合读数据）            ║
    // ║    - M0 未被授权：返回 0                                            ║
    // ║    注意：BRAM 读数据 bram_rdata_reg 已经在 rdata_comb 的顶部，     ║
    // ║    但为了清晰性，这里显式区分 BRAM 和外设路径。                     ║
    // ║                                                                    ║
    // ║  m1_rdata (D-Cache 读数据)：                                        ║
    // ║    - M1 被授权：返回 rdata_comb（已包含 BRAM 寄存数据和外设组合数据）║
    // ║    - M1 未被授权：返回 0                                            ║
    // ║    由于 rdata_comb 已经用 is_bram/is_gpio/... 选择了正确的数据，  ║
    // ║    这里不需要再次区分 BRAM 和外设。                                 ║
    // ╚══════════════════════════════════════════════════════════════════╝
    assign m0_rdata = grant_m0 ? (is_bram ? bram_rdata_reg : rdata_comb) : 32'h0;
    assign m1_rdata = grant_m1 ? rdata_comb : 32'h0;

    // ╔══════════════════════════════════════════════════════════════════╗
    // ║  Ack 组合逻辑（always @(*)）                                       ║
    // ║                                                                    ║
    // ║  逻辑分解：                                                         ║
    // ║                                                                    ║
    // ║  对于 M0（I-Cache，只读）：                                         ║
    // ║    if (grant_m0)                                                    ║
    // ║      if (is_bram)  → m0_ack = bram_ack（延迟 1 周期的读应答）     ║
    // ║      else          → m0_ack = m0_req（同周期的组合应答）           ║
    // ║                                                                    ║
    // ║  对于 M1（D-Cache，读写）：                                         ║
    // ║    if (grant_m1)                                                    ║
    // ║      if (is_bram)                                                   ║
    // ║        if (m1_we) → m1_ack = bram_write_ack（延迟 1 周期写应答）  ║
    // ║        else       → m1_ack = bram_ack（延迟 1 周期读应答）        ║
    // ║      else          → m1_ack = m1_req（同周期的组合应答）           ║
    // ║                                                                    ║
    // ║  未被授权的主设备：ack = 0（默认值）                                ║
    // ╚══════════════════════════════════════════════════════════════════╝
    always @(*) begin
        // 默认值：两个主设备均无应答
        m0_ack = 1'b0;
        m1_ack = 1'b0;

        // === M0 (I-Cache) 应答生成 ===
        if (grant_m0) begin
            if (is_bram) begin
                // I-Cache 只从 BRAM 读指令，使用延迟 1 周期的读应答
                // bram_ack = bram_req_d && !bram_we_d（上一周期有读请求）
                m0_ack = bram_ack;
            end
            else begin
                // 访问外设（GPIO/UART/Timer/PLIC）：同周期组合应答
                // 外设的读数据在同周期就绪，不需要等待
                m0_ack = m0_req;  // 1 周期完成
            end
        end

        // === M1 (D-Cache) 应答生成 ===
        if (grant_m1) begin
            if (is_bram) begin
                // D-Cache 访问 BRAM 时区分读写：
                //   写操作 → bram_write_ack（上一周期的 bram_we_d）
                //   读操作 → bram_ack（上一周期的 bram_req_d 且非写）
                m1_ack = m1_we ? bram_write_ack : bram_ack;
            end
            else begin
                // 访问外设：同周期组合应答
                m1_ack = m1_req;
            end
        end
    end

endmodule
