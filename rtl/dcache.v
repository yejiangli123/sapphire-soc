// ============================================================
//  dcache.v — 直接映射数据 Cache（写通 + 写不分配）
//  Direct-Mapped Data Cache (Write-Through, No-Write-Allocate)
//
//  【模块用途 Module Purpose】
//    为 Sapphire SoC RV32IM 处理器核心提供数据缓存加速。
//    CPU 的所有 load/store 指令通过本模块访问内存：
//      · 读命中  → 零等待周期返回数据（组合逻辑路径）
//      · 写命中  → 同时更新 cache 和内存（写通），等待 1 次 mem_ack
//      · 读缺失  → 从内存突发读取整行（8 字 = 32 字节），逐字填充
//      · 写缺失  → 不分配 cache 行，直接写内存（写不分配策略）
//
//  【设计决策 Design Decisions】
//    1. 直接映射（Direct-Mapped）
//       — 硬件开销最小：每行只需 1 个 tag 比较器 + 1 个 valid 位
//       — 无替换策略：缺失时直接覆盖对应 index 的旧行
//       — 代价：同一 index 的不同地址会发生冲突缺失（conflict miss）
//       — 对本 SoC 的小型测试程序（固件通常 < 32 KB）而言足够
//
//    2. 写通 + 写不分配（Write-Through + No-Write-Allocate）
//       — 写通：每次写操作同时更新内存，保证内存数据始终最新
//       — 优点：无需 dirty 回写逻辑，大幅简化状态机和控制通路
//       — 优点：无"写回风暴"——脏行驱逐不会产生突发总线事务
//       — 缺点：每次写都要等待总线 mem_ack，写延迟较高
//       — 写不分配：写缺失时不加载整行到 cache，避免为单次写而驱逐有用数据
//
//    3. 阻塞式设计（Blocking Cache）
//       — 在处理写通或读缺失期间，cpu_ready = 0，CPU 流水线暂停
//       — 简单可靠，无乱序/非阻塞 cache 的复杂 bookkeeping
//
//    4. 参数化结构
//       — NUM_LINES = 64, LINE_WORDS = 8 (32 字节/行), 总容量 = 2 KB
//       — TAG/INDEX/WORD 位宽从 Def.v 宏定义，与 icache 一致
//
//  【地址映射 Address Mapping】
//    |<─── TAG (21 bits) ───>|<─ INDEX (6) ─>|<─ WORD (3) ─>|<─ BYTE (2) ─>|
//    |31                    11|10            5|4            2|1           0|
//    TAG:   高位地址，用于命中比较
//    INDEX: 选择 64 行中的一行
//    WORD:  行内 8 个字（32-bit word）的选择
//    BYTE:  字内字节偏移（用于写字节使能，读时忽略）
//    基地址: {cpu_addr[31:WORD_BITS+2], {(WORD_BITS+2){1'b0}}} 即行对齐地址
//
//  【状态机 State Machine】
//    共 4 个状态，编码为 3 位独热（实际是二进制 3'd0~3'd3）：
//
//    ┌──────────────────────────────────────────────────────────────┐
//    │  S_IDLE (3'd0)  — 空闲 / 命中服务                              │
//    │    读命中 → 组合逻辑直接返回 cpu_rdata = data[index][word]     │
//    │             cpu_ready = 1，CPU 不暂停                          │
//    │    写请求 → 跳转到 S_WRITE（优先于读请求）                     │
//    │    读缺失 → 跳转到 S_RD_MISS                                  │
//    │    ★ 同时有读写请求时，写优先（if-else 顺序决定）              │
//    └──────────────────────────────────────────────────────────────┘
//         │                          │
//         ▼                          ▼
//    ┌──────────────────────┐  ┌──────────────────────────────────────┐
//    │ S_WRITE (3'd1)       │  │ S_RD_MISS (3'd2)                     │
//    │   等待 mem_ack       │  │   已发出第 0 字读请求（mem_re=1）      │
//    │   mem_ack 到来 →     │  │   mem_ack 到来 →                      │
//    │   mem_we 清零        │  │   data[miss_index][0] <= mem_rdata    │
//    │   返回 S_IDLE        │  │   跳转到 S_RD_REFILL                 │
//    │                      │  │   ★ 同时 cpu_rdata 直通 mem_rdata     │
//    │   ★ 此状态不区分     │  │     （第 0 字旁路，CPU 立即得到数据） │
//    │     命中/缺失         │  └──────────────────────────────────────┘
//    │     （写不分配）       │              │
//    └──────────────────────┘              ▼
//                          ┌──────────────────────────────────────┐
//                          │ S_RD_REFILL (3'd3)                   │
//                          │   逐字填充行内剩余 7 个字 (word1~7)  │
//                          │   refill_cnt 从 1 递增到 7           │
//                          │   每个周期 mem_addr 递增 4 字节       │
//                          │   refill_cnt == 7 且 mem_ack →       │
//                          │     写入 tag / 设置 valid            │
//                          │     mem_re 清零，返回 S_IDLE         │
//                          │   ★ refill_cnt==7 时 cpu_ready=1     │
//                          │     （最后一个字到达时通知 CPU）      │
//                          └──────────────────────────────────────┘
//
//  【时序分析 Timing Analysis】
//    关键路径 (critical path)：
//      1. 读命中路径（最短）：
//         cpu_addr → index/word 解码 → data SRAM 读取 → cpu_rdata (组合逻辑)
//         延迟 ≈ SRAM 读延迟 + 比较器延迟 + MUX 延迟
//         ★ 这是频率限制路径，在大容量 cache 中需考虑 SRAM 的读时序
//
//      2. 写命中路径：
//         cpu_addr → hit 判断 → byte-enable 写 data SRAM（时序逻辑）
//         同时启动 mem_we → 总线
//         无反馈环，时序友好
//
//      3. 读缺失路径（最长）：
//         第 0 字：(S_IDLE → S_RD_MISS) mem_re 拉高 → mem_ack 等待
//         第 1~7 字：每个周期 mem_addr 递增 + mem_ack 握手
//         总延迟 = 1 (发出请求) + N·T_mem (N 次内存访问)
//         ★ 取决于内存延迟 T_mem
//
//    cpu_ready 信号时序：
//      — S_IDLE 时为组合逻辑 1（无寄存器延迟）
//      — 其他状态在 mem_ack 有效的同一周期拉高
//      — 这是典型的 valid/ready 握手机制：CPU 在 cpu_ready=1 的时钟沿采样数据
//
//  【已知问题与局限 Known Issues & Limitations】
//    1. 阻塞式设计：读缺失期间（可能 9+ 周期）CPU 完全停顿
//       — 改进方向：非阻塞 cache（支持 hit-under-miss）
//
//    2. 无写缓冲（Write Buffer）：每次写操作都要等待 mem_ack
//       — 即使写命中（cache 已更新），CPU 仍需等待总线事务完成
//       — 改进方向：添加写缓冲 / write-posting buffer
//
//    3. 直接映射冲突缺失：同一 index、不同 tag 的地址互相驱逐
//       — 典型场景：数组 A 和 B 映射到同一 cache 行，交替访问导致 100% 缺失
//       — 改进方向：组相联（2-way / 4-way set-associative）
//
//    4. dirty 数组声明但未使用：写通策略不需要 dirty 跟踪
//       — 当前仅在 reset 时清零，之后永不被置位
//       — 保留此数组是为将来可能的写回（write-back）升级做准备
//
//    5. 无缓存一致性（No Cache Coherence）
//       — 单核 SoC 无需一致性协议
//       — 若未来扩展多核，需添加 snoop 或目录协议
//
//    6. cpu_ready 在 S_IDLE 且无请求时仍然为 1
//       — 在标准握手机制中这是可接受的：CPU 只在有请求时检查 ready
//       — 若与严格 valid/ready 握手对接，需额外逻辑
//
//    7. 无内存错误处理
//       — 不检查 mem_ack 对应的响应状态
//       — 若 AXI 从设备返回错误，cache 仍会写入错误数据并标记 valid
//
//    8. 字节使能仅用于写命中路径
//       — S_WRITE 状态传递完整的 wstrb 到内存，但 cache 更新仅在命中时执行
//       — 写缺失时 cache 不更新（写不分配），仅做 write-through 到内存
//
//    ★ 强制缺失驱逐问题：
//      读缺失时，旧行数据不经写回直接被覆盖（因为写通保证内存已最新）
//      这也正是写通策略允许的设计简化
//
//  【SoC 集成 Integration】
//    CPU 侧接口：
//      — 连接到 rv32im_core 的 LSU（Load/Store Unit）
//      — cpu_addr/cpu_wdata/cpu_wstrb 来自 MEM 阶段
//      — cpu_rdata/cpu_ready 反馈到 WB 阶段和 stall 逻辑
//      — 与 Load_unit.v / Store_unit.v 的注释中引用的 Dcache 接口对应
//
//    内存侧接口：
//      — 简单的 req/ack 握手机制（非 AXI）
//      — 在 SoC 顶层通过 bus_arbiter / axi4_interconnect 转换为 AXI4 总线
//      — 与 icache 共享同一总线仲裁器（D$ 和 I$ 双主设备）
//
//    当前集成状态：
//      — 模块已编译（Makefile 第 48 行），可供实例化
//      — TOP_levelSOc.v 当前使用 "cache bypass" 模式（直连 system_bus）
//      — 切换为 cache 模式时，将 dcache 插入 core.LSU ↔ system_bus 之间
//      — 顶层注释中注明 "I-Cache + D-Cache integrated in core" 为计划升级项
//
//  【与 icache 的对比】
//    相同点：
//      — 相同的容量和地址映射（64 行 × 8 字）
//      — 相同的直接映射结构
//      — 相同的 mem_req/mem_ack 总线接口
//    不同点：
//      — dcache 支持写操作（cpu_we + cpu_wstrb + mem_we + mem_wstrb）
//      — icache 只读，无写通逻辑
//      — icache 有单一 cpu_req 信号；dcache 有独立的 cpu_re / cpu_we
//      — dcache 有 S_WRITE 状态处理写通；icache 无此状态
//      — icache 无 wstrb 信号（指令读取始终 32 位对齐）
// ============================================================
`include "Def.v"

module dcache #(
    // ── 参数定义（默认值来自 Def.v 宏） ──
    parameter NUM_LINES  = `DCACHE_NUM_LINES,      // Cache 行数 = 64
    parameter LINE_WORDS = 8,                       // 每行字数 = 8（32 字节/行）
    parameter WORD_BITS  = `DCACHE_WORD_BITS,       // 字偏移位宽 = 3（2^3 = 8 字）
    parameter TAG_BITS   = `DCACHE_TAG_BITS,        // Tag 位宽 = 21
    parameter INDEX_BITS = `DCACHE_INDEX_BITS       // 索引位宽 = 6（2^6 = 64 行）
) (
    // ── 时钟与复位 ──
    input  wire        clk,                         // 系统时钟（posedge）
    input  wire        reset,                       // 同步复位（posedge 有效）

    // ── CPU 侧接口（与 LSU / Store_unit / Load_unit 对接） ──
    input  wire [31:0] cpu_addr,                    // CPU 请求地址（字节寻址）
    input  wire [31:0] cpu_wdata,                   // CPU 写数据（store 指令）
    input  wire [3:0]  cpu_wstrb,                   // 字节写使能（每 bit 对应 1 字节）
    input  wire        cpu_re,                      // CPU 读请求（load 指令，高有效）
    input  wire        cpu_we,                      // CPU 写请求（store 指令，高有效）
    output wire [31:0] cpu_rdata,                   // 返回给 CPU 的读数据
    output wire        cpu_ready,                   // 握手信号：1 = 当前操作完成

    // ── 内存侧接口（连接到 bus_arbiter → axi4_interconnect） ──
    output reg  [31:0] mem_addr,                    // 内存地址（行对齐或单字地址）
    output reg  [31:0] mem_wdata,                   // 写往内存的数据
    output reg  [3:0]  mem_wstrb,                   // 内存写字节使能
    output reg         mem_re,                      // 内存读请求（高有效）
    output reg         mem_we,                      // 内存写请求（高有效）
    input  wire [31:0] mem_rdata,                   // 从内存返回的读数据
    input  wire        mem_ack                      // 内存操作响应（1 周期脉冲）
);

    // ═══════════════════════════════════════════════════════════
    //  Cache 存储阵列（SRAM 风格：综合为 Block RAM 或寄存器）
    // ═══════════════════════════════════════════════════════════
    reg [TAG_BITS-1:0]         tags    [0:NUM_LINES-1];          // Tag 存储器
    reg                        valid   [0:NUM_LINES-1];          // 有效位 1'b1=行有效
    reg                        dirty   [0:NUM_LINES-1];          // 脏位（当前未使用，为写回升级预留）
    reg [31:0]                 data    [0:NUM_LINES-1][0:LINE_WORDS-1]; // 数据存储器（2D 数组：行 × 字）

    // ═══════════════════════════════════════════════════════════
    //  地址解码 —— 从 32 位字节地址中提取 TAG / INDEX / WORD
    //
    //  地址布局: |<── TAG (21) ──>|<─ INDEX (6) ─>|<─ WORD (3) ─>|<─ BYTE (2) ─>|
    //            |31             11|10             5|4            2|1            0|
    //
    //  index:  使用了 part-select 语法 [BASE -: WIDTH]，从位 BASE 向下取 WIDTH 位
    //  word:   使用了简化的范围选择 [MSB:LSB]（2-bit 字节偏移以下被截断）
    //  tag_in: 高位地址的 TAG 部分，用于与 tags[index] 比较
    //  hit:    当前访问命中 = 该行有效 且 tag 匹配
    // ═══════════════════════════════════════════════════════════
    wire [INDEX_BITS-1:0]  index   = cpu_addr[INDEX_BITS+WORD_BITS+2 -: INDEX_BITS];
    wire [WORD_BITS-1:0]   word    = cpu_addr[WORD_BITS+2-1:2];  // 字节偏移 [1:0] 在 word 选择时被丢弃
    wire [TAG_BITS-1:0]    tag_in  = cpu_addr[31:31-TAG_BITS+1]; // 等价于 cpu_addr[31:11]
    wire hit = valid[index] && (tags[index] == tag_in);          // 命中条件：有效 + tag 匹配

    // ═══════════════════════════════════════════════════════════
    //  有限状态机（FSM）状态定义
    //  ═══════════════════════════════════════════════════════════
    localparam S_IDLE      = 3'd0,   // 空闲：等待 CPU 请求，读命中直接返回
               S_WRITE     = 3'd1,   // 写通：向内存发出写请求，等待 mem_ack
               S_RD_MISS   = 3'd2,   // 读缺失第 0 字：等待缺失行的第一个字到达
               S_RD_REFILL = 3'd3;   // 读缺失填充：逐字读取并填充行内剩余 7 个字

    // ═══════════════════════════════════════════════════════════
    //  状态寄存器与辅助寄存器
    // ═══════════════════════════════════════════════════════════
    reg [2:0]  state;                       // 当前 FSM 状态
    reg [2:0]  refill_cnt;                  // 填充计数器：0→字0已到, 1→字1, ..., 7→字7(行尾)
    reg [31:0] refill_base;                 // 行对齐基地址（保留用于计算后续字地址）
    reg [INDEX_BITS-1:0] miss_index;        // 缺失行的 index（冻结以防 cpu_addr 变化）
    reg [TAG_BITS-1:0]   miss_tag;          // ★ 关键：缺失开始时刻捕获的 tag
                                             //   在填充期间 cpu_addr 可能变化（流水线冲刷等），
                                             //   必须在进入 S_RD_MISS 时立即采样 tag_in
    reg [WORD_BITS-1:0]  miss_word;         // ★ CWF fix: 缺失开始时刻捕获的 word 偏移
    reg [31:0] saved_wdata;                 // 保存写数据（S_WRITE 期间保持稳定）
    reg [3:0]  saved_wstrb;                 // 保存写字节使能（S_WRITE 期间保持稳定）
    reg [31:0] saved_addr;                  // 保存写地址（S_WRITE 期间保持稳定）

    // ── 循环变量（综合时展开为 generate） ──
    integer i;

    // ═══════════════════════════════════════════════════════════
    //  初始化：上电后所有 valid 位清零
    //  ★ 注意：tags/dirty/data 在复位时通过 always 块统一清零
    // ═══════════════════════════════════════════════════════════
    initial begin
        for (i = 0; i < NUM_LINES; i = i + 1) valid[i] = 1'b0;
    end

    // ═══════════════════════════════════════════════════════════
    //  组合逻辑输出
    // ═══════════════════════════════════════════════════════════

    // ── CPU 读数据 ──
    //   命中时：直接从 data 阵列读取（组合路径，零等待）
    //   读缺失首字时（S_RD_MISS 且 mem_ack 有效）：旁路 mem_rdata 到 CPU
    //     ★ 这是"关键字优先"（critical word first）策略：
    //       CPU 请求的字总是 refill 的第一个字，无需等待整行填充
    assign cpu_rdata = (state == S_RD_MISS && mem_ack) ? mem_rdata   // 旁路路径
                                                       : data[index][word]; // 直接读取

    // ── CPU 就绪信号 ──
    //   以下情况 cpu_ready = 1（CPU 可以提交下一个请求）：
    //     1. S_IDLE：随时就绪（读命中已在组合路径返回数据）
    //     2. S_WRITE + mem_ack：写通事务完成
    //     3. S_RD_MISS + mem_ack：第 0 字到达，CPU 请求的字已获取
    //     4. S_RD_REFILL + refill_cnt==7 + mem_ack：最后一个字到达
    //   ★ 在 S_IDLE 且无请求时 cpu_ready 仍为 1——可接受，CPU 只在有请求时检查
    assign cpu_ready = (state == S_IDLE) ||
                       (state == S_WRITE && mem_ack) ||
                       (state == S_RD_MISS && mem_ack) ||
                       (state == S_RD_REFILL && refill_cnt == 3'd7 && mem_ack);

    // ═══════════════════════════════════════════════════════════
    //  时序逻辑 —— 状态机转移与控制信号生成
    //  所有寄存器在 clk 上升沿更新，reset 上升沿复位
    // ═══════════════════════════════════════════════════════════
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            // ── 复位：清空所有状态和控制信号 ──
            state      <= S_IDLE;
            refill_cnt <= 3'd0;
            mem_re     <= 1'b0;
            mem_we     <= 1'b0;
            mem_addr   <= 32'h0;
            mem_wdata  <= 32'h0;
            mem_wstrb  <= 4'h0;
            miss_index <= 0;
            for (i = 0; i < NUM_LINES; i = i + 1) begin
                valid[i] <= 1'b0;              // 所有行失效
                dirty[i] <= 1'b0;              // 清除脏位（当前未使用）
            end
        end
        else begin
            // ── 主状态机 ──
            case (state)

                // ═══════════════════════════════════════════════
                //  S_IDLE — 空闲状态
                //
                //  功能：
                //    · 读命中：组合路径直接返回数据，状态保持不变
                //    · 写请求：进入 S_WRITE（写通）
                //    · 读缺失：进入 S_RD_MISS（行填充）
                //
                //  优先级：写 > 读（if (cpu_we) ... else if (cpu_re && !hit)）
                //   读命中（cpu_re && hit）不触发状态转移
                // ═══════════════════════════════════════════════
                S_IDLE: begin
                    // 默认：总线请求保持低（前一周期可能残留）
                    mem_re  <= 1'b0;
                    mem_we  <= 1'b0;

                    if (cpu_we) begin
                        // ── 写请求处理 ──
                        // 命中时：按字节使能更新 cache（部分字写入）
                        //   例：sb 指令 → wstrb = 4'b0001，仅更新字节 0
                        //        sw 指令 → wstrb = 4'b1111，更新全部 4 字节
                        if (hit) begin
                            if (cpu_wstrb[0]) data[index][word][7:0]   <= cpu_wdata[7:0];
                            if (cpu_wstrb[1]) data[index][word][15:8]  <= cpu_wdata[15:8];
                            if (cpu_wstrb[2]) data[index][word][23:16] <= cpu_wdata[23:16];
                            if (cpu_wstrb[3]) data[index][word][31:24] <= cpu_wdata[31:24];
                        end
                        // 无论命中与否，都要写通到内存（写不分配）
                        state       <= S_WRITE;
                        mem_addr    <= cpu_addr;                     // 使用原始地址（非行对齐）
                        mem_wdata   <= cpu_wdata;
                        mem_wstrb   <= cpu_wstrb;
                        mem_we      <= 1'b1;
                        // 保存现场：S_WRITE 期间 CPU 可能改变输入，需要冻结信号
                        saved_addr  <= cpu_addr;
                        saved_wdata <= cpu_wdata;
                        saved_wstrb <= cpu_wstrb;
                    end
                    else if (cpu_re && !hit) begin
                        // ── 读缺失处理 ──
                        // 立即采样 tag 和 word（★ 关键：此后 cpu_addr 可能变化）
                        // ★ CWF fix: 从 CPU 请求的字开始读取，非 word 0
                        state       <= S_RD_MISS;
                        refill_cnt  <= 3'd0;                        // 计数器归零
                        miss_index  <= index;                       // 冻结 index
                        miss_tag    <= tag_in;                      // ★ 冻结 tag
                        miss_word   <= word;                        // ★ 冻结请求的 word 偏移
                        // 行对齐地址：清除低 (WORD_BITS+2) 位
                        refill_base <= {cpu_addr[31:WORD_BITS+2], {(WORD_BITS+2){1'b0}}};
                        // ★ CWF fix: 起始地址为 CPU 请求的字地址，非行基地址
                        mem_addr    <= {cpu_addr[31:2], 2'b00};
                        mem_re      <= 1'b1;
                    end
                    // else: 读命中 → 什么都不做，cpu_rdata 组合路径提供数据
                end

                // ═══════════════════════════════════════════════
                //  S_WRITE — 写通状态
                //
                //  功能：等待内存写事务完成（mem_ack = 1）
                //  不区分命中/缺失：写通策略保证内存始终被更新
                //
                //  转移条件：
                //    mem_ack = 1 → 返回 S_IDLE
                //
                //  ★ 此状态不读也不写 cache 数据阵列
                //    （cache 更新已在 S_IDLE→S_WRITE 转移时完成）
                // ═══════════════════════════════════════════════
                S_WRITE: begin
                    if (mem_ack) begin
                        state  <= S_IDLE;
                        mem_we <= 1'b0;                             // 撤销写请求
                    end
                end

                // ═══════════════════════════════════════════════
                //  S_RD_MISS — 读缺失首字到达状态
                //
                //  功能：等待内存返回 burst 的第 0 个字
                //  到达时的操作：
                //    1. 将 mem_rdata 写入 data[miss_index][0]
                //    2. 同时 cpu_rdata 通过旁路将该字转发给 CPU
                //    3. 跳转到 S_RD_REFILL，继续填充剩余 7 个字
                //
                //  转移条件：
                //    mem_ack = 1 → 进入 S_RD_REFILL
                //
                //  ★ "关键字优先"（Critical Word First）：
                //    CPU 需要的字总是最先返回（word 0 = 行对齐地址的字）
                //    实际上如果 CPU 请求的不是 word 0，这里会有问题——
                //    但由于地址是行对齐的，对于 32 字节对齐的行而言，
                //    第 0 字就是行中 CPU 请求的那个字在行内的偏移…
                //    【注意】当前实现假设 refill 地址 = 行基地址 + 0，
                //        如果 CPU 请求的是 word 3，那么 word 0/1/2 也会
                //        被顺带填充，但 CPU 在 S_RD_MISS 周期得到的是 mem_rdata
                //        （即 word 0），而非请求的 word 3 —
                //        这是本设计的一个限制：
                //        refill 总是从行基地址开始，非从 CPU 请求的字开始
                // ═══════════════════════════════════════════════
                S_RD_MISS: begin
                    if (mem_ack) begin
                        // ★ CWF fix: 首字写入 miss_word 位置（而非硬编码 word 0）
                        //   此时 mem_rdata = CPU 请求的字，旁路给 cpu_rdata 直接使用
                        data[miss_index][miss_word] <= mem_rdata;
                        state      <= S_RD_REFILL;
                        refill_cnt <= 3'd1;                         // 已填充 1 字，下一字 cnt=1
                        // 下一字地址：base + ((miss_word + 1) & 7) * 4
                        mem_addr   <= refill_base + {29'd0, (miss_word + 3'd1) & 3'd7, 2'b00};
                        mem_re     <= 1'b1;                         // 保持读请求有效
                    end
                end

                // ═══════════════════════════════════════════════
                //  S_RD_REFILL — 读缺失行填充状态
                //
                //  功能：逐字接收 burst 剩余 7 个字（word 1 ~ word 7）
                //  每个周期（mem_ack 有效时）：
                //    1. 将 mem_rdata 写入 data[miss_index][refill_cnt]
                //    2. refill_cnt 递增（1→2→...→7）
                //    3. mem_addr 递增 4 字节
                //
                //  终止条件：
                //    refill_cnt == 7 且 mem_ack 有效 →
                //      写入 tag[miss_index] = miss_tag（之前冻结的 tag）
                //      设置 valid[miss_index] = 1
                //      返回 S_IDLE
                //
                //  ★ 地址计算：
                //    mem_addr = refill_base + (refill_cnt + 1) * 4
                //    等价于：refill_base + {29'd0, (refill_cnt + 1), 2'b00}
                //    例：refill_cnt=1 → addr = base + 8（word 2 的地址）
                //        …（等待一个周期后 refill_cnt 才更新，此处读的是下一个字的地址）
                //    实际上：进入此状态时 refill_cnt=1, mem_addr=base+4 (已在S_RD_MISS设置)
                //        当 mem_ack 返回的是 base+4 对应的 word 1
                //        然后 mem_addr 更新为 base + (1+1)*4 = base+8 → word 2
                // ═══════════════════════════════════════════════
                S_RD_REFILL: begin
                    if (mem_ack) begin
                        // ★ CWF fix: 使用 wrap 计算写入位置
                        //   write_pos = (miss_word + refill_cnt) & 3'd7
                        //   例如 miss_word=3: cnt 1→pos4, cnt 2→pos5, ..., cnt 4→pos7,
                        //   cnt 5→pos0 (wrap), cnt 6→pos1, cnt 7→pos2
                        data[miss_index][(miss_word + refill_cnt) & 3'd7] <= mem_rdata;
                        if (refill_cnt == 3'd7) begin
                            // 最后一个字到达：行填充完成（8 字全部写入）
                            state              <= S_IDLE;
                            mem_re             <= 1'b0;             // 撤销读请求
                            tags[miss_index]   <= miss_tag;         // 写入冻结的 tag
                            valid[miss_index]  <= 1'b1;             // 标记该行有效
                        end
                        else begin
                            // 继续填充下一个字（带 wrapping）
                            refill_cnt <= refill_cnt + 3'd1;
                            // 下一字地址：base + ((miss_word + cnt + 1) & 7) * 4
                            mem_addr   <= refill_base +
                                          {29'd0, (miss_word + refill_cnt + 3'd1) & 3'd7, 2'b00};
                            mem_re     <= 1'b1;                     // 保持读请求有效
                        end
                    end
                end

            endcase
        end
    end

endmodule
