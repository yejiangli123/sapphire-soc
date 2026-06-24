// ╔══════════════════════════════════════════════════════════════════════════╗
// ║                    Control_unit.v — 控制信号生成单元                        ║
// ╚══════════════════════════════════════════════════════════════════════════╝
//
// ┌──────────────────────────────────────────────────────────────────────────┐
// │ 一、模块在 CPU 流水线中的角色与定位                                        │
// └──────────────────────────────────────────────────────────────────────────┘
//
//   Control_unit 是整个 RISC-V 五级流水线（IF → ID → EX → MEM → WB）中
//   ID 阶段（指令译码阶段）的核心部件。它位于指令寄存器（ir_reg）和寄存器堆
//   （reg_file）之间，负责将 32 位指令中提取出的 opcode、funct3、funct7
//   转换为各组控制信号，驱动下游的 ALU、立即数生成器（imm_gen）、写回多路
//   选择器（wb_mux）等硬件单元。
//
//   流水线位置示意：
//
//     PC → [IMEM/ICache] → [IF/ID reg] → [Control_unit] → [RegFile + ImmGen + ALU_ctrl]
//                                              │
//                          控制信号广播到 EX/MEM/WB 各阶段
//
//   设计原则：
//     - 纯组合逻辑（除 reg_u_type_lui 需要跨 1 拍延迟外，其余全是组合输出）
//     - 无状态机，不产生停顿 — 停顿由 stall_unit 统一管理
//     - 使用 one-hot 方式的 case(1'b1) 结构，综合工具可优化为并行译码
//     - 所有控制信号的编码均与 Def.v 中的宏定义对应，保证全系统一致性
//
// ┌──────────────────────────────────────────────────────────────────────────┐
// │ 二、与其他流水线阶段的交互关系                                            │
// └──────────────────────────────────────────────────────────────────────────┘
//
//   从 ID 阶段接收：
//     - ir_reg 输出的 opcode[6:0]、funct3[2:0]、funct7（即 funct7[5]）
//     - load_type_in（由 load_unit 反馈，用于 load 类型识别备用）
//
//   向 ID 阶段输出：
//     - ir_mux[1:0]       → Bypass 单元 / reg_file：指示需要读几个源寄存器
//     - bypass_ir1/ir2     → Bypass 单元：指示操作数 1/2 是否需要旁路
//     - sb_type            → Bypass 单元：当前是否为 Store/Branch（需要地址计算）
//     - out_u_type         → Bypass 单元 / imm_gen：当前是否为 U 型指令
//     - op_format_out[2:0] → imm_gen（立即数生成器）：选择立即数拼接格式
//
//   向 EX 阶段输出（随流水线寄存器传递）：
//     - aluop[3:0]         → ALU：选择算术/逻辑/比较/乘除操作
//     - ready_in           → 随流水线传递，JALR 标志供 EX 阶段使用
//     - reg_u_type_lui     → 延迟 1 拍，供 WB 阶段的 rd 写回判断使用（LUI 特殊处理）
//
//   向 WB 阶段输出（随流水线寄存器传递）：
//     - werf_contrl        → reg_file：寄存器写使能
//     - wbmux_contol[1:0]  → wb_mux：写回数据选择（ALU结果/MEM读数/PC+4）
//
// ┌──────────────────────────────────────────────────────────────────────────┐
// │ 三、指令类型 → 控制信号速查表                                              │
// └──────────────────────────────────────────────────────────────────────────┘
//
//   指令类型     opcode     werf  wbmux   aluop        ir_mux  op_format
//   ────────────────────────────────────────────────────────────────────────
//   R-type      0110011    1     00      funct3/7 译码  00      I_type
//   M-type      0110011    1     00      MULDIV 操作    00      I_type
//   I-type ALU  0010011    1     00      funct3/7 译码  01      I_type
//   Load        0000011    1     01      ADD (地址计算)  01      I_type_load
//   Store       0100011    0     —      ADD (地址计算)  00      S_type
//   Branch      1100011    0     —      funct3 比较     00      B_type
//   JAL         1101111    1     10      ADD (地址计算)  01      J_type
//   JALR        1100111    1     10      ADD (地址计算)  01      I_type
//   LUI         0110111    1     00      ADD（中转）     01      U_type_LUI
//   AUIPC       0010111    1     00      ADD (地址计算)  01      U_type_AUIPC
//
//   注：
//   - wbmux: 00=ALU, 01=MEM, 10=PC+4
//   - LUI 的"ALU 结果"实际上是立即数（通过 ALU 的 ADD 通道直通，见 EX 阶段实现）
//   - Branch 的 werf=0 因为分支不写回寄存器，比较结果用于跳转判断而非写回
//   - M-type 的 opcode 与 R-type 相同（OPCODE_OP=0110011），靠 funct7[5]=1 区分
//
// ┌──────────────────────────────────────────────────────────────────────────┐
// │ 四、端口逐一定义                                                          │
// └──────────────────────────────────────────────────────────────────────────┘

`include "Def.v"

// ╔══════════════════════════════════════════════════════════════════════════╗
// ║  本地宏定义                                                              ║
// ╚══════════════════════════════════════════════════════════════════════════╝

// 写回数据选择码（仅本模块使用，Def.v 中无对应宏，因此在此局部定义）
//   设计理由：
//     - WB_ALU (2'b00): 写回 ALU 运算结果。涵盖 R/I/M-type 算术逻辑、
//       LUI（立即数经 ALU 直通）、AUIPC（PC+imm 经 ALU 加法）
//     - WB_MEM (2'b01): 写回存储器读出数据。仅 Load 指令使用，数据来自
//       d_cache 或 AXI 总线读取结果
//     - WB_PC  (2'b10): 写回 PC+4。仅 JAL/JALR 使用，将返回地址存入 rd
//     - 2'b11 保留未用（可扩展为 CSR 写回等）
`define WB_ALU  2'b00
`define WB_MEM  2'b01
`define WB_PC   2'b10


module control_unit (
    // ============================================================
    //  全局信号
    // ============================================================
    input  wire           clk,              // 时钟：仅 reg_u_type_lui 使用（上升沿采样），其余为组合逻辑
    input  wire           reset,            // 同步复位：清空 reg_u_type_lui 寄存器

    // ============================================================
    //  输入：来自 IF/ID 流水线寄存器（ir_reg 输出的指令字段译码结果）
    //  ★ load_type_in 目前为预留接口，本模块未实际使用其值
    // ============================================================
    input  wire           load_type_in,     // 由 load_unit 反馈的 load 类型标志（预留，未在本模块中使用）
    input  wire [6:0]     opcode_in,        // 7 位 opcode，来自指令 [6:0]。RISC-V 的 opcode 在指令最低 7 位
    input  wire [2:0]     funct3_in,        // 3 位 funct3，来自指令 [14:12]。细分 ALU 操作 / 分支条件 / Load-Store 宽度
    input  wire           funct7_in,        // funct7[5] 位，来自指令 [30]。1=乘除扩展(M-type)，0=基本 ALU 操作；用于区分 ADD/SUB 和 SRL/SRA

    // ============================================================
    //  输出（1）寄存器堆控制
    // ============================================================
    output reg            werf_contrl,      // 寄存器写使能：1=写回 rd，0=不写回
    //  ★ 为什么 Store/Branch 输出 0：
    //      - Store: 数据写入存储器而非寄存器，rd 字段在 S-type 中被重用作 imm[4:0]
    //      - Branch: 比较结果仅用于跳转判断，不产生寄存器写回值
    //      - 其他所有计算类/加载类/跳转链接类指令均写回 rd

    output reg  [1:0]     wbmux_contol,     // 写回数据选择信号（00=ALU, 01=MEM, 10=PC+4, 11=保留）
    //  ★ 为什么 Load 选择 MEM：
    //      - Load 指令的 "ALU 输出" 是存储器地址，真正的数据来自存储器读取结果
    //      - 因此写回阶段需要 bypass ALU 结果而选择 MEM 通道
    //  ★ 为什么 JAL/JALR 选择 PC+4：
    //      - RISC-V 规范要求将 PC+4 写入 rd（返回地址），ALU 在此期间被用于地址计算
    //  ★ 为什么 LUI/AUIPC 选择 ALU：
    //      - LUI 的立即数在 EX 阶段经 ALU 通道直通（实际为 ADD 操作，加数之一为 0）
    //      - AUIPC 的 PC+imm 在 ALU 中计算

    // ============================================================
    //  输出（2）ALU 控制
    // ============================================================
    output reg  [3:0]     aluop,            // ALU 操作码（4 位），对应 Def.v 中 ALU_ADD/ALU_SUB/.../ALU_DIVU 等宏
    //  ★ 为什么 ALU 操作码为 4 位：
    //      - 基本 ALU 操作 10 种（ADD~SLTU），需 4 位编码
    //      - M 扩展（乘除）复用高 4 位空间，编码为 4'b1010~4'b1111
    //      - 总编码空间 16 个，已使用 16 个，无冗余

    // ============================================================
    //  输出（3）操作数 / 立即数控制
    // ============================================================
    output reg  [1:0]     ir_mux,           // 指令寄存器源操作数选择信号
    //  ★ 编码含义：
    //      00 — 双源操作数：需要读 rs1 和 rs2 (R/S/B-type)
    //      01 — 单源操作数：需要读 rs1，rs2 被立即数替代 (I/U/J-type)
    //      10 — 保留
    //      11 — 保留
    //  ★ 为什么需要这个信号：
    //      - Bypass/Forwarding 单元需要知道有几个源操作数在流水线中活跃
    //      - 双源时两条旁路通道都需要监听，单源时仅通道 1 活跃
    //      - 影响寄存器堆的读端口使能和转发比较逻辑

    output wire           stall,            // 停顿信号：已废弃，固定输出 0
    //  ★ 为什么固定为 0：
    //      - 停顿由 stall_unit 统一管理（集中式停顿控制器）
    //      - 过去的版本可能在此分散处理停顿，重构后统一到 stall_unit
    //      - 保留该端口仅为向后兼容，不影响功能

    output reg  [2:0]     op_format_out,    // 立即数格式选择（3 位），连接到 imm_gen 模块
    //  ★ 对应 Def.v 中的格式宏：
    //      I_type      (3'b000): 12 位符号扩展，用于 I-type ALU / JALR / R-type
    //      I_type_load (3'b001): 12 位符号扩展，用于 Load（与 I_type 相同，独立编码便于区分）
    //      S_type      (3'b010): 12 位拼接 [31:25|11:7]，用于 Store
    //      B_type      (3'b011): 13 位拼接 [31|7|30:25|11:8]，左移 1 位，用于 Branch
    //      J_type      (3'b100): 21 位拼接 [31|19:12|20|30:21]，左移 1 位，用于 JAL
    //      U_type_LUI  (3'b101): 20 位左移 12 位，用于 LUI
    //      U_type_AUIPC(3'b110): 20 位左移 12 位，用于 AUIPC（与 LUI 格式相同，独立编码便于区分）
    //  ★ 为什么 R-type 也输出 I_type：
    //      - R-type 没有立即数，但 imm_gen 仍然需要一个合法输出以避免 X 态传播
    //      - 输出 I_type 格式（12 位符号扩展）不会影响 R-type 的 ALU 运算，因为 EX 阶段
    //        mux 会选择 rs2 而非 imm

    // ============================================================
    //  输出（4）旁路 / 转发辅助信号（供 Bypass/Forwarding 单元使用）
    //  ★ 这些信号逻辑简单，放在此处避免创建独立的转发控制模块
    //  ★ 如需扩展转发逻辑，可将这些信号迁移到独立的 Forwarding Unit
    // ============================================================
    output wire           bypass_ir1,       // 操作数 1 旁路使能：1=rs1 需要旁路检测，0=不需要
    //  ★ 为什么 JAL/LUI/AUIPC 排除在外：
    //      - JAL: rd 由立即数给出（PC+4 写回），不使用 rs1 → 旁路 rs1 无意义
    //      - LUI/AUIPC: 不需要任何源寄存器，rd 从立即数/PC+imm 得到 → 旁路无意义

    output wire           bypass_ir2,       // 操作数 2 旁路使能：1=rs2 需要旁路检测，0=不需要
    //  ★ 为什么仅 R/S/B-type 有效：
    //      - 只有这三种格式同时使用 rs1 和 rs2 作为源操作数
    //      - I/U/J-type 的第二个操作数是立即数，rs2 字段被重用作立即数的一部分
    //      - 因此旁路通道 2（rs2 转发）仅在这些指令时活跃

    output wire           sb_type,          // Store/Branch 类型标志：1=S-type 或 B-type
    //  ★ 用途：
    //      - S-type 和 B-type 的共同点：都需要 ALU 计算地址，但对寄存器堆无写回
    //      - 转发单元据此判断：这两类指令的 ALU 结果不进入寄存器堆，仅用于地址/比较

    output wire           out_u_type,       // U 型指令标志：1=LUI 或 AUIPC
    //  ★ 用途：
    //      - U 型指令的特殊性：20 位立即数需要左移 12 位后放入 rd
    //      - imm_gen 需要据此选择 U 型拼接格式
    //      - Bypass 单元需要排除 U 型指令（因其无源操作数）

    output reg            reg_u_type_lui,   // LUI 指令延迟 1 拍标志（寄存器输出，非组合逻辑）
    //  ★ 为什么需要延迟 1 拍：
    //      - LUI 指令在 ID 阶段译码，但 rd 写回发生在 WB 阶段
    //      - 如果在 ID 阶段直接使用组合逻辑的 u_type_lui 驱动 WB 阶段逻辑，
    //        会导致时序路径过长（ID → WB 的组合路径）
    //      - 因此用寄存器将 LUI 标志延迟 1 拍（ID → EX），使其随流水线寄存器同步到达 WB

    // ============================================================
    //  输出（5）跳转辅助
    // ============================================================
    output wire           ready_in,         // JALR 使能标志：1=当前指令为 JALR
    //  ★ 为什么单独引出 JALR 标志：
    //      - JALR 在 EX 阶段需要特殊处理：计算目标地址(rs1+imm)，同时写回 PC+4
    //      - 该标志随流水线寄存器传递到 EX 阶段，触发 JALR 专用的 PC 更新逻辑

    // ============================================================
    //  输出（6）Load 类型
    // ============================================================
    output wire           i_type_load       // Load 指令标志：1=opcode 为 OPCODE_LOAD(0000011)
    //  ★ 用途：
    //      - 连接到 load_unit / 存储器访问阶段，用于触发 load 操作
    //      - 与 Store 区分，控制数据存储器的读/写方向
);


// ╔══════════════════════════════════════════════════════════════════════════╗
// ║  第一组：指令类型识别（组合逻辑，根据 opcode / funct7 分类）               ║
// ╚══════════════════════════════════════════════════════════════════════════╝
//
//  ☆ 设计理由：
//     RISC-V 的 7 位 opcode 直接定义在指令 [6:0]，可唯一区分 9 种指令格式。
//     此处用连续赋值 wire 提前将 "是什么指令" 计算好，后续控制信号生成只需
//     使用这些布尔变量做 case 选择，避免在多个 always 块中重复写 opcode 比较。
//
//  ☆ M-type 如何识别：
//     M 扩展指令的 opcode 与 R-type 相同 (OPCODE_OP=0110011)，区别在于
//     funct7 的 bit[5]（即指令 [30]）为 1。因此 `m_type = r_type && funct7_in`。
//     这是 RISC-V 规范的标准做法：funct7[5]=0 为基本整数运算，=1 为乘除扩展。

wire r_type       = (opcode_in == `OPCODE_OP);        // 0110011: R-type (寄存器-寄存器 ALU 运算)
wire m_type       = r_type && funct7_in;              // R-type 且 funct7[5]=1 → M 扩展（乘除指令）
wire i_type_alu   = (opcode_in == `OPCODE_OP_IMM);    // 0010011: I-type ALU（立即数 ALU 运算）
assign i_type_load = (opcode_in == `OPCODE_LOAD);     // 0000011: Load（从存储器加载）
wire s_type       = (opcode_in == `OPCODE_STORE);     // 0100011: Store（存入存储器）
wire b_type       = (opcode_in == `OPCODE_BRANCH);    // 1100011: Branch（条件分支）
wire jal_type     = (opcode_in == `OPCODE_JAL);       // 1101111: JAL（跳转并链接）
assign ready_in   = (opcode_in == `OPCODE_JALR);      // 1100111: JALR（寄存器间接跳转并链接）
wire u_type_lui   = (opcode_in == `OPCODE_LUI);       // 0110111: LUI（加载高位立即数）
wire u_type_auipc = (opcode_in == `OPCODE_AUIPC);     // 0010111: AUIPC（PC 加高位立即数）


// ╔══════════════════════════════════════════════════════════════════════════╗
// ║  第二组：辅助控制信号                                                      ║
// ╚══════════════════════════════════════════════════════════════════════════╝

// ── stall ──────────────────────────────────────────────────────────────
// 停顿信号已废弃，固定输出 0。停顿由 stall_unit 统一管理。
// 原因：分散的停顿控制会导致优先级冲突和死锁风险，集中式管理更可靠。
assign stall      = 1'b0;

// ── out_u_type ─────────────────────────────────────────────────────────
// U 型指令（LUI 或 AUIPC）汇总标志。
// 设计理由：LUI 和 AUIPC 的共同点是 20 位立即数需左移 12 位，
// 且都不需要源操作数（rd 直接从立即数 / PC+imm 得到）。
// Bypass 单元据此排除对这两类指令的源操作数转发。
assign out_u_type = u_type_lui || u_type_auipc;

// ── bypass_ir1 ─────────────────────────────────────────────────────────
// 操作数 1（rs1）旁路使能。
// 设计理由：
//   - JAL: rd 由 J-type 立即数给出，rs1 不使用 → 无需旁路 rs1
//   - LUI: 20 位立即数直接加载到 rd 高 20 位，无 rs1 → 无需旁路
//   - AUIPC: PC+imm，无 rs1 → 无需旁路
//   - 其余所有指令均使用 rs1（作为 ALU 操作数、地址基址、或 Store 数据源）
// 综合以上，"需要 rs1" 等价于 "不是 (JAL 或 LUI 或 AUIPC)"。
assign bypass_ir1 = ~(jal_type || u_type_lui || u_type_auipc);

// ── bypass_ir2 ─────────────────────────────────────────────────────────
// 操作数 2（rs2）旁路使能。
// 设计理由：
//   - R-type: 两个源操作数都是寄存器 → rs2 需要旁路
//   - S-type: rs2 是待存储数据 → 需要旁路检查
//   - B-type: rs2 参与分支比较 → 需要旁路检查
//   - I-type: 第二个操作数是立即数（rs2 字段编码立即数）→ rs2 不需要旁路
//   - U-type: 无 rs2 → 不需要旁路
//   - J-type: 无 rs2 → 不需要旁路
// 因此，"需要 rs2" 等价于 "R-type 或 S-type 或 B-type"。
assign bypass_ir2 = r_type || s_type || b_type;

// ── sb_type ────────────────────────────────────────────────────────────
// Store/Branch 汇总标志。
// 设计理由：
//   - 两种指令的共同点：都需要 ALU 计算地址（基址+偏移），但结果不写回寄存器堆
//   - S-type 的 ALU 输出是存储器写入地址
//   - B-type 的 ALU 输出是比较结果（差/大小关系），用于跳转判断
//   - 转发单元据此可识别：这两类指令的 ALU 结果不作为寄存器数据源
assign sb_type    = b_type || s_type;


// ╔══════════════════════════════════════════════════════════════════════════╗
// ║  第三组：立即数格式选择（op_format_out）                                   ║
// ╚══════════════════════════════════════════════════════════════════════════╝
//
//  立即数格式选择直接映射到 imm_gen 的拼接逻辑：
//
//    I_type      (000): inst[31:20] 符号扩展 12→32 位
//    I_type_load (001): 同 I_type（独立编码，便于区分 Load/I-type ALU）
//    S_type      (010): {inst[31:25], inst[11:7]} 拼接 + 符号扩展
//    B_type      (011): {inst[31], inst[7], inst[30:25], inst[11:8]} + 末尾补 0
//    J_type      (100): {inst[31], inst[19:12], inst[20], inst[30:21]} + 末尾补 0
//    U_type_LUI  (101): inst[31:12] 左移 12 位（低 12 位补 0）
//    U_type_AUIPC(110): 同 U_type_LUI（独立编码，AUIPC 场景下需加 PC 基址）
//
//  ☆ 设计理由 —— 为什么用 case(1'b1) 而非 if-else 链：
//     综合工具对 case(1'b1) 的 one-hot 译码优化效果更好，生成的电路更小更快。
//     配合 synthesis parallel_case full_case 指示，综合工具可以安全地优化掉
//     未命中分支。多条 case_item 可以放在同一行（如 i_type_alu, r_type, ready_in）
//     表示它们共享同一个输出值，等同于多个 case 分支的 OR。
//
//  ☆ 为什么 R-type 也用 I_type 格式：
//     R-type 指令没有立即数字段。但 imm_gen 必须输出一个确定值（不能是 X 或高阻）。
//     输出 I_type 格式不会影响功能，因为 EX 阶段有一个 mux 在 ALU 第一个操作数
//     (rs1/PC) 和第二个操作数 (rs2/imm) 之间选择——R-type 时 mux 会选择 rs2
//     而非 imm，imm_gen 的输出被忽略。
//
//  ☆ 为什么 JALR 也用 I_type 格式：
//     JALR 的立即数是 12 位有符号立即数，编码格式与 I-type ALU 完全相同
//     (inst[31:20])。JALR 跳转目标为 (rs1 + sign_extend(imm[11:0])) & ~1。

always @(*) begin
    case (1'b1)  // synthesis parallel_case full_case
        u_type_auipc: op_format_out = `U_type_AUIPC;   // AUIPC: 高位立即数 + PC
        jal_type:     op_format_out = `J_type;         // JAL: 21 位跳转偏移
        u_type_lui:   op_format_out = `U_type_LUI;     // LUI: 高位立即数（低 12 位补 0）
        b_type:       op_format_out = `B_type;         // Branch: 13 位分支偏移
        s_type:       op_format_out = `S_type;         // Store: 12 位存储偏移
        i_type_load:  op_format_out = `I_type_load;    // Load: 12 位加载偏移
        // 以下三种情况（I-type ALU / R-type / JALR）共用 I_type 格式
        //   - I-type ALU: inst[31:20] 为 12 位有符号立即数
        //   - R-type: 无立即数，输出 I_type 保证 imm_gen 不产生 X 态
        //   - JALR: inst[31:20] 为 12 位有符号跳转偏移
        i_type_alu,
        r_type,
        ready_in:     op_format_out = `I_type;
        default:      op_format_out = `I_type;         // 安全回退：未知指令按 I_type 处理
    endcase
end


// ╔══════════════════════════════════════════════════════════════════════════╗
// ║  第四组：ALU 操作码生成（aluop）                                          ║
// ╚══════════════════════════════════════════════════════════════════════════╝
//
//  aluop 是发给 ALU 的 4 位操作码，决定 ALU 执行何种运算。生成分四个场景：
//
//  场景一：M-type（乘除扩展）
//    - opcode=0110011, funct7[5]=1
//    - 根据 funct3 选择乘法/除法/取余操作
//    - RV32M 定义了 8 种操作：MUL/MULH/MULHSU/MULHU/DIV/DIVU/REM/REMU
//    - 编码映射：
//        funct3=000 → MUL,      funct3=001 → MULH
//        funct3=010 → MULHSU,   funct3=011 → MULHU
//        funct3=100 → DIV,      funct3=101 → DIVU
//        funct3=110 → REM,      funct3=111 → REMU
//    ★ 注意：REM 和 DIV 共用 aluop 编码（4'b1110），REMU 和 DIVU 共用（4'b1111）。
//      实际区分由 mul_div_unit 内部的 funct3 判断完成。
//
//  场景二：R-type / I-type ALU（基本整数运算）
//    - R-type:  opcode=0110011, funct7[5]=0
//    - I-type:  opcode=0010011
//    - 根据 funct3（+ funct7 的 bit[5]）查表：
//        funct3=000: funct7[5]=1 → SUB, funct7[5]=0 → ADD
//        funct3=001: SLL（逻辑左移）
//        funct3=010: SLT（有符号小于置位）
//        funct3=011: SLTU（无符号小于置位）
//        funct3=100: XOR（异或）
//        funct3=101: funct7[5]=1 → SRA（算术右移）, funct7[5]=0 → SRL（逻辑右移）
//        funct3=110: OR（或）
//        funct3=111: AND（与）
//    ★ 为什么 ADD/SUB 和 SRL/SRA 需要 funct7 区分：
//      RISC-V 规范中，ADD 和 SUB 的 funct3 相同(000)，SRL 和 SRA 的 funct3 相同(101)。
//      通过 funct7 的最高位（指令 [30]）区分。这减少了指令编码空间的浪费，
//      但需要在译码阶段增加一级判断。
//
//  场景三：Branch（分支比较）
//    - 根据 funct3 选择 ALU 比较操作
//    - BEQ/BNE: 用 SUB 计算差值，差=0 → BEQ 成立，差≠0 → BNE 成立
//    - BLT/BGE: 用 SLT（有符号小于），BGE 的条件取反（rs1≥rs2 ≡ not rs1<rs2）
//    - BLTU/BGEU: 用 SLTU（无符号小于），BGEU 同理取反
//    ★ 为什么 BEQ/BNE 都用 SUB：
//      两个数相等当且仅当 (rs1 - rs2) == 0。ALU 输出差值，
//      分支判断逻辑根据差值和 funct3 决定是否跳转。
//    ★ 为什么 BLT/BGE 共用 SLT：
//      BLT 跳转条件为 rs1<rs2（SLT 的语义），成立则跳转。
//      BGE 跳转条件为 rs1≥rs2，等价于 NOT(rs1<rs2)，对 SLT 结果取反即可。
//      BLTU/BGEU 同理使用 SLTU。
//
//  场景四：default（Load/Store/JAL/JALR/LUI/AUIPC）
//    - 这些指令的 ALU 用途是地址计算或数据直通，统一使用 ADD
//    - Load/Store: 基址 + 偏移 = 存储器地址
//    - JAL/JALR:   PC + 偏移 = 跳转目标（同时计算 PC+4 写回）
//    - LUI:        立即数通过 ALU 的 ADD 通道直通（实际上加数为 0）
//    - AUIPC:      PC + 高位立即数
//    ★ 为什么 LUI 用 ADD 而非特殊操作码：
//      LUI 需将 20 位立即数存入 rd 高 20 位，低 12 位为 0。在 EX 阶段可通过
//      ALU 的 ADD 通道实现：一个操作数为 {imm, 12'b0}，另一个为 0，结果直通。
//      无需为 LUI 单独设计 ALU 操作码，减少 ALU 复杂度。

always @(*) begin
    case (1'b1)  // synthesis parallel_case full_case

        // ─── M-type：乘除扩展 ───────────────────────────────────────────
        // 触发条件：m_type = (opcode==OPCODE_OP) && funct7[5]==1
        // 根据 funct3 选择具体的乘除操作
        m_type: begin
            case (funct3_in)
                `MULDIV_MUL:    aluop = `ALU_MUL;        // 有符号乘法（低 32 位）
                `MULDIV_MULH:   aluop = `ALU_MULH;       // 有符号乘法（高 32 位）
                `MULDIV_MULHSU: aluop = `ALU_MULHSU;     // 有符号×无符号（高 32 位）
                `MULDIV_MULHU:  aluop = `ALU_MULHU;      // 无符号乘法（高 32 位）
                `MULDIV_DIV:    aluop = `ALU_DIV;        // 有符号除法
                `MULDIV_DIVU:   aluop = `ALU_DIVU;       // 无符号除法
                `MULDIV_REM:    aluop = `ALU_REM;        // 有符号取余（与 DIV 同编码，由 funct3 区分）
                `MULDIV_REMU:   aluop = `ALU_REMU;       // 无符号取余（与 DIVU 同编码，由 funct3 区分）
                default:        aluop = `ALU_ADD;        // 安全回退
            endcase
        end

        // ─── R-type / I-type ALU：基本整数运算 ─────────────────────────
        // 触发条件：r_type = (opcode==OPCODE_OP && funct7[5]==0)
        //           i_type_alu = (opcode==OPCODE_OP_IMM)
        // ★ I-type ALU 没有 SUB/SRA（因为 12 位立即数无需区分加减/左右移方向），
        //   但此处统一用 funct7_in 判断也无妨——I-type ALU 指令的 inst[30] 位
        //   恰好是立即数的最高位 imm[11]，用作区分 ADD/SUB 和 SRL/SRA。
        //   实际上 RV32I 规定 I-type ALU 的 funct7[5] 被忽略，所有移位操作
        //   使用低 5 位 imm 作为移位量，本设计中 funct7_in 的取值由指令 [30] 决定，
        //   对 I-type 而言这是 imm[11]，不会错误触发 SUB 或 SRA。
        r_type, i_type_alu: begin
            case (funct3_in)
                3'b000: aluop = funct7_in ? `ALU_SUB  : `ALU_ADD;   // ADD/SUB
                3'b001: aluop = `ALU_SLL;                           // 逻辑左移
                3'b010: aluop = `ALU_SLT;                           // 有符号小于
                3'b011: aluop = `ALU_SLTU;                          // 无符号小于
                3'b100: aluop = `ALU_XOR;                           // 按位异或
                3'b101: aluop = funct7_in ? `ALU_SRA  : `ALU_SRL;   // 算术右移 / 逻辑右移
                3'b110: aluop = `ALU_OR;                            // 按位或
                3'b111: aluop = `ALU_AND;                           // 按位与
                default: aluop = `ALU_ADD;                          // 安全回退
            endcase
        end

        // ─── Branch：分支比较 ─────────────────────────────────────────
        // 触发条件：b_type = (opcode==OPCODE_BRANCH)
        // 根据 funct3 选择 ALU 比较操作，比较结果供 Branch Unit 决策是否跳转
        b_type: begin
            case (funct3_in)
                `BRANCH_EQ:  aluop = `ALU_SUB;     // BEQ:  相等 ↔ 差 = 0
                `BRANCH_NE:  aluop = `ALU_SUB;     // BNE:  不等 ↔ 差 ≠ 0
                `BRANCH_LT:  aluop = `ALU_SLT;     // BLT:  有符号小于（成立时 SLT 输出 1）
                `BRANCH_GE:  aluop = `ALU_SLT;     // BGE:  有符号≥（SLT 输出 0 时成立，取反条件）
                `BRANCH_LTU: aluop = `ALU_SLTU;    // BLTU: 无符号小于
                `BRANCH_GEU: aluop = `ALU_SLTU;    // BGEU: 无符号≥（同上取反）
                default:     aluop = `ALU_SUB;      // 安全回退
            endcase
        end

        // ─── default：Load / Store / JAL / JALR / LUI / AUIPC ─────────
        // 这些指令的 ALU 用途统一为地址计算或数据直通，使用 ADD 即可。
        // ★ Load/Store:  基址寄存器 + 符号扩展立即数 → 存储器访问地址
        // ★ JAL/JALR:    跳转目标计算（PC+imm 或 rs1+imm）
        // ★ LUI:         立即数经 ALU ADD 直通（加数为 0）
        // ★ AUIPC:       PC + 20 位立即数左移 12 位
        default: aluop = `ALU_ADD;
    endcase
end


// ╔══════════════════════════════════════════════════════════════════════════╗
// ║  第五组：寄存器写使能（werf_contrl）                                       ║
// ╚══════════════════════════════════════════════════════════════════════════╝
//
//  决定当前指令是否需要写回 rd 寄存器。
//
//  ★ 写回 1 的指令类型（会产生寄存器结果）：
//      - R-type:   ALU 运算结果（加减移位逻辑等）
//      - M-type:   乘除结果
//      - I-type ALU: 立即数 ALU 运算结果
//      - Load:     存储器读出数据（在 WB 阶段写回）
//      - JAL:      PC+4 返回地址
//      - JALR:     PC+4 返回地址
//      - LUI:      高位立即数（低 12 位为 0）
//      - AUIPC:    PC + 高位立即数
//
//  ★ 写回 0 的指令类型（不产生寄存器结果）：
//      - Store:   数据写入存储器，无寄存器写回
//      - Branch:  仅比较跳转，无寄存器写回
//    （其他未列出的 opcode：如 FENCE、ECALL、CSR 指令等，默认不写回）
//
//  ☆ 设计考量：
//      此信号随流水线寄存器逐级传递（ID→EX→MEM→WB），在 WB 阶段作为 reg_file
//      的写使能。即使 ID 阶段就生成了正确的值，也必须等待 WB 阶段才使用，
//      因为在此之前数据尚未就绪（Load 需等 MEM 完成，ALU 需等 EX 完成）。

always @(*) begin
    case (1'b1)  // synthesis parallel_case full_case
        // 所有需要写回寄存器的指令类型
        r_type, i_type_alu, i_type_load, jal_type, ready_in,
        u_type_lui, u_type_auipc: werf_contrl = 1'b1;
        // Store、Branch 及其他未识别指令均不写回
        default:                   werf_contrl = 1'b0;
    endcase
end


// ╔══════════════════════════════════════════════════════════════════════════╗
// ║  第六组：写回数据选择（wbmux_contol）                                      ║
// ╚══════════════════════════════════════════════════════════════════════════╝
//
//  决定 WB 阶段写入 rd 的数据来源：
//
//    WB_ALU (2'b00): ALU 运算结果
//      - 涵盖 R/I/M-type 算术逻辑运算结果
//      - LUI 的 20 位立即数（经 ALU ADD 直通）
//      - AUIPC 的 PC+imm
//
//    WB_MEM (2'b01): 存储器读出数据
//      - 仅 Load 指令使用
//      - 数据来自 d_cache / AXI 读数据通道
//
//    WB_PC  (2'b10): PC+4（返回地址）
//      - JAL/JALR 将下一条指令的地址（PC+4）写入 rd
//      - 符合 RISC-V 规范：rd = PC + 4（链接地址）
//
//  ☆ 优先级说明：
//      Load 和 JAL/JALR 都在需要写回的指令之列（werf=1），但写回数据的
//      来源不同。Load 优先于 JAL/JALR 的判断：因为 Load 的 opcode 和
//      JAL/JALR 互斥（不同 opcode），两者不可能同时为真。
//      此处 case 的分支顺序决定了：如果是 Load 则选 MEM，
//      如果是 JAL/JALR 则选 PC+4，其余默认选 ALU。

always @(*) begin
    case (1'b1)  // synthesis parallel_case full_case
        i_type_load:              wbmux_contol = `WB_MEM;   // Load: 写回存储器数据
        jal_type, ready_in:       wbmux_contol = `WB_PC;    // JAL/JALR: 写回 PC+4
        default:                  wbmux_contol = `WB_ALU;   // 其他: 写回 ALU 结果
    endcase
end


// ╔══════════════════════════════════════════════════════════════════════════╗
// ║  第七组：寄存器源操作数选择（ir_mux）                                      ║
// ╚══════════════════════════════════════════════════════════════════════════╝
//
//  ir_mux[1:0] 指示当前指令需要从寄存器堆读取几个源操作数：
//
//    2'b00 — 双源操作数：R-type (rs1+rs2), S-type (rs1 基址+rs2 数据), B-type (rs1+rs2)
//    2'b01 — 单源操作数：I-type (rs1+imm), U-type (无 rs/仅 imm), J-type (无 rs/仅 imm)
//
//  ☆ 为什么 S-type 是双源而 I-type 是单源：
//      - S-type 格式中，rs1 提供存储器基址，rs2 提供待存储数据
//      - I-type 格式中，rs1 提供 ALU 操作数或基址，rs2 字段被重用作立即数编码
//      - 因此 S-type 需要读两个寄存器，I-type 只需读一个
//
//  ☆ 为什么这个信号重要：
//      - 转发（Forwarding/Bypass）单元需要知道流水线中有几个活跃的源寄存器
//      - 双源时：需要同时比较 EX 和 MEM 阶段的目标寄存器与 rs1/rs2
//      - 单源时：仅需比较 rs1，rs2 通道空闲（降低功耗，减少误转发）
//      - 寄存器堆的读端口使能也可据此优化

always @(*) begin
    if (r_type || s_type || b_type)
        ir_mux = 2'b00;    // 双源：R/S/B-type（两者都读寄存器）
    else
        ir_mux = 2'b01;    // 单源：I/U/J-type（第二个操作数是立即数）
end


// ╔══════════════════════════════════════════════════════════════════════════╗
// ║  第八组：LUI 控制寄存器（reg_u_type_lui）                                  ║
// ╚══════════════════════════════════════════════════════════════════════════╝
//
//  ☆ 设计理由 —— 为什么 LUI 需要额外的一级寄存器延迟：
//
//     LUI 指令的语义是将 20 位立即数存入 rd 的高 20 位，低 12 位为 0。
//     在 ID 阶段译码出的 u_type_lui 是组合逻辑信号。而 rd 的写回发生在
//     WB 阶段（ID 之后 3 个时钟周期）。
//
//     如果直接使用组合逻辑的 u_type_lui 驱动 WB 阶段的写回逻辑，会产生一条
//     从 ID 阶段直接到 WB 阶段的组合逻辑路径：
//        ID(Control_unit) → WB(reg_file write logic)
//     这条路径跨越了 EX 和 MEM 两个阶段，时序极长，可能成为关键路径瓶颈。
//
//     因此设计者选择在 ID 阶段末尾用一个寄存器将 LUI 标志寄存一拍：
//        ID 组合: u_type_lui → [reg] → reg_u_type_lui
//     然后 reg_u_type_lui 随流水线寄存器逐级传递到 WB 阶段：
//        ID→EX→MEM→WB（每级寄存器一拍，共 3 拍到达 WB）
//
//     同理，AUIPC 也可能有类似需求，但当前设计中没有 reg_u_type_auipc 寄存器，
//     可能是因为 AUIPC 的立即数在 EX 阶段已与 PC 相加完成，不需要特殊处理的缘故。
//     LUI 的特殊性在于：它需要绕过 ALU 实际运算，仅做立即数放置。
//
//  ☆ 复位行为：
//     reset 时 reg_u_type_lui 清零，确保复位后不会误写寄存器。

always @(posedge clk or posedge reset) begin
    if (reset)
        reg_u_type_lui <= 1'b0;       // 复位：清除 LUI 标志
    else
        reg_u_type_lui <= u_type_lui; // 将 ID 阶段组合译码结果寄存一拍
end


endmodule
