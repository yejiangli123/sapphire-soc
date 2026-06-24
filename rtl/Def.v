// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║                      Sapphire-SoC 全局宏定义文件 (Def.v)                         ║
// ╚══════════════════════════════════════════════════════════════════════════════╝
//
// 【文件作用】
//   本文件定义了整个 Sapphire-SoC 项目中所有模块共享的 Verilog 宏常量。
//   通过统一的宏定义保证各模块间编码一致，避免"魔术数字"带来的调试困难。
//   各子模块通过 `include "Def.v"` 引用这些宏，实现单一真相源 (single source of truth)。
//
// 【架构约定】
//   - 指令 opcode / funct3 / funct7 严格遵循 RISC-V RV32IM 非特权规格 Vol.1
//   - ALU 操作码自行编码，与 RISC-V 指令编码相互独立，通过译码单元 (decoder) 桥接
//   - M 扩展乘除法单元采用多周期状态机，状态宏在此统一定义
//   - AXI4 总线参数遵循 AMBA AXI4 协议 v2.0
//   - 缓存参数基于 2KB 直接映射结构，I-Cache 和 D-Cache 对称设计
//
// 【如何添加新宏】
//   1. 确定宏属于哪个功能组，在对应区域添加
//   2. 写清楚每个 bit 的含义和合法取值
//   3. 注明该宏被哪些模块使用
//   4. 不要修改已有宏的值——可能破坏现有设计
//
// ═══════════════════════════════════════════════════════════════════════════════


// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║ 第1组：RISC-V 指令操作码 (major opcode, inst[6:0])                             ║
// ╚══════════════════════════════════════════════════════════════════════════════╝
//
// 【设计背景】
//   RISC-V 基本整数指令集 (RV32I) 使用固定的 7-bit major opcode 区分指令大类。
//   这些 opcode 直接取自 RISC-V 非特权规格 Vol.1, Chapter 25 (RV32/64G 指令集列表)。
//
// 【使用位置】
//   译码单元 (decoder) 解析 inst[6:0] 后与这些宏比较，决定后续流水线控制逻辑。
//   控制器 (controller) 根据 opcode 生成 ALU 操作选择、寄存器写使能、
//   存储器访问使能、分支跳转等控制信号。
//
// 【编码说明】
//   opcode 的低两位 [1:0] 通常是 2'b11 表示 32-bit 指令 (非压缩指令)，
//   但这不是绝对规则 (例如 LOAD 和 STORE 也是 2'b11)。
//   此 SoC 不包含 C 扩展 (压缩指令)，因此所有指令的 inst[1:0] 都是 2'b11。

`define OPCODE_LOAD     7'b0000011   // LOAD 类: LB/LH/LW/LBU/LHU → 从存储器读数据到寄存器
                                     //   被使用: decoder, controller, LSU
`define OPCODE_MISC_MEM 7'b0001111   // 存储器屏障: FENCE (串行化访存) / FENCE.I (指令缓存同步)
                                     //   被使用: decoder, controller
                                     //   当前实现通常按 NOP 处理 (单核无需 memory ordering)
`define OPCODE_OP_IMM   7'b0010011   // 立即数算术: ADDI/SLTI/SLTIU/XORI/ORI/ANDI/SLLI/SRLI/SRAI
                                     //   被使用: decoder, controller, ALU
`define OPCODE_AUIPC    7'b0010111   // AUIPC: 将 PC + (imm << 12) 写入目标寄存器 → 构建 PC 相对地址
                                     //   被使用: decoder, controller, ALU
`define OPCODE_STORE    7'b0100011   // STORE 类: SB/SH/SW → 将寄存器数据写入存储器
                                     //   被使用: decoder, controller, LSU
`define OPCODE_OP       7'b0110011   // 寄存器-寄存器算术: ADD/SUB/SLL/SLT/SLTU/XOR/SRL/SRA/OR/AND
                                     //   以及 M 扩展的乘除法 (funct7[5]=1 时)
                                     //   被使用: decoder, controller, ALU, mul_div_unit
`define OPCODE_LUI      7'b0110111   // LUI: 将 20-bit 立即数左移 12 位装入目标寄存器高位
                                     //   被使用: decoder, controller
`define OPCODE_BRANCH   7'b1100011   // 条件分支: BEQ/BNE/BLT/BGE/BLTU/BGEU → 根据比较结果跳转
                                     //   被使用: decoder, controller, branch_unit
`define OPCODE_JALR     7'b1100111   // JALR: 跳转并链接寄存器 → PC = (rs1 + imm) & ~1, rd = PC+4
                                     //   用于: 函数返回 (ret), 间接调用, switch-case 跳转表
                                     //   被使用: decoder, controller
`define OPCODE_JAL      7'b1101111   // JAL: 跳转并链接 → PC = PC + imm, rd = PC+4
                                     //   用于: 函数调用 (call), 无条件跳转
                                     //   被使用: decoder, controller
`define OPCODE_SYSTEM   7'b1110011   // 系统指令: CSRRW/CSRRS/CSRRC/CSRRWI/CSRRSI/CSRRCI (CSR 访问)
                                     //   + ECALL (环境调用) / EBREAK (断点) / MRET (机器模式返回)
                                     //   被使用: decoder, controller, CSR 单元

// ═══════════════════════════════════════════════════════════════════════════════


// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║ 第2组：立即数类型编码 (用于 imm_gen 模块的立即数多路选择)                         ║
// ╚══════════════════════════════════════════════════════════════════════════════╝
//
// 【设计背景】
//   RISC-V 有六种基本立即数格式: I-type, S-type, B-type, U-type, J-type，以及
//   I-type 的一个变体 (用于 LOAD 的 unsigned 加载，符号扩展策略略有不同)。
//   立即数生成单元 (imm_gen) 根据 inst 字段和这些类型码，将分散在指令不同 bit 位置
//   的立即数片段拼接并符号扩展为完整的 32-bit 操作数。
//
// 【为什么需要这些编码】
//   译码器根据 opcode + funct3 确定立即数类型，传入 imm_mux 信号给 imm_gen。
//   imm_gen 内部用这个 3-bit 选择码驱动一个多路复用器，挑选正确的拼接方案。
//
// 【注意】
//   I_type 和 I_type_load 的区别仅在于符号扩展策略：
//   I_type → 算术指令用，高位符号扩展
//   I_type_load → LOAD 用，实际上与 I_type 相同 (都是 signed 扩展)，
//     但分离出来以便未来支持不同的扩展策略或用于调试

`define I_type          3'b000   // I-type 立即数 (12-bit, inst[31:20]) → ADDI/SLTI/ANDI 等
`define I_type_load     3'b001   // I-type LOAD 立即数 (12-bit, inst[31:20]) → LB/LH/LW 等
`define S_type          3'b010   // S-type 立即数 (12-bit, inst[31:25] + inst[11:7]) → SB/SH/SW
`define B_type          3'b011   // B-type 立即数 (13-bit with LSB=0, inst[31]+inst[7]+inst[30:25]+inst[11:8])
                                 //   → BEQ/BNE/BLT 等条件分支
`define J_type          3'b100   // J-type 立即数 (21-bit with LSB=0, inst[31]+inst[19:12]+inst[20]+inst[30:21])
                                 //   → JAL 无条件跳转
`define U_type_LUI      3'b101   // U-type 立即数 (20-bit, inst[31:12]) → LUI 装入高位
`define U_type_AUIPC    3'b110   // U-type 立即数 (20-bit, inst[31:12]) → AUIPC PC 相对寻址
                                 //   注意: LUI 和 AUIPC 虽然立即数格式相同 (都是 U-type)，
                                 //   但 ALU 操作不同: LUI 直通立即数，AUIPC 加 PC，
                                 //   分离编码可以让流水线更清晰地追踪数据流

// ═══════════════════════════════════════════════════════════════════════════════


// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║ 第3组：ALU 算术逻辑单元操作码                                                  ║
// ╚══════════════════════════════════════════════════════════════════════════════╝
//
// 【设计背景】
//   这些宏定义了 ALU 内部的操作选择码 (alu_op)，由控制器根据指令译码结果生成。
//   注意：此处的编码是 SoC 内部自定义的，与 RISC-V 指令的 funct3/funct7 编码不同。
//   这样设计的好处是：
//     1. ALU 不需要知道 RISC-V 指令格式，只处理抽象的操作码 → 解耦
//     2. 可以合并功能相同的指令 (例如 ADDI 和 ADD 都映射到 ALU_ADD)
//     3. 便于添加自定义操作 (如 M 扩展的乘除法操作码)
//
// 【修复记录】(重要)
//   原 PDF 文档中的 ALU 编码表与 ALU.v 实际使用的编码不一致，已在此文件中统一修正。
//   - 统一使用 4-bit 编码以容纳 RISC-V 全部 10 种基础 ALU 操作 + M 扩展
//   - funct3 的部分操作 (如 ADD/SUB 共享 funct3=000) 通过 funct7[5] 区分，
//     但 ALU 操作码层面已将它们分离为不同编码 (ALU_ADD vs ALU_SUB)
//
// 【使用位置】
//   controller → alu_op → ALU.v (计算) + forwarding unit (判断数据相关性)

`define ALU_ADD         4'b0000   // 加法: rs1 + rs2 或 rs1 + imm (ADDI/ADD/AUIPC/JAL/JALR 的地址计算)
`define ALU_SUB         4'b0001   // 减法: rs1 - rs2 (SUB) → 也用于分支比较 (BEQ/BNE/BLT/BGE 内部用减法)
`define ALU_XOR         4'b0010   // 按位异或: rs1 ^ rs2 (XOR/XORI)
`define ALU_OR          4'b0011   // 按位或:  rs1 | rs2 (OR/ORI)
`define ALU_AND         4'b0100   // 按位与:  rs1 & rs2 (AND/ANDI)
`define ALU_SLL         4'b0101   // 逻辑左移: rs1 << rs2[4:0] (SLL/SLLI)
`define ALU_SRL         4'b0110   // 逻辑右移: rs1 >> rs2[4:0] (SRL/SRLI，高位补 0)
`define ALU_SRA         4'b0111   // 算术右移: rs1 >> rs2[4:0] (SRA/SRAI，高位补符号位)
`define ALU_SLT         4'b1000   // 有符号小于: (signed(rs1) < signed(rs2)) ? 1 : 0 (SLT/SLTI)
`define ALU_SLTU        4'b1001   // 无符号小于: (rs1 < rs2) ? 1 : 0 (SLTU/SLTIU)

// ═══════════════════════════════════════════════════════════════════════════════


// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║ 第4组：M 扩展 (乘除法) funct3 编码 — 来自 RISC-V 指令格式                        ║
// ╚══════════════════════════════════════════════════════════════════════════════╝
//
// 【设计背景】
//   RISC-V M 扩展定义了 8 条整数乘除法指令，共享 opcode=OPCODE_OP (0110011)，
//   通过 funct7[5]=1 与基本整数指令区分，再用 funct3 区分具体操作。
//   这些宏直接对应 RISC-V 规范的 funct3 字段值，用于译码阶段识别具体指令。
//
// 【操作语义】
//   MUL     → rd = (rs1 * rs2)[31:0]              // 乘法低 32 位
//   MULH    → rd = (signed(rs1) * signed(rs2))[63:32]  // 有符号乘法高 32 位
//   MULHSU  → rd = (signed(rs1) * unsigned(rs2))[63:32] // 有符号×无符号 高 32 位
//   MULHU   → rd = (unsigned(rs1) * unsigned(rs2))[63:32] // 无符号乘法高 32 位
//   DIV     → rd = signed(rs1) / signed(rs2)         // 有符号除法
//   DIVU    → rd = unsigned(rs1) / unsigned(rs2)     // 无符号除法
//   REM     → rd = signed(rs1) % signed(rs2)         // 有符号取余
//   REMU    → rd = unsigned(rs1) % unsigned(rs2)     // 无符号取余
//
// 【特殊条件】除零: DIV/DIVU/REM/REMU 除数为 0 时:
//   DIV(U)  → rd = -1 (即 0xFFFFFFFF)
//   REM(U)  → rd = rs1 (被除数)
//   上溢: DIV 中 -2^31 ÷ -1 时 → rd = -2^31 (0x80000000)

`define MULDIV_MUL      3'b000   // MUL    — 乘法低 32 位
`define MULDIV_MULH     3'b001   // MULH   — 有符号×有符号 高 32 位
`define MULDIV_MULHSU   3'b010   // MULHSU — 有符号×无符号 高 32 位
`define MULDIV_MULHU    3'b011   // MULHU  — 无符号×无符号 高 32 位
`define MULDIV_DIV      3'b100   // DIV    — 有符号除法
`define MULDIV_DIVU     3'b101   // DIVU   — 无符号除法
`define MULDIV_REM      3'b110   // REM    — 有符号取余
`define MULDIV_REMU     3'b111   // REMU   — 无符号取余

// ═══════════════════════════════════════════════════════════════════════════════


// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║ 第5组：M 扩展 ALU 操作码 — 通过 ALU 流水线传递至乘除法单元的内部编码               ║
// ╚══════════════════════════════════════════════════════════════════════════════╝
//
// 【设计背景】
//   乘除法单元 (mul_div_unit) 是 ALU 流水线的一个附属模块。当控制器检测到 M 扩展指令时，
//   会将对应的 ALU 操作码通过 alu_op 信号沿流水线传递到执行阶段，mul_div_unit 根据
//   该操作码启动相应的乘除法状态机。
//
// 【为什么 M 扩展要独立的操作码】
//   - M 扩展指令的 opcode 是 OPCODE_OP，与普通寄存器运算相同
//   - 控制器通过 funct7[5]=1 识别 M 指令，分配专用的 ALU 操作码
//   - ALU.v 收到这些操作码时不自己计算，而是触发 mul_div_unit 并等待其完成
//   - 这种设计保持了 ALU 接口的统一性 (所有运算都走 alu_op 通道)
//
// 【编码复用说明】
//   DIV 和 REM 使用相同的 ALU 操作码 (ALU_DIV = ALU_REM = 4'b1110)，
//   DIVU 和 REMU 同理 (ALU_DIVU = ALU_REMU = 4'b1111)。
//   ★【已修复】mul_div_unit 通过新增的 funct3_in 端口区分除法与取余：
//       funct3=MULDIV_DIV(100) → 返回商, funct3=MULDIV_REM(110) → 返回余数
//       funct3=MULDIV_DIVU(101)→ 返回商, funct3=MULDIV_REMU(111)→ 返回余数
//   这样复用是因为除法和取余的计算过程完全相同 (都需要迭代除法算法)，
//   仅在最后返回结果时选择商还是余数。

`define ALU_MUL         4'b1010   // 乘法低 32 位 → MUL
`define ALU_MULH        4'b1011   // 有符号×有符号 高 32 位 → MULH
`define ALU_MULHSU      4'b1100   // 有符号×无符号 高 32 位 → MULHSU
`define ALU_MULHU       4'b1101   // 无符号×无符号 高 32 位 → MULHU
`define ALU_DIV         4'b1110   // 有符号除法 → DIV，同时复用为 REM 的操作码
`define ALU_DIVU        4'b1111   // 无符号除法 → DIVU，同时复用为 REMU 的操作码
`define ALU_REM         4'b1110   // 有符号取余 → 与 ALU_DIV 相同，mul_div_unit 内部通过 funct3 区分
`define ALU_REMU        4'b1111   // 无符号取余 → 与 ALU_DIVU 相同，mul_div_unit 内部通过 funct3 区分

// ═══════════════════════════════════════════════════════════════════════════════


// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║ 第6组：M 扩展乘除法单元状态机                                               ║
// ╚══════════════════════════════════════════════════════════════════════════════╝
//
// 【设计背景】
//   整数乘法 (32×32→64→截断至32) 和除法 (32÷32) 无法在单周期内完成。
//   mul_div_unit 使用有限状态机 (FSM) 控制多周期迭代计算。
//   这三个宏定义 FSM 的状态编码，供 mul_div_unit 内部使用，同时也通过
//   md_state 输出给流水线控制器，用于生成 stall 信号。
//
// 【为什么需要多周期】
//   - 乘法: 串行移位相加算法需要 32 个周期 (每周期处理 1-bit 乘数)
//   - 除法: 非恢复型 (non-restoring) 除法算法需要 32 个周期
//   - 流水线必须在这期间暂停 (stall) ID 阶段，直到 M 单元完成运算
//
// 【状态转移】
//   IDLE → (检测到 M 指令且操作数就绪) → BUSY → (迭代完成) → DONE → (结果被读取) → IDLE
//   处于 BUSY 状态时，流水线控制器拉高 stall 信号，阻止新指令进入执行阶段。

`define MD_IDLE         2'b00   // 空闲: 乘除法单元无操作，流水线正常流动
`define MD_BUSY         2'b01   // 计算中: 正在执行迭代乘法/除法，流水线暂停 (stall)
`define MD_DONE         2'b10   // 完成: 计算结果就绪，等待被提交到写回阶段

// ═══════════════════════════════════════════════════════════════════════════════


// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║ 第7组：分支类型编码 (branch_unit 使用)                                          ║
// ╚══════════════════════════════════════════════════════════════════════════════╝
//
// 【设计背景】
//   分支单元 (branch_unit) 根据 ALU 比较结果决定是否跳转。
//   这里的编码直接对应 RISC-V 条件分支的 funct3 字段 (inst[14:12])。
//   这样设计的好处是译码器可以直接将 funct3 传入 branch_unit 作为分支类型，
//   无需额外转换，简化译码逻辑。
//
// 【跳转条件】
//   funct3[2:1] 编码比较类型: 00=等于, 10/11=小于(符号/无符号)
//   funct3[0]   取反:    0=真时跳转, 1=假时跳转
//   因此:
//   - BEQ  (000): funct3[1]=0 → =,   funct3[0]=0 → 跳转当 rs1==rs2
//   - BNE  (001): funct3[1]=0 → =,   funct3[0]=1 → 跳转当 rs1!=rs2
//   - BLT  (100): funct3[1]=1 → <s, funct3[0]=0 → 跳转当 rs1< rs2 (有符号)
//   - BGE  (101): funct3[1]=1 → <s, funct3[0]=1 → 跳转当 rs1>=rs2 (有符号)
//   - BLTU (110): funct3[1]=1 → <u, funct3[0]=0 → 跳转当 rs1< rs2 (无符号)
//   - BGEU (111): funct3[1]=1 → <u, funct3[0]=1 → 跳转当 rs1>=rs2 (无符号)

`define BRANCH_EQ       3'b000   // 相等时跳转     (BEQ)
`define BRANCH_NE       3'b001   // 不等时跳转     (BNE)
`define BRANCH_LT       3'b100   // 有符号小于时跳转 (BLT)
`define BRANCH_GE       3'b101   // 有符号大于等于时跳转 (BGE)
`define BRANCH_LTU      3'b110   // 无符号小于时跳转 (BLTU)
`define BRANCH_GEU      3'b111   // 无符号大于等于时跳转 (BGEU)

// ═══════════════════════════════════════════════════════════════════════════════


// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║ 第8组：控制信号宏                                                              ║
// ╚══════════════════════════════════════════════════════════════════════════════╝
//
// 【设计背景】
//   将常用的 1-bit 控制信号电平用有意义的宏名表示，提高代码可读性。
//   避免了 `1'b1`, `1'b0` 的裸写，让代码意图更加明确。
//
// 【使用示例】
//   assign mem_read_en  = (op == OPCODE_LOAD)  ? MEM_READ  : ~MEM_READ;
//   assign mem_write_en = (op == OPCODE_STORE) ? MEM_WRITE : ~MEM_WRITE;
//   assign reg_write_en = (rd != 5'd0)         ? REG_WRITE : ~REG_WRITE;

`define MEM_READ        1'b1   // 存储器读使能   → LOAD 指令时有效
`define MEM_WRITE       1'b1   // 存储器写使能   → STORE 指令时有效
`define REG_WRITE       1'b1   // 寄存器写使能   → 目的寄存器非 x0 时有效

// ═══════════════════════════════════════════════════════════════════════════════


// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║ 第9组：AXI4 总线协议响应码 (RRESP / BRESP)                                    ║
// ╚══════════════════════════════════════════════════════════════════════════════╝
//
// 【设计背景】
//   SoC 使用 AMBA AXI4 协议连接处理器核心与存储器/外设。
//   RRESP (读响应) 和 BRESP (写响应) 是 2-bit 信号，指示总线事务完成状态。
//   这些宏定义了四种标准 AXI 响应类型。
//
// 【使用位置】
//   LSU (Load/Store Unit) 解析 AXI 响应 → 触发异常或重试
//   当前实现中 SLVERR/DECERR 通常触发 trap (异常)，交由异常处理程序处理

`define AXI_RESP_OKAY   2'b00   // 正常访问成功 → 数据有效
`define AXI_RESP_EXOKAY 2'b01   // 独占访问成功 (Exclusive OK) → 仅用于原子操作，当前 SoC 未使用
`define AXI_RESP_SLVERR 2'b10   // 从设备错误 (Slave Error) → 从设备收到请求但无法完成 (如访问越界)
`define AXI_RESP_DECERR 2'b11   // 译码错误 (Decode Error) → 地址未映射到任何从设备

// ═══════════════════════════════════════════════════════════════════════════════


// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║ 第10组：AXI4 突发传输类型 (ARBURST / AWBURST)                                  ║
// ╚══════════════════════════════════════════════════════════════════════════════╝
//
// 【设计背景】
//   AXI4 支持三种突发模式。当前 SoC 的缓存填充 (cache line fill) 使用 INCR 模式
//   实现连续地址的多拍传输 (8 words per burst for a 32-byte cache line)。
//   处理器核心的普通 LSU 访问通常使用单拍传输 (INCR, burst_len=1)。

`define AXI_BURST_FIXED 2'b00   // 固定地址突发 → 所有传输使用同一地址 (FIFO 类外设常用)
                                //   当前 SoC 未使用此模式
`define AXI_BURST_INCR  2'b01   // 增量突发 → 地址递增 (每拍 + transfer_size)
                                //   缓存行填充/写回使用此模式，8 拍连续地址
`define AXI_BURST_WRAP  2'b10   // 回环突发 → 地址到边界后回绕 (用于缓存行边界对齐)
                                //   当前 SoC 未使用此模式

// ═══════════════════════════════════════════════════════════════════════════════


// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║ 第11组：AXI4 传输宽度编码 (ARSIZE / AWSIZE)                                    ║
// ╚══════════════════════════════════════════════════════════════════════════════╝
//
// 【设计背景】
//   定义 AXI 总线上单次数据传输的字节宽度。编码值 N 表示 2^N 字节。
//   RV32 处理器主要使用 WORD (4 字节) 传输，LOAD/STORE 字节/半字时
//   通过写选通 (WSTRB) 信号控制具体哪些字节有效。

`define AXI_SIZE_BYTE   3'b000   // 1 字节 (2^0) → LB/SB
`define AXI_SIZE_HALF   3'b001   // 2 字节 (2^1) → LH/SH
`define AXI_SIZE_WORD   3'b010   // 4 字节 (2^2) → LW/SW (RV32 主要使用)

// ═══════════════════════════════════════════════════════════════════════════════


// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║ 第12组：缓存结构参数 — I-Cache (指令缓存)                                        ║
// ╚══════════════════════════════════════════════════════════════════════════════╝
//
// 【架构概述】
//   Sapphire-SoC 采用哈佛架构的独立 L1 I-Cache 和 D-Cache。
//   都是直接映射 (direct-mapped) 结构，无组相联，设计简单且时序路径短。
//
// 【I-Cache 地址映射】  (32-bit 地址 → 各字段)
//   ┌──────────────┬────────────┬───────────┬──────────┐
//   │    TAG       │   INDEX    │   WORD    │   BYTE   │
//   │   21 bits    │   6 bits   │  3 bits   │  2 bits  │
//   │  [31:11]     │  [10:5]    │  [4:2]    │  [1:0]   │
//   └──────────────┴────────────┴───────────┴──────────┘
//
// 【参数计算】
//   - 每行 8 个 32-bit 字 = 2^3 字 → WORD_BITS = 3
//   - 64 个缓存行 = 2^6 → INDEX_BITS = 6
//   - 总容量: 64 行 × 8 字 × 4 字节 = 2048 字节 = 2KB
//   - TAG 位宽: 32 - 2(byte) - 3(word) - 6(index) = 21 bits
//
// 【I-Cache 特点】
//   - 只读不写 → 无脏位 (dirty bit)，无写回逻辑
//   - 缺失时通过 AXI4 读通道填充整行 (INCR burst, 8 beats)
//   - 命中延迟: 1 周期 (组合逻辑 TAG 比较 + RAM 读出)

`define ICACHE_WORD_BITS  3     // 每行 8 个字 = 2^3 → 用于字偏移 (inst[4:2])
`define ICACHE_NUM_LINES  64    // 缓存行总数 = 64 → 决定了索引位宽
`define ICACHE_INDEX_BITS 6     // log2(64) = 6 → 用于选择缓存行 (inst[10:5])
`define ICACHE_TAG_BITS   21    // 32 - 2(byte) - 3(word) - 6(index) → TAG 比较宽度

// ═══════════════════════════════════════════════════════════════════════════════


// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║ 第13组：缓存结构参数 — D-Cache (数据缓存)                                        ║
// ╚══════════════════════════════════════════════════════════════════════════════╝
//
// 【架构概述】
//   D-Cache 与 I-Cache 采用完全对称的结构，同样是 2KB 直接映射。
//   但 D-Cache 支持读写，具有以下额外特性：
//
// 【D-Cache 与 I-Cache 的关键区别】
//   1. D-Cache 有 write-back 机制 → 含脏位 (dirty bit)，替换时需写回
//   2. D-Cache 支持字节/半字/字访问 → 通过 LSU 的写选通 (WSTRB) 实现
//   3. D-Cache 替换时先写回脏行，再填充新行 → 2 次 AXI 事务
//   4. D-Cache SRAM 有独立的写使能 → 支持部分字写入
//
// 【地址映射】 (与 I-Cache 相同)
//   ┌──────────────┬────────────┬───────────┬──────────┐
//   │    TAG       │   INDEX    │   WORD    │   BYTE   │
//   │   21 bits    │   6 bits   │  3 bits   │  2 bits  │
//   └──────────────┴────────────┴───────────┴──────────┘
//
// 【设计决策：为什么 D-Cache 和 I-Cache 参数完全相同】
//   - 简化设计: 相同的 SRAM 宏单元、相同的 TAG 比较器、相同的控制逻辑
//   - 2KB 对 I 和 D 分别足够: 跑 Dhrystone/CoreMark 等 benchmark 时命中率 >95%
//   - 面积约束: 更大的缓存需要更大的 SRAM，而 2KB 是 FPGA Block RAM 友好的大小
//   - 路径延迟: 直接映射 + 小容量 → 关键路径短，有利于提高主频

`define DCACHE_WORD_BITS  3     // 每行 8 个字 → D-Cache 字偏移
`define DCACHE_NUM_LINES  64    // 缓存行总数 = 64
`define DCACHE_INDEX_BITS 6     // log2(64) = 6 → D-Cache 索引位宽
`define DCACHE_TAG_BITS   21    // D-Cache TAG 比较宽度

// ═══════════════════════════════════════════════════════════════════════════════
//                             End of Def.v
// ═══════════════════════════════════════════════════════════════════════════════
