// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  TOP_levelSOc.v — Sapphire SoC 顶层集成（RV32IM + 系统总线 + 外设）          ║
// ║                                                                            ║
// ║  【模块定位与架构角色】                                                       ║
// ║  本模块是整个 Sapphire SoC 的顶层集成文件，相当于芯片的"顶层网表"。它完成     ║
// ║  以下核心任务：                                                               ║
// ║    1. 实例化 RV32IM 处理器核心（rv32im_core）                                ║
// ║    2. 实例化系统总线（system_bus）—— 地址译码 + 数据多路复用                 ║
// ║    3. 实例化块内存 BRAM（bram）—— 指令/数据统一存储                          ║
// ║    4. 实例化所有外设：Timer、GPIO、UART、PLIC                                ║
// ║    5. 将所有模块的接口通过 wire 互连，形成完整的 SoC                          ║
// ║                                                                            ║
// ║  【升级历史】                                                                 ║
// ║  相对于简单的 RV32I SoC（TOP_levelSOc_simple.v），本版本进行了以下升级：      ║
// ║    - RV32IM 核心：增加了 M 扩展（整数乘除法硬件单元 mul_div_unit）           ║
// ║    - Cache 旁路模式：本顶层当前使用 cache bypass 模式（缓存集成在核内，      ║
// ║      但通过 unified bus 直连存储器）。注释中预留了 I-Cache + D-Cache 的       ║
// ║      独立主端口和总线仲裁器（bus_arbiter）的接线方案。                        ║
// ║    - BRAM 带寄存器响应：组合读数据路径，满足单周期取指/加载时序要求。        ║
// ║                                                                            ║
// ║  【完整 SoC 架构框图】                                                       ║
// ║                                                                            ║
// ║                         ┌─────────────────────────────────────┐            ║
// ║                         │            TOP_levelSOc             │            ║
// ║                         │  (riscv_soc)                        │            ║
// ║                         │                                     │            ║
// ║  ┌──────────┐           │  ┌─────────────────────────────┐    │            ║
// ║  │  clk     │───────────┼─▶│                             │    │            ║
// ║  │  reset   │───────────┼─▶│     rv32im_core             │    │            ║
// ║  └──────────┘           │  │  (5-stage pipeline core)    │    │            ║
// ║                         │  │                             │    │            ║
// ║                         │  │  ┌─────┐ ┌─────┐ ┌─────┐   │    │            ║
// ║                         │  │  │ IF  │▶│ ID  │▶│ EX  │   │    │            ║
// ║                         │  │  │取指 │ │译码 │ │执行 │   │    │            ║
// ║                         │  │  └─────┘ └─────┘ └──┬──┘   │    │            ║
// ║                         │  │                     │      │    │            ║
// ║                         │  │  ┌─────┐ ┌─────┐    │      │    │            ║
// ║                         │  │  │ WB  │◀│ MEM │◀───┘      │    │            ║
// ║                         │  │  │写回 │ │访存 │           │    │            ║
// ║                         │  │  └─────┘ └─────┘           │    │            ║
// ║                         │  │                             │    │            ║
// ║                         │  │  unified bus (cache bypass) │    │            ║
// ║                         │  └─────────────┬───────────────┘    │            ║
// ║                         │                │                    │            ║
// ║                         │  ┌─────────────▼───────────────┐    │            ║
// ║                         │  │      system_bus              │    │            ║
// ║                         │  │  地址译码 + 数据 MUX          │    │            ║
// ║                         │  │  ┌─────────────────────────┐ │    │            ║
// ║                         │  │  │ sel_mem  → 0x0xxx_xxxx  │ │    │            ║
// ║                         │  │  │ sel_gpio → 0x20xx_xxxx  │ │    │            ║
// ║                         │  │  │ sel_uart → 0x30xx_xxxx  │ │    │            ║
// ║                         │  │  │ sel_timer→ 0x40xx_xxxx  │ │    │            ║
// ║                         │  │  │ sel_plic → 0x50xx_xxxx  │ │    │            ║
// ║                         │  │  └─────────────────────────┘ │    │            ║
// ║                         │  └──┬──────┬──────┬──────┬──────┘    │            ║
// ║                         │     │      │      │      │           │            ║
// ║                         │  ┌──▼──┐ ┌─▼──┐ ┌─▼──┐ ┌▼────┐ ┌───▼───┐       ║
// ║                         │  │BRAM │ │GPIO│ │UART│ │Timer│ │ PLIC  │       ║
// ║                         │  │64KB │ │    │ │    │ │     │ │       │       ║
// ║                         │  └─────┘ └──┬─┘ └──┬─┘ └──┬──┘ └───┬───┘       ║
// ║                         │             │      │      │        │           ║
// ║                         └─────────────┼──────┼──────┼────────┼───────────┘
// ║                                       │      │      │        │
// ║                                gpio_out  uart_tx  (内部)  plic_irq
// ║                                gpio_in   uart_rx  timer_irq plic_irq_id
// ║                                            (UART IRQ)
// ║                                                                            ║
// ║  【数据流路径说明】                                                           ║
// ║                                                                            ║
// ║  1. 取指路径（IF）:                                                          ║
// ║     PC → instruction_memory（经 system_bus → BRAM）→ instr → IF/ID 流水线  ║
// ║     当前实现中，instruction_memory 通过 unified bus 的 cpu_addr/cpu_rdata   ║
// ║     访问 BRAM。system_bus 根据地址译码，将访问路由到 BRAM（sel_mem=1），     ║
// ║     BRAM 通过组合读在同一周期返回指令字。                                     ║
// ║                                                                            ║
// ║  2. 加载路径（MEM）:                                                         ║
// ║     EX 计算出地址 → MEM 阶段 → unified bus（cpu_addr/cpu_re）               ║
// ║     → system_bus 译码 → BRAM/外设 返回数据 → load_unit 对齐/符号扩展        ║
// ║     → MEM/WB 流水线 → WB 阶段写回寄存器文件                                  ║
// ║                                                                            ║
// ║  3. 存储路径（MEM）:                                                         ║
// ║     EX 计算出地址 + rs2 数据 → MEM 阶段 → store_unit 字节对齐               ║
// ║     → unified bus（cpu_addr/cpu_wdata/cpu_we）→ system_bus 译码            ║
// ║     → 目标外设的写使能门控有效 → 下个时钟沿写入                                ║
// ║                                                                            ║
// ║  4. 中断路径:                                                                ║
// ║     Timer/UART → timer_irq/uart_irq → PLIC 仲裁（优先级编码）              ║
// ║     → plic_irq + plic_irq_id → rv32im_core 的 irq/irq_id 端口             ║
// ║     → 核内部中断响应逻辑（跳转到 mtvec 指向的中断服务程序）                   ║
// ║                                                                            ║
// ║  【地址映射总表】                                                             ║
// ║  ┌─────────────────────┬────────────┬──────────────────┐                   ║
// ║  │ 地址范围             │ 外设       │ 译码条件          │                   ║
// ║  ├─────────────────────┼────────────┼──────────────────┤                   ║
// ║  │ 0x0000_0000~0x0FFF_ │ BRAM       │ addr[31:28]==0   │                   ║
// ║  │ FFFF (256MB)        │            │                  │                   ║
// ║  ├─────────────────────┼────────────┼──────────────────┤                   ║
// ║  │ 0x2000_0000~0x20FF_ │ GPIO       │ addr[31:24]==0x20│                   ║
// ║  │ FFFF (16MB)         │            │                  │                   ║
// ║  ├─────────────────────┼────────────┼──────────────────┤                   ║
// ║  │ 0x3000_0000~0x30FF_ │ UART       │ addr[31:24]==0x30│                   ║
// ║  │ FFFF (16MB)         │            │                  │                   ║
// ║  ├─────────────────────┼────────────┼──────────────────┤                   ║
// ║  │ 0x4000_0000~0x40FF_ │ Timer      │ addr[31:24]==0x40│                   ║
// ║  │ FFFF (16MB)         │            │                  │                   ║
// ║  ├─────────────────────┼────────────┼──────────────────┤                   ║
// ║  │ 0x5000_0000~0x50FF_ │ PLIC       │ addr[31:24]==0x50│                   ║
// ║  │ FFFF (16MB)         │            │                  │                   ║
// ║  └─────────────────────┴────────────┴──────────────────┘                   ║
// ║                                                                            ║
// ║  【设计决策与权衡】                                                           ║
// ║                                                                            ║
// ║  1. 统一总线 vs 哈佛总线                                                      ║
// ║     当前使用统一总线（unified bus），CPU 通过单一的地址/数据端口访问指令      ║
// ║     和数据。这与 RISC-V 的冯·诺依曼风格一致，但存在结构冒险（取指和数据访问   ║
// ║     争用同一总线）。注释中预留了 I$/D$ 独立端口和 bus_arbiter 的接线方案。    ║
// ║                                                                            ║
// ║  2. 组合读 vs 寄存器读                                                        ║
// ║     system_bus 和 BRAM 均采用组合读（地址变化后数据在同一周期有效），         ║
// ║     这保证了单周期取指和加载，无需等待状态。代价是关键路径较长。              ║
// ║                                                                            ║
// ║  3. 广播地址/数据 + 使能门控                                                   ║
// ║     system_bus 将 cpu_addr 和 cpu_wdata 广播到所有外设，仅通过写使能门控     ║
// ║     （cpu_we && sel_xxx）来防止误写入。这简化了布线，但所有外设端口都有       ║
// ║     信号翻转（功耗略高）。                                                     ║
// ║                                                                            ║
// ║  4. Cache Bypass 模式                                                        ║
// ║     当前版本中，rv32im_core 内部的 I-Cache 和 D-Cache 被旁路（bypass），      ║
// ║     指令和数据直接通过 unified bus 访问 BRAM。这是为了在验证初期简化调试，    ║
// ║     避免缓存一致性/命中/缺失等复杂问题。待核心功能验证通过后，可启用缓存。    ║
// ║                                                                            ║
// ║  5. 中断架构                                                                  ║
// ║     使用 PLIC（平台级中断控制器）进行中断仲裁：Timer 和 UART 的中断信号       ║
// ║     送入 PLIC，PLIC 根据优先级编码选出最高优先级中断，将 irq 和 irq_id       ║
// ║     送给 CPU。CPU 中 ISR 通过读 PLIC 的 CLAIM 寄存器获取中断 ID，处理完毕    ║
// ║     后写 COMPLETE 寄存器。                                                    ║
// ║                                                                            ║
// ╚══════════════════════════════════════════════════════════════════════════════╝

// ============================================================
//  头文件包含: Def.v — 全局宏定义
// ============================================================
// Def.v 定义了整个 SoC 所需的所有宏常量，按功能分为以下几组:
//
// ┌──────────────────────────────────────────────────────────────────┐
// │ 第一组：指令操作码 (Instruction Opcodes, 7-bit)                    │
// │   定义了 RISC-V RV32I 基础指令集的 10 种 opcode，以及 RV32M 和     │
// │   SYSTEM 指令的 opcode。Decoder 模块用这些宏来识别指令类型。        │
// │                                                                  │
// │   `OPCODE_LOAD     7'b0000011  // lw/lh/lb/lbu/lhu               │
// │   `OPCODE_MISC_MEM 7'b0001111  // FENCE / FENCE.I                │
// │   `OPCODE_OP_IMM   7'b0010011  // addi/slti/xori/ori/andi/slli等 │
// │   `OPCODE_AUIPC    7'b0010111  // auipc (PC + 立即数<<12)        │
// │   `OPCODE_STORE    7'b0100011  // sw/sh/sb                       │
// │   `OPCODE_OP       7'b0110011  // add/sub/slt/xor/or/and/sll等   │
// │   `OPCODE_LUI      7'b0110111  // lui (加载立即数到高位)          │
// │   `OPCODE_BRANCH   7'b1100011  // beq/bne/blt/bge/bltu/bgeu      │
// │   `OPCODE_JALR     7'b1100111  // jalr (寄存器间接跳转)           │
// │   `OPCODE_JAL      7'b1101111  // jal (跳转并链接)               │
// │   `OPCODE_SYSTEM   7'b1110011  // CSRRx + ECALL/EBREAK/MRET      │
// │                                                                  │
// │   设计说明：opcode 的 bit[6:5] 编码了指令大类：                    │
// │     - 00: LOAD/STORE/MISC_MEM/OP_IMM/OP                          │
// │     - 01: AUIPC/LUI                                              │
// │     - 11: BRANCH/JALR/JAL/SYSTEM                                 │
// │   这是 RISC-V ISA 规范的有意设计，使译码逻辑可以用少量的高位       │
// │   比较来快速判断指令类别。                                         │
// └──────────────────────────────────────────────────────────────────┘
//
// ┌──────────────────────────────────────────────────────────────────┐
// │ 第二组：立即数类型 (Immediate Types, 3-bit)                        │
// │   RISC-V 有 6 种立即数编码格式（I/S/B/U/J + I_load变体），        │
// │   对应 instr[31:7] 的不同位域拼接方式。imm_gen 模块根据此选择      │
// │   信号从指令字的 25-bit 高位域中提取并符号扩展为 32-bit 立即数。   │
// │                                                                  │
// │   `I_type      3'b000  // I型: 算术/逻辑立即数 (addi/slti等)     │
// │   `I_type_load 3'b001  // I型_load: 加载指令 (lw/lh/lb)         │
// │                         注：与普通I型编码相同，语义区分用于控制    │
// │   `S_type      3'b010  // S型: 存储指令 (sw/sh/sb)               │
// │   `B_type      3'b011  // B型: 分支指令 (beq/bne/blt等)          │
// │   `J_type      3'b100  // J型: jal 跳转指令                      │
// │   `U_type_LUI  3'b101  // U型: lui 加载高位立即数                │
// │   `U_type_AUIPC 3'b110 // U型: auipc PC+立即数高位               │
// │                                                                  │
// │   设计说明：I_type 和 I_type_load 虽然编码相同（都是 I 型），      │
// │   但在控制逻辑中需要区分，因为 load 指令会触发内存读操作，         │
// │   而普通 I 型 ALU 指令不访问内存。                                 │
// └──────────────────────────────────────────────────────────────────┘
//
// ┌──────────────────────────────────────────────────────────────────┐
// │ 第三组：ALU 操作码 (ALU Operations, 4-bit)                         │
// │   定义了 ALU 支持的所有运算操作。ALU_Control 模块根据指令的        │
// │   funct3/funct7 字段生成这些操作码，ALU 模块据此执行运算。         │
// │                                                                  │
// │   `ALU_ADD  4'b0000  // 加法 (add/addi/auipc/地址计算)           │
// │   `ALU_SUB  4'b0001  // 减法 (sub)                               │
// │   `ALU_XOR  4'b0010  // 按位异或 (xor/xori)                      │
// │   `ALU_OR   4'b0011  // 按位或 (or/ori)                          │
// │   `ALU_AND  4'b0100  // 按位与 (and/andi)                        │
// │   `ALU_SLL  4'b0101  // 逻辑左移 (sll/slli)                      │
// │   `ALU_SRL  4'b0110  // 逻辑右移 (srl/srli)                      │
// │   `ALU_SRA  4'b0111  // 算术右移 (sra/srai)                      │
// │   `ALU_SLT  4'b1000  // 有符号小于置位 (slt/slti)               │
// │   `ALU_SLTU 4'b1001  // 无符号小于置位 (sltu/sltiu)             │
// │                                                                  │
// │   设计说明：                                                       │
// │   - 编码 4'bxxxx，高 2 位暗示操作类别：00=加减, 01=逻辑/移位,     │
// │     10=比较。这使得 ALU 内部可以用高位做粗粒度译码。               │
// │   - 注意 ADD 用于多种场景：add 指令、addi 指令、auipc、load/store  │
// │     地址计算、JAL/JALR 的 PC+4 计算。减法仅用于 sub 指令。         │
// └──────────────────────────────────────────────────────────────────┘
//
// ┌──────────────────────────────────────────────────────────────────┐
// │ 第四组：M 扩展相关 (M-Extension, RV32M)                             │
// │   RV32M 扩展提供了硬件整数乘除法指令。                              │
// │                                                                  │
// │   MULDIV funct3 编码 (funct3 用于区分不同的乘除指令):              │
// │   `MULDIV_MUL    3'b000  // mul     — 乘法(低32位)               │
// │   `MULDIV_MULH   3'b001  // mulh    — 有符号×有符号 高32位       │
// │   `MULDIV_MULHSU 3'b010  // mulhsu  — 有符号×无符号 高32位       │
// │   `MULDIV_MULHU  3'b011  // mulhu   — 无符号×无符号 高32位       │
// │   `MULDIV_DIV    3'b100  // div     — 有符号除法                 │
// │   `MULDIV_DIVU   3'b101  // divu    — 无符号除法                 │
// │   `MULDIV_REM    3'b110  // rem     — 有符号取余                 │
// │   `MULDIV_REMU   3'b111  // remu    — 无符号取余                 │
// │                                                                  │
// │   M 扩展 ALU 操作码 (传递到 mul_div_unit 的 op 编码):              │
// │   `ALU_MUL    4'b1010   `ALU_MULH   4'b1011                       │
// │   `ALU_MULHSU 4'b1100   `ALU_MULHU  4'b1101                       │
// │   `ALU_DIV    4'b1110   `ALU_DIVU   4'b1111                       │
// │   `ALU_REM    4'b1110   `ALU_REMU   4'b1111                       │
// │   注：DIV 和 REM 共享同一 ALU op，由 funct3 区分。                 │
// │                                                                  │
// │   M 单元多周期状态 (mul_div_unit 内部状态机):                      │
// │   `MD_IDLE  2'b00  // 空闲，等待新的乘除操作                      │
// │   `MD_BUSY  2'b01  // 正在计算（乘法需要 1 周期，除法需要多周期）  │
// │   `MD_DONE  2'b10  // 计算完成，结果有效                          │
// │                                                                  │
// │   设计说明：                                                       │
// │   - 乘法使用单周期组合逻辑或小规模流水线                             │
// │   - 除法使用迭代算法（逐位试商），需要多个时钟周期                   │
// │   - M 单元忙时产生 m_unit_stall，暂停整个流水线直到计算完成         │
// └──────────────────────────────────────────────────────────────────┘
//
// ┌──────────────────────────────────────────────────────────────────┐
// │ 第五组：分支类型 (Branch Types, 3-bit)                              │
// │   定义了 RISC-V 的 6 种条件分支的编码。branch_unit 使用这些宏来     │
// │   判断分支是否应该跳转。                                            │
// │                                                                  │
// │   `BRANCH_EQ   3'b000  // beq  — 等于时跳转                       │
// │   `BRANCH_NE   3'b001  // bne  — 不等时跳转                       │
// │   `BRANCH_LT   3'b100  // blt  — 有符号小于时跳转                 │
// │   `BRANCH_GE   3'b101  // bge  — 有符号大于等于时跳转             │
// │   `BRANCH_LTU  3'b110  // bltu — 无符号小于时跳转                 │
// │   `BRANCH_GEU  3'b111  // bgeu — 无符号大于等于时跳转             │
// │                                                                  │
// │   设计说明：分支类型的编码与 RV32I funct3 字段一致，因此可以直接    │
// │   将指令的 funct3 字段传递给 branch_unit，无需额外译码。            │
// └──────────────────────────────────────────────────────────────────┘
//
// ┌──────────────────────────────────────────────────────────────────┐
// │ 第六组：控制信号常量 (Control Signals, 1-bit)                       │
// │   定义了核心控制信号的使能电平值，提高代码可读性。                   │
// │                                                                  │
// │   `MEM_READ   1'b1   // 内存读使能                                │
// │   `MEM_WRITE  1'b1   // 内存写使能                                │
// │   `REG_WRITE  1'b1   // 寄存器文件写使能                          │
// └──────────────────────────────────────────────────────────────────┘
//
// ┌──────────────────────────────────────────────────────────────────┐
// │ 第七组：AXI4 协议常量 (AXI4 Protocol)                               │
// │   为未来的 AXI 总线扩展预留的常量定义。当前 SoC 使用自定义的        │
// │   system_bus（简化的 MMIO 总线），但模块库中已有 axi_lite_manager   │
// │   和 axi4_interconnect 等 AXI 模块，可以使用这些常量。              │
// │                                                                  │
// │   AXI 响应码: `AXI_RESP_OKAY(2'b00), `AXI_RESP_EXOKAY(2'b01),    │
// │              `AXI_RESP_SLVERR(2'b10), `AXI_RESP_DECERR(2'b11)    │
// │   AXI 突发类型: `AXI_BURST_FIXED(2'b00), `AXI_BURST_INCR(2'b01), │
// │                `AXI_BURST_WRAP(2'b10)                            │
// │   AXI 传输大小: `AXI_SIZE_BYTE(3'b000), `AXI_SIZE_HALF(3'b001),  │
// │                `AXI_SIZE_WORD(3'b010)                            │
// └──────────────────────────────────────────────────────────────────┘
//
// ┌──────────────────────────────────────────────────────────────────┐
// │ 第八组：Cache 参数 (Direct-Mapped Cache Parameters)               │
// │   为 rv32im_core 内部的 I-Cache 和 D-Cache 定义结构参数。         │
// │   当前版本中缓存被旁路（bypass），但这些参数在启用缓存时使用。     │
// │                                                                  │
// │   地址划分（32-bit 地址）:                                         │
// │     [31:11] TAG (21 bits)  — 用于检测缓存命中/缺失               │
// │     [10:5]  INDEX (6 bits) — 选择 64 个缓存行之一                │
// │     [4:2]   WORD (3 bits)  — 选择行内的 8 个 32-bit 字之一       │
// │     [1:0]   BYTE (2 bits)  — 字节偏移（字内对齐）                 │
// │                                                                  │
// │   缓存结构：                                                        │
// │   - 直接映射 (direct-mapped)                                       │
// │   - 64 行 × 8 字/行 × 4 字节/字 = 2KB                             │
// │   - 每行包含：21-bit TAG + 1-bit valid + 8×32-bit data           │
// │                                                                  │
// │   `ICACHE_WORD_BITS  3   // 行内字偏移位宽                        │
// │   `ICACHE_NUM_LINES  64  // 缓存行数                             │
// │   `ICACHE_INDEX_BITS 6   // 索引位宽 = ceil(log2(64))            │
// │   `ICACHE_TAG_BITS   21  // Tag 位宽 = 32-2-3-6                  │
// │   (D-Cache 参数同 I-Cache，但使用独立的宏名以便将来独立调优)      │
// └──────────────────────────────────────────────────────────────────┘
`include "Def.v"

// ═══════════════════════════════════════════════════════════════════════════════
//  模块声明: riscv_soc — Sapphire SoC 顶层模块
// ═══════════════════════════════════════════════════════════════════════════════
module riscv_soc (
    // === 时钟与复位 ===
    // clk:   系统主时钟。所有同步逻辑（CPU、总线、外设）都在此时钟域运行。
    //        当前设计为单一时钟域，无跨时钟域问题。
    // reset: 异步复位（高有效）。连接到所有模块的 reset 端口。
    //        复位后 CPU 从 PC=0x00000000 开始取指（BRAM 中的 firmware 入口）。
    input         clk,
    input         reset,

    // === GPIO 引脚（对外物理接口）===
    // gpio_out[31:0]: GPIO 输出引脚。由 GPIO 控制器的 out_reg 寄存器驱动。
    //                 每个引脚的方向由 dir_reg 控制（0=输入，1=输出）。
    //                 当引脚配置为输出时，gpio_out[n] 的值驱动到外部引脚。
    // gpio_in[31:0]:  GPIO 输入引脚。来自外部世界的信号，经 GPIO 内部
    //                 两级同步器（消除亚稳态）后可由 CPU 读取。
    output [31:0] gpio_out,
    input  [31:0] gpio_in,

    // === UART 引脚（对外物理接口）===
    // uart_tx: UART 发送线。由 uart_full 模块的 TX 状态机驱动，
    //          空闲时为高电平（逻辑 1），起始位为低电平（逻辑 0）。
    // uart_rx: UART 接收线。来自外部设备的串行数据输入，
    //          由 uart_rx 子模块进行采样和反串行化。
    output        uart_tx,
    input         uart_rx
);

  // ╔══════════════════════════════════════════════════════════════════════════╗
  // ║  内部互连信号声明 (Interconnect Wire Declarations)                       ║
  // ║                                                                        ║
  // ║  以下是 SoC 内部模块之间的所有连接信号。信号分为以下层次：              ║
  // ║                                                                        ║
  // ║  第一层：CPU ⇔ 系统总线 接口 (cpu_* signals)                            ║
  // ║    CPU 通过统一的存储器总线（unified bus）与系统总线通信。              ║
  // ║    这是 CPU 与外界的唯一数据通道（冯·诺依曼架构）。                     ║
  // ║                                                                        ║
  // ║  第二层：系统总线 ⇔ BRAM 接口 (mem_* signals)                           ║
  // ║    系统总线将地址 0x0000_0000~0x0FFF_FFFF 的访问路由到 BRAM。           ║
  // ║    BRAM 存放固件代码（指令）和运行时数据。                              ║
  // ║                                                                        ║
  // ║  第三层：系统总线 ⇔ 各外设接口 (timer_*, gpio_*, uart_*, plic_*)       ║
  // ║    每个外设拥有独立的地址/数据/写使能信号。                              ║
  // ║    系统总线通过地址译码将 CPU 请求路由到对应的外设。                    ║
  // ║                                                                        ║
  // ║  第四层：外设中断信号 (timer_irq, uart_irq → plic_irq, plic_irq_id)    ║
  // ║    外设产生的中断请求汇聚到 PLIC 进行优先级仲裁，                      ║
  // ║    PLIC 将最终的中断信号送给 CPU。                                      ║
  // ╚══════════════════════════════════════════════════════════════════════════╝

  // ─────────────────────────────────────────────────────────────────────────
  //  CPU ⇔ 系统总线 接口 (Unified Bus)
  //
  //  cpu_addr[31:0]:  CPU 发出的 32-bit 字节地址。来源：取指时来自 PC，
  //                    加载/存储时来自 ALU 计算的地址。
  //  cpu_wdata[31:0]: CPU 发出的 32-bit 写数据。来源：rs2 寄存器值经
  //                    store_unit 字节对齐后输出。
  //  cpu_rdata[31:0]: 返回给 CPU 的 32-bit 读数据。目的：取指时送到
  //                    指令寄存器，加载时经 load_unit 对齐/扩展后写回寄存器。
  //  cpu_we:          CPU 写使能（高有效）。store 指令时置 1。
  //  cpu_re:          CPU 读使能（高有效）。取指和 load 指令时置 1。
  //  cpu_ready:       总线就绪信号。当前 system_bus 恒为 1（零等待），
  //                   但接口保留此信号以便将来支持慢速外设时的反压。
  // ─────────────────────────────────────────────────────────────────────────
  wire [31:0] cpu_addr, cpu_wdata, cpu_rdata;
  wire        cpu_we, cpu_re, cpu_ready;
  wire [3:0]  cpu_be;                       // ★ 字节写使能掩码（core → BRAM）

  // ─────────────────────────────────────────────────────────────────────────
  //  系统总线 ⇔ BRAM 接口 (Memory Interface)
  //
  //  mem_addr[31:0]:  系统总线输出的地址，直接等于 cpu_addr（广播模式）。
  //  mem_wdata[31:0]: 系统总线输出的写数据，直接等于 cpu_wdata。
  //  mem_we:          门控后的 BRAM 写使能 = cpu_we && sel_mem。
  //                   仅当 CPU 写地址落在 0x0xxx_xxxx 区域时有效。
  //  mem_re:          门控后的 BRAM 读使能 = cpu_re && sel_mem。
  //  mem_rdata[31:0]: BRAM 返回的读数据（组合读，零等待）。
  // ─────────────────────────────────────────────────────────────────────────
  wire [31:0] mem_addr, mem_wdata, mem_rdata;
  wire        mem_we, mem_re;

  // ─────────────────────────────────────────────────────────────────────────
  //  Timer 接口
  //
  //  timer_addr[31:0]:  广播到 Timer 的地址 = cpu_addr。
  //  timer_wdata[31:0]: 广播到 Timer 的写数据 = cpu_wdata。
  //  timer_we:          门控写使能 = cpu_we && sel_timer。
  //  timer_rdata[31:0]: Timer 返回的读数据（组合读）。
  //  timer_irq:         Timer 产生的中断请求信号。
  //                     当 counter == compare 且 enable=1 时置位。
  // ─────────────────────────────────────────────────────────────────────────
  wire [31:0] timer_addr, timer_wdata, timer_rdata;
  wire        timer_we;
  wire        timer_irq;

  // ─────────────────────────────────────────────────────────────────────────
  //  GPIO 接口
  //
  //  gpio_addr[31:0]:  广播到 GPIO 的地址 = cpu_addr。
  //  gpio_wdata[31:0]: 广播到 GPIO 的写数据 = cpu_wdata。
  //  gpio_we:          门控写使能 = cpu_we && sel_gpio。
  //  gpio_rdata[31:0]: GPIO 返回的读数据（组合读）。
  //                    包含 dir_reg、out_reg、in_sync2 三个寄存器。
  // ─────────────────────────────────────────────────────────────────────────
  wire [31:0] gpio_addr, gpio_wdata, gpio_rdata;
  wire        gpio_we;

  // ─────────────────────────────────────────────────────────────────────────
  //  UART 接口
  //
  //  uart_addr[31:0]:  广播到 UART 的地址 = cpu_addr。
  //  uart_wdata[31:0]: 广播到 UART 的写数据 = cpu_wdata。
  //  uart_we:          门控写使能 = cpu_we && sel_uart。
  //  uart_rdata[31:0]: UART 返回的读数据（组合读）。
  //  uart_irq:         UART 产生的中断请求信号。
  //                    = irq_enable & (tx_empty | rx_ready)
  // ─────────────────────────────────────────────────────────────────────────
  wire [31:0] uart_addr, uart_wdata, uart_rdata;
  wire        uart_we;
  wire        uart_irq;

  // ─────────────────────────────────────────────────────────────────────────
  //  PLIC 接口
  //
  //  plic_addr[31:0]:  广播到 PLIC 的地址 = cpu_addr。
  //  plic_wdata[31:0]: 广播到 PLIC 的写数据 = cpu_wdata。
  //  plic_we:          门控写使能 = cpu_we && sel_plic。
  //  plic_rdata[31:0]: PLIC 返回的读数据（组合读）。
  //  plic_irq:         PLIC 向 CPU 发出的中断请求信号。
  //                    当存在有效且使能的中断源时置 1。
  //                    连接到 rv32im_core 的 irq 端口。
  //  plic_irq_id[4:0]: PLIC 向 CPU 发出的中断 ID（5-bit，最多 32 个中断源）。
  //                    标识当前优先级最高的中断源的编号。
  //                    连接到 rv32im_core 的 irq_id 端口。
  // ─────────────────────────────────────────────────────────────────────────
  wire [31:0] plic_addr, plic_wdata, plic_rdata;
  wire        plic_we;
  wire        plic_irq;
  wire [4:0]  plic_irq_id;


  // ╔══════════════════════════════════════════════════════════════════════════╗
  // ║                                                                        ║
  // ║  实例化 1: rv32im_core — RV32IM 五级流水线处理器核心                     ║
  // ║                                                                        ║
  // ║  【架构概述】                                                             ║
  // ║  rv32im_core 是一个经典的 5 级流水线 RISC-V 处理器，支持 RV32I 基础      ║
  // ║  整数指令集 + RV32M 整数乘除法扩展。流水线五级划分如下：                  ║
  // ║                                                                        ║
  // ║  ┌──────────┬─────────────────────────────────────────────────────┐    ║
  // ║  │ IF 取指   │ 从指令存储器读取指令字。PC 由 branch_prediction_unit │    ║
  // ║  │ (第1级)   │ 预测下一条指令地址。支持顺序 PC+4、分支目标、JALR    │    ║
  // ║  │           │ 目标三种地址源。pipeline_stall 可冻结 IF 阶段。       │    ║
  // ║  ├──────────┼─────────────────────────────────────────────────────┤    ║
  // ║  │ ID 译码   │ 指令译码：Decoder 解析 opcode/funct3/funct7；       │    ║
  // ║  │ (第2级)   │ Control Unit 生成 ALU op、写回选择、寄存器写使能等   │    ║
  // ║  │           │ 控制信号；RegFile 读出 rs1 和 rs2 的值；imm_gen      │    ║
  // ║  │           │ 从指令字提取并符号扩展立即数。                        │    ║
  // ║  │           │ ★ 关键结构：                                          │    ║
  // ║  │           │   - Hazard Detection: 检测 RAW 数据冒险               │    ║
  // ║  │           │   - Bypass Unit: 生成转发选择信号 bypass_irmux1/2    │    ║
  // ║  │           │   - Branch Prediction Unit: 静态分支方向预测         │    ║
  // ║  ├──────────┼─────────────────────────────────────────────────────┤    ║
  // ║  │ EX 执行   │ 核心计算阶段：                                        │    ║
  // ║  │ (第3级)   │   1) Forwarding MUX: 根据 ID/EX 流水线中锁存的      │    ║
  // ║  │           │      bypass_irmux1/2 选择 ALU 操作数来源             │    ║
  // ║  │           │      (RF 输出 / MEM 阶段 ALU 结果 / WB 阶段写回数据) │    ║
  // ║  │           │   2) ALU: 执行 RV32I 的 10 种算术/逻辑/移位/比较    │    ║
  // ║  │           │   3) M-Unit (mul_div_unit): 执行乘除法，多周期时     │    ║
  // ║  │           │      产生 m_unit_stall 暂停流水线                     │    ║
  // ║  │           │   4) Branch Unit: 解析分支/jump 指令的跳转方向       │    ║
  // ║  │           │   5) Wrong Predict Detection: 比较预测与实际方向     │    ║
  // ║  ├──────────┼─────────────────────────────────────────────────────┤    ║
  // ║  │ MEM 访存  │ 数据存储器访问阶段：                                   │    ║
  // ║  │ (第4级)   │   1) Load Unit: 根据 funct3 和地址低 2 位进行        │    ║
  // ║  │           │      字节/半字/字 对齐和符号/零扩展                   │    ║
  // ║  │           │   2) Store Unit: 根据 funct3 和地址低 2 位生成       │    ║
  // ║  │           │      字节使能掩码和字节对齐的写数据                    │    ║
  // ║  │           │   3) 通过 unified bus 访问 system_bus               │    ║
  // ║  ├──────────┼─────────────────────────────────────────────────────┤    ║
  // ║  │ WB 写回   │ 写回阶段（纯组合逻辑 MUX）：                           │    ║
  // ║  │ (第5级)   │   wb_wbmux 选择写回数据来源：                         │    ║
  // ║  │           │     - 2'b00: ALU 结果 (R-type / I-type ALU)         │    ║
  // ║  │           │     - 2'b01: Load 数据 (lw/lh/lb/lbu/lhu)          │    ║
  // ║  │           │     - 2'b10: PC+4 (JAL/JALR 返回地址)              │    ║
  // ║  │           │   写回到寄存器文件 (RegFile) 中 rd 指定的寄存器。     │    ║
  // ║  └──────────┴─────────────────────────────────────────────────────┘    ║
  // ║                                                                        ║
  // ║  【流水线寄存器 (Pipeline Registers)】                                    ║
  // ║  四级流水线寄存器划分：                                                  ║
  // ║    - IF/ID  (if_id_stage):    PC、指令字                               ║
  // ║    - ID/EX  (id_ex_stage):    PC、rs1_data、rs2_data、imm、rd_addr、   ║
  // ║                               aluop、mem_write、mem_read、reg_write、  ║
  // ║                               funct3、opcode、wbmux、bypass_irmux1/2   ║
  // ║    - EX/MEM (ex_mem_stage):   alu_result、write_data、rd_addr、        ║
  // ║                               mem_write、mem_read、reg_write、wbmux    ║
  // ║    - MEM/WB (mem_wb_stage):   alu_result、load_data、pc_plus_4、       ║
  // ║                               rd_addr、reg_write、wbmux                ║
  // ║  每级流水线寄存器都受 stall（冻结）和 flush（清零）控制。               ║
  // ║                                                                        ║
  // ║  【Forwarding (数据前递) 架构】                                           ║
  // ║  为避免 RAW (Read-After-Write) 数据冒险导致的流水线停顿，核心采用了      ║
  // ║  完整的数据前递（forwarding / bypass）机制：                            ║
  // ║                                                                        ║
  // ║  前递路径（在 EX 阶段入口处通过 MUX 选择 ALU 操作数）：                  ║
  // ║    ex_fwd1 = (ex_fwd1 == 2'b01) → mem_alu_result   // 上一条指令的     ║
  // ║             │                                          ALU 结果         ║
  // ║             │ (ex_fwd1 == 2'b10) → wb_write_data     // 上上条指令的     ║
  // ║             │                                          写回数据         ║
  // ║             │ (default)         → ex_rs1_data         // 寄存器文件      ║
  // ║             │                                          读出的值         ║
  // ║                                                                        ║
  // ║  前递选择信号 (ex_fwd1/ex_fwd2) 的生成：                                ║
  // ║    bypass_unit 在 ID 阶段根据以下信息生成 bypass_irmux1/2：             ║
  // ║     - 当前指令的 rs1/rs2 地址                                           ║
  // ║     - EX 阶段的 rd 地址 + reg_write 使能                               ║
  // ║     - MEM 阶段的 rd 地址 + reg_write 使能                              ║
  // ║    比较 rs1/rs2 是否匹配流水线后级的 rd：                                ║
  // ║     - 匹配 EX.rd  → fwd_sel = 2'b01 (前递 MEM 结果)                   ║
  // ║     - 匹配 MEM.rd → fwd_sel = 2'b10 (前递 WB 结果)                    ║
  // ║     - 不匹配      → fwd_sel = 2'b00 (使用 RF 读出值)                   ║
  // ║  这些选择信号通过 ID/EX 流水线寄存器锁存，在 EX 阶段使用。             ║
  // ║                                                                        ║
  // ║  【Stall (流水线停顿) 机制】                                              ║
  // ║  pipeline_stall 由 stall_unit 统一管理，当以下任一条件满足时拉高：      ║
  // ║    1. load_use_stall (bypass_stall): 加载指令后紧跟使用加载结果的      ║
  // ║       指令。即使有 forwarding，加载数据在 MEM 阶段才返回，               ║
  // ║       EX 阶段无法前递，必须停顿 1 个周期。                               ║
  // ║    2. multi_cycle_stall (m_unit_stall): M 扩展的乘除单元正在计算      ║
  // ║       （特别是除法需要多个周期），必须等待。                              ║
  // ║    3. imem_ready / dmem_ready: 指令/数据存储器未就绪（当前设计中        ║
  // ║       总线零等待，这两个信号通常为 1，但保留了反压能力）。               ║
  // ║  stall=1 时：PC 不更新，IF/ID 流水线寄存器冻结，ID 阶段控制信号清零。  ║
  // ║                                                                        ║
  // ║  【M-Unit 集成方式】                                                      ║
  // ║  mul_div_unit 在 EX 阶段与 ALU 并行工作：                               ║
  // ║    - 检测：ex_is_m_op = ex_aluop_orig[3] && ex_aluop_orig[2]          ║
  // ║      (即 aluop 的高 2 位为 10，对应 `ALU_MUL ~ `ALU_REMU)             ║
  // ║    - 启动：m_unit_start = ex_is_m_op && !pipeline_stall               ║
  // ║    - 旁路 ALU：当 ex_is_m_op=1 时，ALU 被强制为 ADD（pass-through），  ║
  // ║      实际的乘除结果由 m_unit_result 提供。                              ║
  // ║    - 多周期：乘法在 1 个周期内完成，除法需要多周期（取决于操作数位宽）。 ║
  // ║      除法期间 m_unit_stall=1，暂停流水线直到完成。                      ║
  // ║    - 结果选择：ex_alu_result = ex_is_m_op ? m_unit_result             ║
  // ║                                              : alu_raw_result          ║
  // ║                                                                        ║
  // ║  【分支预测与误预测恢复】                                                  ║
  // ║    - 分支预测单元 (branch_prediction_unit) 在 ID 阶段根据 opcode 和     ║
  // ║      历史信息预测分支方向 (branch_taken)。                                ║
  // ║    - 预测结果通过 1 级延迟寄存器 (branch_taken_d) 与 EX 阶段实际        ║
  // ║      分支方向 (branch_taken_ex | jalr_enable_ex) 进行 XOR 比较。       ║
  // ║    - 如不匹配 (wrong_predict=1)：触发 flush，清除 IF/ID 流水线中的      ║
  // ║      错误指令，PC 更新为正确目标地址。                                    ║
  // ║                                                                        ║
  // ╚══════════════════════════════════════════════════════════════════════════╝
  rv32im_core core (
    .clk      (clk),
    .reset    (reset),

    // Unified Bus（cache bypass 模式——CPU 通过单一总线直接访问存储器）
    // 当前设计旁路了核内的 I-Cache 和 D-Cache，指令和数据都经过此总线。
    .mem_addr  (cpu_addr),     // CPU 发出的字节地址
    .mem_wdata (cpu_wdata),    // CPU 发出的写数据
    .mem_we    (cpu_we),       // 写使能
    .mem_re    (cpu_re),       // 读使能
    .mem_be    (cpu_be),       // ★ 字节写使能掩码（来自 store_unit → 连接到 BRAM）
    .mem_rdata (cpu_rdata),    // 返回给 CPU 的读数据
    .mem_ready (cpu_ready),    // 总线就绪（当前恒为 1）

    // 中断接口（来自 PLIC）
    // PLIC 仲裁后的中断请求和中断 ID，CPU 在 trap 时读取 irq_id 确定中断源。
    .irq       (plic_irq),     // 中断请求信号（高有效）
    .irq_id    (plic_irq_id)   // 中断源 ID（0~31，0 表示无中断）
  );

  // ╔══════════════════════════════════════════════════════════════════════════╗
  // ║                                                                        ║
  // ║  实例化 2: system_bus — 系统总线（地址译码 + 数据多路复用）              ║
  // ║                                                                        ║
  // ║  【功能】                                                                ║
  // ║  system_bus 是 SoC 的中央互连单元，角色相当于传统 SoC 中的总线矩阵      ║
  // ║  (bus matrix) 或 AHB/APB 总线桥。它完成两项核心功能：                   ║
  // ║    1. 地址译码：根据 cpu_addr 的高位判断目标外设，选中对应的            ║
  // ║       sel_xxx 信号。                                                    ║
  // ║    2. 数据多路复用：将被选中外设的读数据 (xxx_rdata) 选择后返回 CPU。   ║
  // ║                                                                        ║
  // ║  【端口连接逻辑】                                                        ║
  // ║   - CPU 侧：cpu_* 连接到 rv32im_core 的 unified bus 端口。             ║
  // ║   - BRAM 侧：mem_* 连接到 bram 模块。mem_we/mem_re 经过 sel_mem       ║
  // ║     门控，确保只在地址命中 BRAM 区域时 BRAM 才响应。                   ║
  // ║   - 外设侧：timer_*、gpio_*、uart_*、plic_* 连接到对应外设。           ║
  // ║     地址和数据广播到所有外设，写使能通过 AND 门控 (cpu_we && sel_xxx)  ║
  // ║     确保只有被选中的外设才执行写操作。                                  ║
  // ║                                                                        ║
  // ║  【mem_ready 连接说明】                                                  ║
  // ║  mem_ready 端口被硬连接到 1'b1。这是因为：                              ║
  // ║    1. BRAM 使用组合读，无需等待。                                       ║
  // ║    2. 当前设计中 BRAM 是 memeory 侧的唯一从设备。                       ║
  // ║    3. 如果将来将外设也接到 mem_* 通道，需要根据选中的从设备来           ║
  // ║       动态生成 mem_ready。                                              ║
  // ╚══════════════════════════════════════════════════════════════════════════╝
  system_bus bus (
    .clk        (clk),
    .reset      (reset),

    // CPU 侧接口（单主设备：rv32im_core）
    .cpu_addr   (cpu_addr),
    .cpu_wdata  (cpu_wdata),
    .cpu_we     (cpu_we),
    .cpu_re     (cpu_re),
    .cpu_rdata  (cpu_rdata),
    .cpu_ready  (cpu_ready),

    // 存储器侧接口（BRAM）
    .mem_addr   (mem_addr),
    .mem_wdata  (mem_wdata),
    .mem_we     (mem_we),
    .mem_re     (mem_re),
    .mem_rdata  (mem_rdata),
    .mem_ready  (1'b1),    // BRAM 始终就绪（组合读，零等待）

    // 外设接口（地址广播 + 写使能门控）
    .timer_addr (timer_addr),
    .timer_wdata(timer_wdata),
    .timer_we   (timer_we),
    .timer_rdata(timer_rdata),

    .gpio_addr  (gpio_addr),
    .gpio_wdata (gpio_wdata),
    .gpio_we    (gpio_we),
    .gpio_rdata (gpio_rdata),

    .uart_addr  (uart_addr),
    .uart_wdata (uart_wdata),
    .uart_we    (uart_we),
    .uart_rdata (uart_rdata),

    .plic_addr  (plic_addr),
    .plic_wdata (plic_wdata),
    .plic_we    (plic_we),
    .plic_rdata (plic_rdata)
  );

  // ╔══════════════════════════════════════════════════════════════════════════╗
  // ║                                                                        ║
  // ║  实例化 3: bram — 块 RAM (64KB，指令 + 数据统一存储)                    ║
  // ║                                                                        ║
  // ║  【功能】                                                                ║
  // ║  BRAM 是 SoC 的主存储器，同时存储 CPU 的指令（firmware）和运行时数据。  ║
  // ║  容量 64KB (16384 × 32-bit)，通过 firmware.hex 文件在仿真时初始化。    ║
  // ║                                                                        ║
  // ║  【读写特性】                                                            ║
  // ║   - 组合读：rdata 由 mem[word_addr] 组合输出，零等待。                  ║
  // ║   - 同步写：we=1 时在 clk 上升沿写入，支持字节使能 (be)。              ║
  // ║   - 字节使能 (cpu_be)：★ 已修复，core 的 store_mask → cpu_be → BRAM.be ║
  // ║     SB/SH/SW 指令的字节写掩码正确传递，支持逐字节写入。                 ║
  // ║                                                                        ║
  // ║  【复位行为】                                                            ║
  // ║  复位不清零 BRAM 内容（保留 firmware），CPU 从地址 0x00000000 取指。   ║
  // ║                                                                        ║
  // ║  【be 信号连接说明】★【已修复】                                         ║
  // ║  原硬编码为 4'b1111，现已改为 cpu_be 信号：                              ║
  // ║    core (store_unit.store_mask) → cpu_be → BRAM.be                      ║
  // ║  SB/SH/SW 指令的字节写掩码正确传递到 BRAM，支持逐字节写入。             ║
  // ║  store_unit 根据 funct3 和 addr_low 动态生成掩码：                      ║
  // ║    SB: 掩码 = 4'b0001 << addr[1:0]（仅目标字节使能）                   ║
  // ║    SH: 掩码 = addr[1] ? 4'b1100 : 4'b0011（目标半字使能）              ║
  // ║    SW: 掩码 = 4'b1111（全字使能）                                      ║
  // ╚══════════════════════════════════════════════════════════════════════════╝
  bram memory (
    .clk   (clk),
    .reset (reset),
    .addr  (mem_addr),      // 字节地址（来自 system_bus 的 mem_addr）
    .wdata (mem_wdata),     // 写数据（来自 system_bus 的 mem_wdata）
    .we    (mem_we),        // 写使能（仅当地址命中 BRAM 区域时有效）
    .be    (cpu_be),         // ★ 字节使能：来自 core 的 store_mask（SB/SH/SW 正确实现）
    .rdata (mem_rdata)      // 读数据（组合输出，返回给 system_bus）
  );

  // ╔══════════════════════════════════════════════════════════════════════════╗
  // ║                                                                        ║
  // ║  实例化 4: timer — 可编程定时器                                          ║
  // ║                                                                        ║
  // ║  【功能】                                                                ║
  // ║  提供可编程的硬件定时器，用于产生周期性中断或测量时间间隔。              ║
  // ║                                                                        ║
  // ║  【寄存器】                                                              ║
  // ║    - 0x00 COUNT (只读):  当前计数值（enable=1 时每个时钟周期递增）     ║
  // ║    - 0x04 COMPARE (读/写): 比较值，写操作同时清除中断                   ║
  // ║    - 0x08 CTRL (读/写):   控制寄存器，bit0=enable（启动/停止计数）     ║
  // ║                                                                        ║
  // ║  【中断行为】                                                            ║
  // ║    - 当 counter == compare 且 enable=1 时，timer_irq 置 1。            ║
  // ║    - 中断锁存（irq_latch）：一旦触发后保持为 1，直到软件写 COMPARE     ║
  // ║      寄存器时清除。这确保了软件有足够时间响应中断。                      ║
  // ║    - timer_irq 连接到 PLIC 的 irq_sources[0]（最低优先级中断源）。     ║
  // ║                                                                        ║
  // ║  【设计说明】                                                            ║
  // ║  写 COMPARE 寄存器时同时清除中断锁存——这是一个简洁的软件-硬件协议：     ║
  // ║  中断服务程序在处理完定时事件后，写入新的 COMPARE 值来设置下一次定时    ║
  // ║  并同时清除当前中断。                                                    ║
  // ╚══════════════════════════════════════════════════════════════════════════╝
  timer timer0 (
    .clk       (clk),
    .reset     (reset),
    .addr      (timer_addr),     // 地址（广播自 system_bus）
    .wdata     (timer_wdata),    // 写数据（广播自 system_bus）
    .we        (timer_we),       // 写使能（仅当地址命中 Timer 区域时有效）
    .rdata     (timer_rdata),    // 读数据（返回给 system_bus）
    .timer_irq (timer_irq)       // 中断请求（连接到 PLIC）
  );

  // ╔══════════════════════════════════════════════════════════════════════════╗
  // ║                                                                        ║
  // ║  实例化 5: gpio — 通用输入输出控制器                                     ║
  // ║                                                                        ║
  // ║  【功能】                                                                ║
  // ║  提供 32 个可独立配置方向的 GPIO 引脚。每个引脚可配置为输入或输出。      ║
  // ║                                                                        ║
  // ║  【寄存器】                                                              ║
  // ║    - 0x00 DIR (读/写): 方向寄存器，bit[n]=1 表示引脚 n 为输出，         ║
  // ║                        0 表示输入。复位值全 0（全部输入）。              ║
  // ║    - 0x04 OUT (读/写): 输出数据寄存器，bit[n] 的值驱动到 gpio_out[n]。  ║
  // ║                        仅在 DIR[n]=1 时对外部有实际影响。               ║
  // ║    - 0x08 IN  (只读):  输入数据寄存器，反映 gpio_in[n] 经两级同步器    ║
  // ║                        后的值。                                          ║
  // ║                                                                        ║
  // ║  【两级同步器】                                                          ║
  // ║  gpio_in 是异步输入信号（来自外部世界），直接采样会产生亚稳态。          ║
  // ║  GPIO 模块内部使用两级触发器链（in_sync1 → in_sync2）消除亚稳态风险。  ║
  // ║  第一级可能输出亚稳态，第二级以极高的概率输出稳定的 0 或 1。            ║
  // ║  代价是引入 2 个时钟周期的输入延迟。                                     ║
  // ║                                                                        ║
  // ║  【GPIO 中断】                                                           ║
  // ║  当前 GPIO 版本不支持中断（没有 irq 输出端口）。如果需要 GPIO 边沿/电平  ║
  // ║  触发中断，需要在 GPIO 模块中增加中断检测逻辑和中断输出信号。            ║
  // ╚══════════════════════════════════════════════════════════════════════════╝
  gpio gpio0 (
    .clk      (clk),
    .reset    (reset),
    .addr     (gpio_addr),      // 地址（广播自 system_bus）
    .wdata    (gpio_wdata),     // 写数据（广播自 system_bus）
    .we       (gpio_we),        // 写使能（仅当地址命中 GPIO 区域时有效）
    .rdata    (gpio_rdata),     // 读数据（返回给 system_bus）
    .gpio_out (gpio_out),       // GPIO 输出引脚（连接到芯片顶层端口）
    .gpio_in  (gpio_in)         // GPIO 输入引脚（连接到芯片顶层端口）
  );

  // ╔══════════════════════════════════════════════════════════════════════════╗
  // ║                                                                        ║
  // ║  实例化 6: uart_full — 全功能 UART（通用异步收发器）                     ║
  // ║                                                                        ║
  // ║  【功能】                                                                ║
  // ║  提供标准异步串行通信接口，用于与 PC（终端/调试器）或其他串行设备通信。  ║
  // ║  是 SoC 调试和运行时交互的主要通道。                                     ║
  // ║                                                                        ║
  // ║  【模块组成】                                                            ║
  // ║  uart_full 集成了 TX（发送）和 RX（接收）两部分：                        ║
  // ║    - uart_rx 子模块：异步串行接收，支持可配置波特率。                    ║
  // ║      输出 rx_ready（接收完毕）和 rx_data_buf（接收的 8-bit 数据）。     ║
  // ║    - TX 状态机：异步串行发送，11 个状态（idle/start/8×data/stop/idle）  ║
  // ║      支持可配置波特率。                                                  ║
  // ║                                                                        ║
  // ║  【寄存器】                                                              ║
  // ║    - 0x00 DATA (读/写):  写 = 发送数据缓冲 (tx_data[7:0])，             ║
  // ║                           读 = 最后接收的字节 (rx_data_buf[7:0])        ║
  // ║    - 0x04 CTRL (读/写):  控制寄存器 [17]rx_enable [16]tx_enable        ║
  // ║                           [15:0]baud_div（波特率分频系数）              ║
  // ║    - 0x08 STAT (只读):   状态寄存器 [31]irq_enable [1]rx_ready         ║
  // ║                           [0]tx_empty（发送缓冲区空闲）                  ║
  // ║                                                                        ║
  // ║  【波特率计算】                                                          ║
  // ║  baud_div = clk_freq / baud_rate                                         ║
  // ║  例：50MHz clk / 115200 baud = 434（默认值）                             ║
  // ║  复位后默认 115200 波特率，tx_enable=1, rx_enable=1。                   ║
  // ║                                                                        ║
  // ║  【中断】                                                                ║
  // ║  uart_irq = irq_enable & (tx_empty | rx_ready)                          ║
  // ║  当 irq_enable=1 时，发送完成（tx_empty=1）或接收到数据（rx_ready=1）   ║
  // ║  均会触发中断。uart_irq 连接到 PLIC 的 irq_sources[1]。                 ║
  // ╚══════════════════════════════════════════════════════════════════════════╝
  uart_full uart0 (
    .clk      (clk),
    .reset    (reset),
    .addr     (uart_addr),      // 地址（广播自 system_bus）
    .wdata    (uart_wdata),     // 写数据（广播自 system_bus）
    .we       (uart_we),        // 写使能（仅当地址命中 UART 区域时有效）
    .rdata    (uart_rdata),     // 读数据（返回给 system_bus）
    .tx       (uart_tx),        // UART 发送引脚（连接到芯片顶层端口）
    .rx       (uart_rx),        // UART 接收引脚（连接到芯片顶层端口）
    .uart_irq (uart_irq)        // 中断请求（连接到 PLIC）
  );

  // ╔══════════════════════════════════════════════════════════════════════════╗
  // ║                                                                        ║
  // ║  实例化 7: plic — 平台级中断控制器 (Platform-Level Interrupt Controller)║
  // ║                                                                        ║
  // ║  【功能】                                                                ║
  // ║  PLIC 是 RISC-V 平台标准的中断管理单元，负责：                           ║
  // ║    1. 收集所有外设的中断请求（本例中: timer_irq + uart_irq）            ║
  // ║    2. 根据各中断源的优先级进行仲裁                                      ║
  // ║    3. 输出最高优先级中断的请求信号 (plic_irq) 和 ID (plic_irq_id)       ║
  // ║    4. 提供 Claim/Complete 机制，支持中断的查询和完成                    ║
  // ║                                                                        ║
  // ║  【中断源连接】                                                          ║
  // ║  irq_sources = {29'b0, uart_irq, timer_irq}                             ║
  // ║     - irq_sources[0] = timer_irq  (Timer 中断，低优先级)                ║
  // ║     - irq_sources[1] = uart_irq   (UART 中断，次低优先级)               ║
  // ║     - irq_sources[31:2] = 0       (保留，未使用)                        ║
  // ║  注意：RISC-V PLIC 规范中，中断源 0 保留（表示"无中断"），               ║
  // ║  但本实现中 irq_sources[0] 连接了 Timer。这是因为硬件连线简单，         ║
  // ║  软件通过优先级寄存器区分中断源。                                        ║
  // ║                                                                        ║
  // ║  【寄存器映射】                                                          ║
  // ║    - 0x0000 PENDING (只读):    中断等待寄存器，bit[n]=1 表示中断源 n   ║
  // ║                                 有待处理的中断                           ║
  // ║    - 0x0004 ENABLE (读/写):    中断使能寄存器，bit[n]=1 允许中断源 n   ║
  // ║    - 0x0020 THRESHOLD (读/写): 优先级阈值，仅优先级 > threshold 的     ║
  // ║                                 中断才会被上报                           ║
  // ║    - 0x0040~0x007C PRIORITY[0..31] (读/写): 各中断源的优先级(8-bit)   ║
  // ║    - 0x0080 CLAIM/COMPLETE:    读 = Claim (获取最高优先级中断 ID      ║
  // ║                                 并清除 pending)，写 = Complete         ║
  // ║                                 (表示中断处理完成)                      ║
  // ║                                                                        ║
  // ║  【中断仲裁流程】                                                        ║
  // ║    1. 外设产生中断 → irq_sources[n] 出现上升沿 → pending[n] = 1        ║
  // ║    2. 优先级计算：effective_prio[n] = (prio[n] > threshold)            ║
  // ║                                        && pending[n] && enable[n]      ║
  // ║    3. 优先级编码器选出 effective_prio 中最高的中断号 → plic_irq_id     ║
  // ║    4. 如有有效中断：plic_irq = 1，CPU 进入 trap handler                ║
  // ║    5. CPU 读 CLAIM (0x80) → 返回 plic_irq_id，清除 pending[irq_id]    ║
  // ║    6. CPU 执行中断服务程序                                              ║
  // ║    7. CPU 写 COMPLETE (0x80) → 通知 PLIC 中断处理完成                  ║
  // ║                                                                        ║
  // ║  【优先级策略】                                                          ║
  // ║  高编号中断源具有更高优先级（highest_irq 的 for 循环从 31 递减到 0）。  ║
  // ║  这与 RISC-V PLIC 规范一致：中断源 ID 越大，默认优先级越高。            ║
  // ║  但实际优先级由 PRIORITY 寄存器决定，软件可编程调整。                    ║
  // ║                                                                        ║
  // ║  【上升沿检测】                                                          ║
  // ║  irq_rising = irq_sources & ~irq_sources_d（前次值）                    ║
  // ║  pending 锁存上升沿触发的 pending 位，确保即使外设中断信号撤消，         ║
  // ║  PLIC 仍保持中断请求，直到软件显式 Claim。                               ║
  // ╚══════════════════════════════════════════════════════════════════════════╝
  plic plic0 (
    .clk         (clk),
    .reset       (reset),
    .addr        (plic_addr),     // 地址（广播自 system_bus）
    .wdata       (plic_wdata),    // 写数据（广播自 system_bus）
    .we          (plic_we),       // 写使能（仅当地址命中 PLIC 区域时有效）
    .rdata       (plic_rdata),    // 读数据（返回给 system_bus）

    // 中断源输入：Timer 和 UART 的中断请求信号
    // {29'b0, uart_irq, timer_irq} 中：
    //   bit[1] = uart_irq  — UART 中断
    //   bit[0] = timer_irq — Timer 中断
    // 高位全部接地（未使用）
    .irq_sources ({29'b0, uart_irq, timer_irq}),

    // 中断输出到 CPU
    .plic_irq    (plic_irq),      // 中断请求信号（连接到 rv32im_core 的 irq 端口）
    .plic_irq_id (plic_irq_id)    // 中断源 ID（连接到 rv32im_core 的 irq_id 端口）
  );

endmodule

// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  文件结束: TOP_levelSOc.v                                                   ║
// ║                                                                            ║
// ║  【扩展方向（TODO / Future Work）】                                          ║
// ║                                                                            ║
// ║  1. 启用 I-Cache 和 D-Cache:                                                ║
// ║     将 rv32im_core 内部的 cache bypass 切换为启用缓存模式，                ║
// ║     增加独立的 I$ 和 D$ 主端口，使用 bus_arbiter 进行仲裁。                 ║
// ║                                                                            ║
// ║  2. 增加字节使能支持:                                                        ║
// ║     将 store_unit 的 store_mask 信号通过系统总线连接到 BRAM 的 be 端口，    ║
// ║     以正确支持 SB（存字节）和 SH（存半字）指令。                             ║
// ║                                                                            ║
// ║  3. 升级总线协议:                                                             ║
// ║     将 system_bus 升级为 AXI4-Lite 或 AXI4 总线，支持多主多从、             ║
// ║     burst 传输、流水线读写，提高系统带宽和可扩展性。                         ║
// ║                                                                            ║
// ║  4. 增加 SPI / I2C 外设:                                                      ║
// ║     通过 system_bus 连接更多外设，扩展 SoC 的 I/O 能力。                    ║
// ║     需要注意慢速外设的 wait state 插入（cpu_ready 动态控制）。               ║
// ║                                                                            ║
// ║  5. 增加 JTAG 调试接口:                                                       ║
// ║     实现 RISC-V Debug Spec (0.13)，支持通过 JTAG 进行硬件断点、             ║
// ║     单步执行、寄存器/存储器访问等调试功能。                                  ║
// ║                                                                            ║
// ║  6. 增加功耗管理:                                                             ║
// ║     增加时钟门控 (clock gating) 和电源域划分，支持 WFI (Wait For Interrupt) ║
// ║     指令的低功耗模式。                                                       ║
// ║                                                                            ║
// ╚══════════════════════════════════════════════════════════════════════════════╝
