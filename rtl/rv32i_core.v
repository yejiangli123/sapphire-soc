// ============================================================
//  rv32i_core.v — RV32IM 5级流水线核心处理器
// ============================================================
//
// 【模块定位与架构角色】
//   本模块是 Sapphire SoC 的中央处理器核心，实现 RV32IM 指令集：
//     - RV32I：整数基本指令集（计算、分支、访存、系统）
//     - RV32M：整数乘除法扩展（MUL/MULH/MULHSU/MULHU/DIV/DIVU/REM/REMU）
//   采用经典 5 级流水线架构：IF → ID → EX → MEM → WB
//   内部集成：
//     - 前递（Bypass/Forwarding）单元：解决数据冒险，减少流水线停顿
//     - 冒险检测单元（Hazard Detection）：处理 Load-Use 冒险与分支/JALR 冒险
//     - 分支预测单元（Branch Prediction）：ID 阶段预测分支方向
//     - 多周期乘除法单元（M-Unit）：支持 RV32M 乘除法，带流水线停顿
//     - 加载/存储单元（Load/Store Unit）：处理非对齐访问、字节/半字/字读写
//   外部接口为统一存储器总线（Unified Memory Bus），在 SoC 顶层通过 system_bus
//   连接到 BRAM 与外设。
//
// 【设计决策摘要】
//   1. 5级流水线：经典的 IF/ID/EX/MEM/WB 划分，每级之间由流水线寄存器模块隔离
//   2. 前递架构：ID 阶段计算前递选择信号（bypass_irmux1/2），锁存到 ID/EX 流水线
//      寄存器中（ex_fwd1/ex_fwd2），EX 阶段根据锁存的信号选择操作数来源
//   3. 停顿机制：三级停顿源 — pipeline_stall（总停顿）、bypass_stall（Load-Use）、
//      m_unit_stall（多周期乘除法）、icache_ready（I-Cache 未就绪）
//   4. M-Unit 集成：EX 阶段检测 M-extension 操作码（ex_aluop_orig[3] && [2]），
//      启动多周期乘除法单元，busy 期间拉高 m_unit_stall 冻结流水线
//   5. 分支处理：ID 阶段预测方向，EX 阶段确认结果；若预测错误则冲刷 IF 级
//      （wrong_predict 信号），并在 ID/EX 寄存器通过 hazard_ctrl_mux_sel 插入 NOP
//   6. 无缓存直连：I-Cache 和 D-Cache 旁路（bypass），指令和数据直接通过总线访问
//      外部 BRAM，简化调试和初始验证
//   7. funct3 追踪：两级移位寄存器（id_ex_funct3 → ex_mem_funct3）将 funct3 从
//      ID 阶段传递到 MEM 阶段，供 load_unit 和 store_unit 使用
//
// 【包含文件 Def.v 宏定义分组说明】
//   Def.v 定义了整个处理器使用的所有宏，按功能分组如下：
//
//   ■ 指令操作码 (opcode[6:0]) — 用于 decoder/control_unit 识别指令类型
//     OPCODE_LOAD, OPCODE_MISC_MEM, OPCODE_OP_IMM, OPCODE_AUIPC, OPCODE_STORE,
//     OPCODE_OP, OPCODE_LUI, OPCODE_BRANCH, OPCODE_JALR, OPCODE_JAL, OPCODE_SYSTEM
//
//   ■ 立即数类型 (op_format) — 用于 imm_gen 生成 32 位立即数
//     I_type, I_type_load, S_type, B_type, J_type, U_type_LUI, U_type_AUIPC
//
//   ■ ALU 操作码 (aluop[3:0]) — 控制 ALU 执行何种运算
//     ALU_ADD, ALU_SUB, ALU_XOR, ALU_OR, ALU_AND, ALU_SLL, ALU_SRL, ALU_SRA,
//     ALU_SLT, ALU_SLTU
//
//   ■ M 扩展 funct3 编码 — RV32M 乘法/除法操作子类型
//     MULDIV_MUL, MULDIV_MULH, MULDIV_MULHSU, MULDIV_MULHU,
//     MULDIV_DIV, MULDIV_DIVU, MULDIV_REM, MULDIV_REMU
//
//   ■ M 扩展 ALU 操作码 (aluop[3:0]) — 高两位=10 标识 M 操作
//     ALU_MUL, ALU_MULH, ALU_MULHSU, ALU_MULHU, ALU_DIV, ALU_DIVU,
//     ALU_REM, ALU_REMU
//
//   ■ M 单元状态机状态 (mul_div_unit 内部使用)
//     MD_IDLE, MD_BUSY, MD_DONE
//
//   ■ 分支类型编码 (funct3) — branch_unit 判断分支条件
//     BRANCH_EQ, BRANCH_NE, BRANCH_LT, BRANCH_GE, BRANCH_LTU, BRANCH_GEU
//
//   ■ 控制信号宏 — 语义化控制信号值
//     MEM_READ, MEM_WRITE, REG_WRITE
//
//   ■ AXI4 协议宏 — 用于总线互联（本核心未直接使用）
//     AXI_RESP_OKAY, AXI_RESP_EXOKAY, AXI_RESP_SLVERR, AXI_RESP_DECERR
//     AXI_BURST_FIXED, AXI_BURST_INCR, AXI_BURST_WRAP
//     AXI_SIZE_BYTE, AXI_SIZE_HALF, AXI_SIZE_WORD
//
//   ■ Cache 参数宏 — 直接映射 Cache 结构参数（Cache 旁路后未使用）
//     ICACHE_WORD_BITS, ICACHE_NUM_LINES, ICACHE_INDEX_BITS, ICACHE_TAG_BITS
//     DCACHE_WORD_BITS, DCACHE_NUM_LINES, DCACHE_INDEX_BITS, DCACHE_TAG_BITS
//
// ============================================================

`include "Def.v"

module rv32im_core (
    // ========================================================================
    //  全局信号
    // ========================================================================
    input  clk,                          // 系统时钟，全同步设计统一时钟域
    input  reset,                        // 异步复位（高有效），复位所有流水线寄存器

    // ========================================================================
    //  统一存储器总线接口（Unified Memory Bus）
    //  旁路 Cache 直连 SoC system_bus，用于调试模式
    // ========================================================================
    output [31:0] mem_addr,              // 存储器地址总线（来自 MEM 阶段的 ALU 结果）
    output [31:0] mem_wdata,             // 存储器写数据总线（来自 MEM 阶段的 store_data）
    output        mem_we,                // 存储器写使能（高有效）
    output        mem_re,                // 存储器读使能（高有效）
    output [3:0]  mem_be,                // ★ 字节写使能掩码（来自 store_unit，用于 SB/SH/SW）
    input  [31:0] mem_rdata,             // 存储器读回数据
    input         mem_ready,             // 存储器就绪信号（握手完成标志）

    // ========================================================================
    //  中断接口（来自 PLIC 平台级中断控制器）
    // ========================================================================
    input         irq,                   // 中断请求信号（当前版本未在核心内处理）
    input  [4:0]  irq_id                 // 中断源 ID（标识是哪个外设发起中断）
);

  // ==========================================================================
  //  内部信号声明区 — 按流水线阶段分组
  //  所有 wire 必须在模块体开始处声明，以避免 Verilog 隐式声明问题
  // ==========================================================================

  // ==========================================================================
  //  IF 阶段信号（Instruction Fetch — 取指）
  //  职责：产生 PC 地址、从指令存储器取指、处理分支跳转
  // ==========================================================================
  wire [31:0] pc;                       // 当前程序计数器值（输出到指令存储器）
  wire [31:0] pc_plus4;                 // PC + 4，用于 JAL/JALR 的返回地址
  wire [31:0] branch_target;            // 分支目标地址（来自 prog_counter 的 pc_imm）
  wire [31:0] branch_offset;            // 分支偏移量（来自 ID 阶段的立即数生成器）
  wire [31:0] jalr_target;              // JALR 目标地址 = rs1_data + imm
  wire        pc_mux_in;                // PC 选择信号：1=选择 jalr_target，0=正常顺序/branch
  wire        branch_taken;             // 分支预测输出：1=预测跳转，0=预测不跳转
  wire        wrong_predict;            // 分支预测错误标志：预测结果 XOR 实际结果
  wire        jalr_enable;              // JALR 使能信号（控制 PC MUX 和分支单元）
  wire        pipeline_stall;           // 总流水线停顿信号（多个停顿源 OR 的结果）
  wire        bypass_stall;             // Load-Use 冒险导致的停顿（bypass_unit 输出）
  wire        bypass_u_type_ctrl;       // U 型指令绕过控制（当前未使用，保留接口）
  wire        m_unit_stall;             // 多周期乘除法单元忙标志 → 冻结流水线
  wire [31:0] instr;                    // 从指令存储器读出的 32 位指令字
  wire        icache_ready;             // I-Cache 就绪信号 → stall_unit（旁路模式下用 imem_ready）

  // ==========================================================================
  //  ID 阶段信号（Instruction Decode — 译码）
  //  职责：译码指令、读取寄存器文件、生成立即数、检测冒险、分支预测
  // ==========================================================================
  wire [31:0] id_pc;                    // ID 阶段的 PC 值（来自 IF/ID 流水线寄存器）
  wire [31:0] id_instr;                 // ID 阶段的指令字（来自 IF/ID 流水线寄存器）
  wire [4:0]  id_rs1_addr;             // 源寄存器 1 地址（decoder 输出）
  wire [4:0]  id_rs2_addr;             // 源寄存器 2 地址（decoder 输出）
  wire [4:0]  id_rd_addr;              // 目的寄存器地址（decoder 输出）
  wire [31:0] id_rs1_data;             // 源寄存器 1 读出数据（来自 reg_file）
  wire [31:0] id_rs2_data;             // 源寄存器 2 读出数据（来自 reg_file）
  wire [6:0]  id_opcode;               // 指令操作码 opcode[6:0]（decoder 输出）
  wire [6:0]  id_funct7_full;          // 完整 funct7[6:0]（decoder 输出）
  wire [2:0]  id_funct3;               // funct3[2:0]（decoder 输出）
  wire        id_funct7_bit;           // funct7[5]：区分 ADD/SUB 和 MUL/DIV 的关键位
  wire        id_load_itype;           // 是否为 load 类 I 型指令（用于控制单元）
  wire [31:0] decoder_pc_imm;          // decoder 计算的 PC+立即数（用于分支目标，当前未用）
  wire [31:0] decoder_pc_4;            // decoder 传递的 PC+4（当前未用）
  wire        id_werf;                 // 寄存器文件写使能（control_unit 输出）
  wire [1:0]  id_wbmux;                // 写回数据选择：00=ALU, 01=MEM(load), 10=PC+4(JAL/JALR)
  wire [3:0]  id_aluop;                // ALU 操作码（control_unit 输出，4-bit）
  wire [1:0]  id_irmux;                // 立即数/寄存器选择：bit0=rs2用imm, bit1=rs1用imm
  wire        id_ctrl_stall;           // 控制单元停顿请求（当前未使用，保留）
  wire [2:0]  id_op_format;            // 指令格式类型（I/S/B/U/J），驱动 imm_gen 的 mux
  wire        bypass_ir1;              // 源寄存器 1 是否需要前递（control_unit 输出）
  wire        bypass_ir2;              // 源寄存器 2 是否需要前递（control_unit 输出）
  wire        sb_type;                 // 是否为 S/B 型指令（用于 bypass_unit）
  wire        out_u_type;              // 是否为 U 型输出指令（LUI/AUIPC，用于 bypass_unit）
  wire [31:0] id_imm;                  // 立即数生成器输出的 32 位立即数
  wire [1:0]  bypass_irmux1;           // 源操作数 1 前递选择：
                                       //   2'b00=寄存器文件, 2'b01=MEM阶段ALU结果,
                                       //   2'b10=WB阶段写回数据
  wire [1:0]  bypass_irmux2;           // 源操作数 2 前递选择（同上编码）
  wire        hazard_pc_write;         // 冒险检测：是否允许更新 PC（Hazard_Detection 输出）
  wire        hazard_ifid_write;       // 冒险检测：是否允许更新 IF/ID 寄存器
  wire        hazard_ctrl_mux_sel;     // 冒险检测：是否在 ID/EX 寄存器插入 NOP（冲刷控制）
  wire        id_mem_write;            // ID 阶段判断当前指令是否为 S-type 存储指令

  // ==========================================================================
  //  ID/EX 流水线寄存器输出信号（进入 EX 阶段）
  //  这些信号锁存了 ID 阶段产生的控制信息和数据，供 EX 阶段使用
  // ==========================================================================
  wire [31:0] ex_pc;                   // EX 阶段的 PC 值
  wire [31:0] ex_rs1_data;             // EX 阶段的源寄存器 1 原始数据（前递前）
  wire [31:0] ex_rs2_data;             // EX 阶段的源寄存器 2 原始数据（前递前）
  wire [31:0] ex_imm;                  // EX 阶段的立即数
  wire [4:0]  ex_rs1_addr;             // EX 阶段的源寄存器 1 地址（供前递比较用）
  wire [4:0]  ex_rs2_addr;             // EX 阶段的源寄存器 2 地址（供前递比较用）
  wire [3:0]  ex_aluop_orig;           // EX 阶段的原始 ALU 操作码（来自 control_unit）
  wire        ex_alu_src;              // ALU 第二操作数选择：1=立即数(ex_imm)，0=寄存器(ex_fwd_rs2)
  wire        ex_mem_write;            // EX 阶段存储写使能
  wire [2:0]  ex_funct3;               // EX 阶段的 funct3
  wire [6:0]  ex_opcode;               // EX 阶段的操作码
  wire [1:0]  ex_wbmux;                // EX 阶段的写回选择
  wire [1:0]  ex_fwd1;                 // ★ 锁存的前递选择信号 1（从 ID 阶段的 bypass_irmux1 锁存）
  wire [1:0]  ex_fwd2;                 // ★ 锁存的前递选择信号 2（从 ID 阶段的 bypass_irmux2 锁存）

  // ==========================================================================
  //  EX 阶段信号（Execute — 执行）
  //  职责：前递选择操作数、ALU 运算、M 单元运算、分支跳转判断
  // ==========================================================================
  wire [4:0]  ex_rd;                   // EX 阶段的目的寄存器地址（来自 ID/EX 寄存器）
  wire        ex_mem_read;             // EX 阶段的存储器读使能（load 指令标志）
  wire        ex_reg_write;            // EX 阶段的寄存器写使能
  wire [31:0] ex_alu_operand1;         // ALU 操作数 1（前递后的最终值）
  wire [31:0] ex_alu_operand2;         // ALU 操作数 2（前递后的最终值，或立即数）
  wire [31:0] ex_alu_result;           // EX 阶段最终运算结果（ALU 或 M-unit 的 mux 输出）
  wire        ex_is_m_op;              // 当前 EX 指令是否为 M 扩展操作（乘除法）
  wire [31:0] alu_raw_result;          // ALU 原始运算结果（非 M 操作时使用）
  wire [31:0] m_unit_result;           // 乘除法单元运算结果（M 操作时使用）
  wire        m_unit_start;            // 启动乘除法单元的信号

  // ==========================================================================
  //  EX/MEM 流水线寄存器输出信号（进入 MEM 阶段）
  // ==========================================================================
  wire [31:0] mem_alu_result;          // MEM 阶段的 ALU 结果（用作存储器地址或传递到 WB）
  wire [31:0] mem_write_data;          // MEM 阶段的存储数据（已前递的 rs2 数据）
  wire [31:0] mem_pc_plus_4;           // MEM 阶段的 PC+4（用于 JAL/JALR 的返回地址写回）
  wire        mem_mem_write;           // MEM 阶段存储器写使能
  wire        mem_mem_read;            // MEM 阶段存储器读使能
  wire [4:0]  mem_rd;                  // MEM 阶段目的寄存器地址
  wire        mem_reg_write;           // MEM 阶段寄存器写使能
  wire [1:0]  mem_wbmux;               // MEM 阶段写回选择

  // ==========================================================================
  //  MEM 阶段信号（Memory Access — 访存）
  //  职责：通过总线读写存储器、load_unit 处理读回数据、store_unit 处理写数据
  // ==========================================================================
  wire [31:0] mem_addr_int;            // 内部存储器地址（直连 mem_alu_result）
  wire [31:0] mem_wdata_int;           // 内部存储器写数据（直连 mem_write_data）
  wire        mem_we_int;              // 内部存储器写使能
  wire        mem_re_int;              // 内部存储器读使能
  wire [31:0] mem_rdata_int;           // 内部存储器读回数据（来自总线 mem_rdata）
  wire [31:0] load_result;             // load_unit 处理后的加载数据（字节/半字/字对齐与扩展）
  wire        load_mux_sel;            // load_unit 输出选择（当前未使用，保留）
  reg  [2:0]  id_ex_funct3;            // ★ funct3 追踪寄存器：ID→ID/EX 阶段
  reg  [2:0]  ex_mem_funct3;           // ★ funct3 追踪寄存器：ID/EX→EX/MEM 阶段
                                       // 两级移位寄存器解决 funct3 不在流水线寄存器中传递的问题

  // ==========================================================================
  //  存储单元信号
  // ==========================================================================
  wire [31:0] store_data_out;          // store_unit 对齐后的存储数据
  wire [3:0]  store_mask;              // 字节写掩码（4-bit，每位对应一个字节）
  wire        store_req;               // 存储请求信号（当前未使用，保留供 Cache 使用）

  // ==========================================================================
  //  MEM/WB 流水线寄存器输出 ± WB 阶段信号（Write Back — 写回）
  //  职责：选择最终的写回数据，写入寄存器文件
  // ==========================================================================
  wire [31:0] wb_write_data;           // 最终写回数据（经 wbmux 选择）
  wire [4:0]  wb_write_addr;           // 写回目的寄存器地址
  wire        wb_write_enable;         // 写回使能（寄存器文件写使能）
  wire [31:0] wb_alu_result;           // WB 阶段的 ALU 结果（来自 MEM/WB 寄存器）
  wire [31:0] wb_load_data;            // WB 阶段的加载数据（来自 MEM/WB 寄存器）
  wire [31:0] wb_pc_plus_4;            // WB 阶段的 PC+4（来自 MEM/WB 寄存器）
  wire [1:0]  wb_wbmux;                // WB 阶段的写回数据选择控制

  // ==========================================================================
  //  分支跳转相关信号
  // ==========================================================================
  wire        branch_taken_ex;         // EX 阶段分支实际结果（branch_unit 输出）
  wire        jalr_enable_ex;          // EX 阶段 JALR 实际使能（branch_unit 输出）
  wire        if_flush;                // IF 阶段冲刷信号（= wrong_predict）


  // ##########################################################################
  //  IF 阶段（Instruction Fetch — 取指）
  //
  //  流程：
  //    1. stall_unit 综合各停顿源，产生 pipeline_stall
  //    2. prog_counter 根据 stall/branch/jalr/wrong_predict 产生下一条 PC
  //    3. instruction_memory 根据 PC 读出 32 位指令
  //    4. IF/ID 流水线寄存器锁存 PC 和指令，传递给 ID 阶段
  //
  //  关键设计：
  //    - pc_mux_in = jalr_enable，JALR 优先级高于 branch
  //    - wrong_predict 触发 IF 冲刷（if_flush），使 IF/ID 寄存器装入 NOP
  //    - pipeline_stall 冻结 PC 和 IF/ID 寄存器，保持当前状态
  // ##########################################################################

  // PC+4：用于 JAL/JALR 的返回地址（x[rd] = pc + 4）
  assign pc_plus4 = pc + 32'd4;

  // --------------------------------------------------------------------------
  //  stall_unit：流水线停顿控制单元
  //  汇集所有停顿源，产生统一的 pipeline_stall 信号
  //
  //  停顿源：
  //    - imem_ready：指令存储器未就绪（Cache miss / 总线等待）
  //    - dmem_ready：数据存储器未就绪（load/store 等待总线握手）
  //    - bypass_stall：Load-Use 数据冒险（load 指令后紧跟使用其结果的指令）
  //    - m_unit_stall：多周期乘除法单元正在运算
  //
  //  停顿效果：pipeline_stall=1 时：
  //    - PC 不更新（prog_counter 保持当前值）
  //    - IF/ID 寄存器不更新（保持上一条指令）
  //    - 其他流水线寄存器也冻结
  // --------------------------------------------------------------------------
  stall_unit u_stall (
    .imem_ready(imem_ready),            // 指令存储器就绪
    .dmem_ready(dmem_ready),            // 数据存储器就绪（总线握手完成）
    .load_use_stall(bypass_stall),      // Load-Use 冒险停顿
    .multi_cycle_stall(m_unit_stall),   // 多周期乘除法停顿
    .stall(pipeline_stall)              // 总停顿输出
  );

  // PC MUX 控制：JALR 使能时选择 jalr_target，否则由 prog_counter 内部控制
  assign pc_mux_in = jalr_enable;

  // --------------------------------------------------------------------------
  //  prog_counter：程序计数器模块
  //
  //  输入控制信号及其优先级（模块内部处理）：
  //    - reset：清零 PC
  //    - stall_in：停顿 → 保持当前 PC
  //    - pc_mux_in (JALR)：选择 jalr_target
  //    - branch_taken_in + ready_in：选择 branch_target（预测跳转 + JALR使能）
  //    - wrong_predict_in：预测错误 → 选择正确的目标地址
  //    - 默认：pc_plus4（顺序执行）
  //
  //  输出：
  //    - pc_out：下一周期的取指地址
  //    - pc_imm：计算的分支/跳转目标地址（= pc + imm）
  //
  // --------------------------------------------------------------------------
  prog_counter u_pc (
    .clk(clk),
    .reset(reset),
    .jalr_type_in(jalr_target),          // JALR 目标地址 = rs1 + imm
    .immbj(branch_offset),               // 分支偏移量（来自 ID 阶段立即数）
    .stall_in(pipeline_stall_diag),      // ★ 流水线停顿修复：强制 stall=0，配合 1-cycle Inst_memory
    .pc_mux_in(pc_mux_in),               // JALR 选择信号
    .branch_taken_in(branch_taken),      // 分支预测结果
    .ready_in(jalr_enable),              // JALR 使能
    .wrong_predict_in(wrong_predict),    // 预测错误标志
    .pc_plus4_in(pc_plus4),              // PC+4（顺序执行默认值）
    .pc_imm(branch_target),              // 分支目标地址输出
    .pc_out(pc)                          // 最终 PC 输出
  );

  // --------------------------------------------------------------------------
  //  instruction_memory：指令存储器（I-Cache 旁路模式）
  //
  //  ★ BYPASS CACHE：当前版本绕过 I-Cache，直接使用原始的 instruction_memory
  //    模块。该模块内部通常映射到 BRAM 的一个端口，实现单周期读取。
  //
  //  接口：
  //    - addr(pc)：32 位字节地址
  //    - instr：32 位指令输出
  //    - imem_ready：指令就绪标志
  // --------------------------------------------------------------------------
  wire imem_ready;
  instruction_memory u_imem (
    .clk(clk),
    .reset(reset),
    .addr(pc),
    .instr(instr),
    .imem_ready(imem_ready)
  );


  // ##########################################################################
  //  IF/ID 流水线寄存器
  //
  //  功能：在 IF 和 ID 阶段之间插入寄存器，实现流水线隔离
  //
  //  控制信号：
  //    - stall：pipeline_stall=1 时保持当前值（冻结）
  //    - flush：if_flush=1 时清零输出（插入 NOP，等效于指令 0x00000000）
  //
  //  flush 触发条件：wrong_predict=1，即分支预测错误
  //  此时 IF 阶段已取到错误路径的指令，必须丢弃
  // ##########################################################################

  // if_flush：IF 阶段冲刷信号，等于 wrong_predict
  // 当分支预测错误时，IF/ID 寄存器中的错误指令必须被清除
  assign if_flush = wrong_predict;

  if_id_stage u_if_id (
    .clk(clk),
    .reset(reset),
    .stall(pipeline_stall),             // 流水线停顿时保持
    .flush(if_flush),                   // 预测错误时清零
    .pc(pc),                            // 当前 PC 输入
    .instr(instr),                      // 当前指令输入
    .if_id_pc(id_pc),                   // ID 阶段 PC 输出
    .if_id_instr(id_instr)              // ID 阶段指令输出
  );


  // ##########################################################################
  //  ID 阶段（Instruction Decode — 译码）
  //
  //  流程：
  //    1. decoder：从 32 位指令中提取 opcode/funct3/funct7/寄存器地址
  //    2. control_unit：根据 opcode/funct3/funct7 生成所有控制信号
  //    3. reg_file：读取源寄存器数据
  //    4. imm_gen：根据指令格式生成 32 位立即数
  //    5. bypass_unit：检测数据前递需求，生成前递选择信号
  //    6. Hazard_Detection_Unit：检测 Load-Use 和分支冒险
  //    7. branch_prediction_unit：预测分支方向
  //    8. jalr_type_adder：计算 JALR 目标地址
  // ##########################################################################

  // --------------------------------------------------------------------------
  //  decoder：指令译码器
  //  从 32 位指令字中提取所有字段：
  //    - 源/目的寄存器地址（rs1, rs2, rd）
  //    - opcode, funct3, funct7
  //    - 指令类型标志（load_itype）
  // --------------------------------------------------------------------------
  decoder u_decoder (
    .instr_in(id_instr),
    .bypass_stall(pipeline_stall),
    .clk(clk),
    .reset(reset),
    .stall(pipeline_stall),
    .pc_imm_in(branch_offset),          // 用于计算分支目标（内部使用）
    .pc_4_in(pc_plus4),                 // 用于计算返回地址（内部使用）
    .wrong_predict_in(wrong_predict),
    .source_reg_1(id_rs1_addr),         // rs1 地址 [19:15]
    .source_reg_2(id_rs2_addr),         // rs2 地址 [24:20]
    .dest_reg(id_rd_addr),              // rd  地址 [11:7]
    .opcode_out(id_opcode),             // opcode [6:0]
    .funct_3_out(id_funct3),            // funct3 [14:12]
    .funct_7_out(id_funct7_full),       // funct7 [31:25]
    .load_itype(id_load_itype),         // 是否为 load 类型 I 指令
    .instr_out(),                       // 指令输出（悬空，不使用）
    .pc_imm_out(decoder_pc_imm),        // PC+imm 输出（悬空，不使用）
    .pc_4_out(decoder_pc_4)             // PC+4 输出（悬空，不使用）
  );

  // funct7[5]：区分 RV32I 和 RV32M 的关键位
  //   - funct7[5]=0：RV32I 操作（ADD/SUB/SLL/SRL/SRA/SLT/SLTU/XOR/OR/AND）
  //   - funct7[5]=1：RV32M 乘除法操作（MUL/MULH/.../DIV/REM）
  assign id_funct7_bit = id_funct7_full[5];

  // --------------------------------------------------------------------------
  //  control_unit：主控制单元
  //
  //  输入：opcode, funct3, funct7_bit, load_type
  //  输出：数据通路所有控制信号
  //
  //  关键输出信号说明：
  //    - werf_contrl：寄存器文件写使能（算术/load/LUI/AUIPC/JAL/JALR=1，store/branch=0）
  //    - wbmux_contol：写回数据选择（00=ALU结果, 01=load数据, 10=PC+4, 11=保留）
  //    - aluop：ALU 操作选择（4-bit编码，包含 M 扩展操作码）
  //    - ir_mux：立即数/寄存器选择（bit0=rs2用imm, bit1=rs1用imm）
  //    - op_format_out：立即数格式（I/S/B/U/J）
  //    - bypass_ir1/ir2：源寄存器是否需要前递（1=需要）
  //    - sb_type：S/B 类型指令标志
  //    - out_u_type：U 型输出指令标志（LUI/AUIPC）
  // --------------------------------------------------------------------------
  control_unit u_control (
    .clk(clk),
    .reset(reset),
    .load_type_in(id_load_itype),
    .opcode_in(id_opcode),
    .funct3_in(id_funct3),
    .funct7_in(id_funct7_bit),
    .werf_contrl(id_werf),              // 寄存器写使能
    .wbmux_contol(id_wbmux),            // 写回数据选择
    .aluop(id_aluop),                   // ALU 操作码
    .ir_mux(id_irmux),                  // 立即数选择
    .stall(id_ctrl_stall),              // 控制单元停顿（保留）
    .op_format_out(id_op_format),       // 指令格式类型
    .bypass_ir1(bypass_ir1),            // rs1 前递需求
    .bypass_ir2(bypass_ir2),            // rs2 前递需求
    .sb_type(sb_type),                  // S/B 类型标志
    .out_u_type(out_u_type),            // U 型输出标志
    .reg_u_type_lui(),                  // LUI 标志（悬空）
    .ready_in(jalr_enable),             // JALR 使能
    .i_type_load()                      // I 型 load 标志（悬空）
  );

  // --------------------------------------------------------------------------
  //  reg_file：寄存器文件（32 个 32 位通用寄存器）
  //
  //  RISC-V 规范：
  //    - x0 硬连线为 0（read_data 为 0，忽略 write）
  //    - 双读端口（组合逻辑读出）+ 单写端口（时序逻辑写入）
  //    - WB 阶段写入：wb_write_enable=1 时在 clk 上升沿写入 wb_write_data
  //
  //  读操作在 ID 阶段进行（读取 rs1, rs2），
  //  写操作在 WB 阶段进行（写入 rd），时间差 = 3 个周期
  //  前递机制弥补了这 3 个周期的数据新鲜度问题
  // --------------------------------------------------------------------------
  reg_file u_regfile (
    .clk(clk),
    .reset(reset),
    .read_address1(id_rs1_addr),        // rs1 读地址
    .read_address2(id_rs2_addr),        // rs2 读地址
    .write_enable(wb_write_enable),     // 写使能（来自 WB 阶段）
    .write_address(wb_write_addr),      // 写地址（rd，来自 WB 阶段）
    .write_data(wb_write_data),         // 写数据（来自 WB 阶段 MUX）
    .read_data1(id_rs1_data),           // rs1 读出数据
    .read_data2(id_rs2_data)            // rs2 读出数据
  );

  // --------------------------------------------------------------------------
  //  imm_gen：立即数生成器
  //
  //  根据指令格式（op_format）从 instr[31:7] 拼接出 32 位立即数：
  //    - I_type：符号扩展 instr[31:20]
  //    - S_type：符号扩展 {instr[31:25], instr[11:7]}
  //    - B_type：符号扩展 {instr[31], instr[7], instr[30:25], instr[11:8], 1'b0}
  //    - U_type：{instr[31:12], 12'b0}
  //    - J_type：符号扩展 {instr[31], instr[19:12], instr[20], instr[30:21], 1'b0}
  // --------------------------------------------------------------------------
  imm_gen u_immgen (
    .instr(id_instr[31:7]),             // 指令高 25 位（含立即数字段）
    .imm_mux(id_op_format),             // 立即数格式选择
    .imm(id_imm)                        // 32 位立即数输出
  );
  // branch_offset 连接到 id_imm：分支指令使用 B_type 格式的立即数作为偏移量
  assign branch_offset = id_imm;

  // --------------------------------------------------------------------------
  //  bypass_unit：前递单元（位于 ID 阶段）
  //
  //  功能：检测数据冒险，生成前递选择信号（bypass_irmux1/2）
  //
  //  前递策略（按优先级）：
  //    1. EX 阶段前递（bypass_irmux = 2'b01）：
  //       若 ID 的 rs1/rs2 与 EX 阶段的 rd 匹配，且 EX 阶段指令会写寄存器，
  //       则前递 EX 阶段的 ALU 结果（当前在 mem_alu_result 线上）
  //       注意：此时 EX 的结果尚未写入寄存器文件（差 2 个周期）
  //    2. MEM 阶段前递（bypass_irmux = 2'b10）：
  //       若 ID 的 rs1/rs2 与 MEM 阶段的 rd 匹配，且 MEM 阶段指令会写寄存器，
  //       则前递 MEM 阶段的结果（当前在 wb_write_data 线上，即将写回）
  //       注意：MEM 结果比 EX 更新，但 WB 结果已经写入寄存器文件（无需前递）
  //    3. 无前递（bypass_irmux = 2'b00）：
  //       从寄存器文件读取（默认路径）
  //
  //  Load-Use 停顿：
  //    若 ID 的 rs1/rs2 与 EX 阶段的 rd 匹配，且 EX 指令是 load（ex_load=1），
  //    则 load 数据要在 MEM 阶段结束时才可用（差 1 个周期，无法前递），
  //    必须停顿 1 个周期（bypass_stall=1）
  //
  //  输入信号：
  //    - bypass_ir1/ir2：control_unit 指示该源寄存器是否被当前指令使用
  //    - ex_rd, ex_reg_write, ex_load：EX 阶段的目的寄存器信息
  //    - mem_rd, mem_reg_write：MEM 阶段的目的寄存器信息
  //    - rs1, rs2：ID 阶段的源寄存器地址
  //
  //  输出信号：
  //    - irmux1, irmux2：前递选择信号（锁存到 ID/EX 流水线寄存器中）
  //    - stall_out：Load-Use 停顿请求
  // --------------------------------------------------------------------------
  bypass_unit u_bypass (
    .clk(clk),
    .reset(reset),
    .flush(wrong_predict),              // 预测错误时复位前递状态
    .sb_type_in(sb_type),               // S/B 类型指令标志
    .in_u_type(out_u_type),             // U 型输出指令标志
    .bypass_ir1_in(bypass_ir1),         // rs1 是否需要前递
    .bypass_ir2_in(bypass_ir2),         // rs2 是否需要前递
    .op_format_contrl_in(id_op_format), // 指令格式类型
    .ir_mux_in(id_irmux),               // 立即数选择控制
    .rs1(id_rs1_addr),                  // ID 阶段 rs1 地址
    .rs2(id_rs2_addr),                  // ID 阶段 rs2 地址
    // ★ 显式流水线信号（替代内部移位寄存器，增强可读性和可控性）
    .ex_rd(ex_rd),                      // EX 阶段目的寄存器地址
    .ex_reg_write(ex_reg_write),        // EX 阶段寄存器写使能
    .ex_load(ex_mem_read),              // EX 阶段是否为 load 指令
    .mem_rd(mem_rd),                    // MEM 阶段目的寄存器地址
    .mem_reg_write(mem_reg_write),      // MEM 阶段寄存器写使能
    .irmux1(bypass_irmux1),             // rs1 前递选择输出
    .irmux2(bypass_irmux2),             // rs2 前递选择输出
    .stall_out(bypass_stall),           // Load-Use 停顿输出
    .u_type_contrl_in(bypass_u_type_ctrl) // U 型控制（保留）
  );

  // --------------------------------------------------------------------------
  //  Hazard_Detection_Unit：冒险检测单元
  //
  //  检测以下两种冒险：
  //
  //  1. Load-Use 数据冒险（检测 ID 阶段与 EX 阶段的寄存器依赖）：
  //     若 ID 阶段的 rs1 或 rs2 与 EX 阶段的 rd 相同，
  //     且 EX 阶段是 load 指令（ex_mem_read=1），
  //     则需要停顿 1 个周期：冻结 PC 和 IF/ID 寄存器，在 ID/EX 插入 NOP
  //
  //  2. 分支/JALR 控制冒险：
  //     分支指令在 ID 阶段预测，但真实方向在 EX 阶段才确认。
  //     JALR 指令目标地址在 ID 阶段即可算出，但仍需 1 个周期的延迟。
  //     冒险检测单元协调冲刷和停顿。
  //
  //  输出：
  //    - PC_write：PC 更新使能（0=冻结PC）
  //    - IF_ID_write：IF/ID 寄存器更新使能（0=冻结）
  //    - control_mux_sel：ID/EX 寄存器冲刷使能（1=插入NOP/清零控制信号）
  // --------------------------------------------------------------------------
  Hazard_Detection_Unit u_hazard (
    .ID_EX_rs1(id_rs1_addr),            // ID 阶段 rs1 地址
    .ID_EX_rs2(id_rs2_addr),            // ID 阶段 rs2 地址
    .EX_MEM_rd(ex_rd),                  // EX 阶段 rd 地址
    .MEM_WB_rd(mem_rd),                 // MEM 阶段 rd 地址
    .ID_EX_mem_read(ex_mem_read),       // EX 阶段 load 标志（注意：连接同一信号）
    .EX_MEM_mem_read(ex_mem_read),      // EX 阶段 load 标志（注意：连接同一信号）
    .branch_taken(branch_taken_ex),     // EX 阶段分支实际结果
    .jump(jalr_enable),                 // JALR 使能
    .PC_write(hazard_pc_write),         // PC 写使能输出
    .IF_ID_write(hazard_ifid_write),    // IF/ID 写使能输出
    .control_mux_sel(hazard_ctrl_mux_sel) // ID/EX NOP 插入控制
  );

  // id_mem_write：判断当前 ID 指令是否为 S-type 存储指令
  // 用于 ID/EX 流水线寄存器的 mem_write 信号
  assign id_mem_write = (id_op_format == `S_type);

  // --------------------------------------------------------------------------
  //  branch_prediction_unit：分支预测单元（位于 ID 阶段）
  //
  //  采用简单的静态/动态混合预测策略：
  //    - 输入当前指令的 opcode，识别是否为分支指令
  //    - 预测分支方向（branch_taken_out），默认预测不跳转
  //    - 预测结果在 EX 阶段被验证（branch_taken_ex 为实际结果）
  //    - 若预测错误（wrong_predict=1），冲刷 IF 阶段并修正 PC
  //
  //  ★ 设计说明：
  //    - branch_in 固定为 0：不使用外部分支输入，wrong_predict 在顶层由
  //      branch_taken_d ^ (branch_taken_ex | jalr_enable_ex) 生成
  //    - wrong_predict_out 未连接：预测错误检测逻辑在核心顶层实现，
  //      使用延迟 1 周期的预测值与 EX 阶段实际值比较
  //    - is_branch 未连接：分支类型信息在其他路径传递
  // --------------------------------------------------------------------------
  branch_prediction_unit u_branch_pred (
    .clock(clk),
    .opcode_in(id_opcode),              // ID 阶段操作码
    .reset(reset),
    .branch_in(1'b0),                   // ★ 不使用外部分支输入
    .branch_taken_out(branch_taken),     // 分支预测输出
    .wrong_predict_out(),               // ★ 不使用（外部生成 wrong_predict）
    .is_branch()                        // ★ 不使用（悬空）
  );

  // --------------------------------------------------------------------------
  //  jalr_type_adder：JALR 目标地址加法器
  //
  //  JALR 指令：jalr rd, offset(rs1)
  //  目标地址 = rs1_data + sign_extend(offset)
  //
  //  在 ID 阶段计算，因为 JALR 不依赖 ALU（使用独立的加法器），
  //  可以在 ID 阶段就确定跳转目标，减少 1 个周期的分支延迟
  // --------------------------------------------------------------------------
  jalr_type_adder u_jalr (
    .reg_1(id_rs1_data),                // rs1 读出数据
    .imm(id_imm),                       // 立即数（I-type 格式）
    .jalr_out(jalr_target)              // JALR 目标地址 = rs1 + imm
  );


  // ##########################################################################
  //  ID/EX 流水线寄存器
  //
  //  功能：锁存 ID 阶段产生的所有控制信号和数据，传递给 EX 阶段
  //
  //  关键设计：
  //    1. alu_src 的生成：
  //       id_alu_src = id_irmux[0] || (id_op_format == S_type)
  //       即在以下情况 ALU 第二操作数使用立即数：
  //         - id_irmux[0]=1（control_unit 指示 rs2 使用立即数，如 I-type 算术指令）
  //         - S_type 存储指令（地址计算 = rs1 + imm）
  //
  //    2. ★ 前递选择信号锁存：
  //       id_fwd1 = bypass_irmux1，id_fwd2 = bypass_irmux2
  //       将 ID 阶段计算的前递选择信号锁存到 ID/EX 寄存器，
  //       在 EX 阶段使用 ex_fwd1/ex_fwd2 进行前递 MUX 选择。
  //       这是本设计的核心创新：前递控制信号在 ID 阶段预计算，
  //       EX 阶段直接使用，避免 EX 阶段进行复杂的寄存器地址比较。
  //
  //    3. flush 信号：
  //       flush = hazard_ctrl_mux_sel（来自 Hazard_Detection_Unit）
  //       当 Load-Use 冒险或分支冒险需要冲刷时，将 ID/EX 寄存器清零，
  //       等效于在 EX 阶段插入 NOP 指令（所有控制信号清零）。
  // ##########################################################################

  id_ex_stage u_id_ex (
    .clk(clk),
    .reset(reset),
    .stall(pipeline_stall),             // 流水线停顿时保持
    .flush(hazard_ctrl_mux_sel),        // 冒险冲刷时清零
    // ---- ID 阶段输入 ----
    .id_pc(id_pc),
    .id_rs1_data(id_rs1_data),
    .id_rs2_data(id_rs2_data),
    .id_imm(id_imm),
    .id_rs1(id_rs1_addr),
    .id_rs2(id_rs2_addr),
    .id_rd(id_rd_addr),
    .id_alu_ctrl(id_aluop),
    .id_alu_src(id_irmux[0] || (id_op_format == `S_type)), // ALU 第二操作数选择
    .id_mem_write(id_mem_write),        // 存储写使能
    .id_mem_read(id_load_itype),        // load 使能
    .id_reg_write(id_werf),             // 寄存器写使能
    .id_funct3(id_funct3),
    .id_opcode(id_opcode),
    .id_wbmux(id_wbmux),
    .id_fwd1(bypass_irmux1),            // ★ 锁存前递选择 1
    .id_fwd2(bypass_irmux2),            // ★ 锁存前递选择 2
    // ---- EX 阶段输出 ----
    .ex_pc(ex_pc),
    .ex_rs1_data(ex_rs1_data),
    .ex_rs2_data(ex_rs2_data),
    .ex_imm(ex_imm),
    .ex_rs1(ex_rs1_addr),
    .ex_rs2(ex_rs2_addr),
    .ex_rd(ex_rd),
    .ex_alu_ctrl(ex_aluop_orig),
    .ex_alu_src(ex_alu_src),
    .ex_mem_write(ex_mem_write),
    .ex_mem_read(ex_mem_read),
    .ex_reg_write(ex_reg_write),
    .ex_funct3(ex_funct3),
    .ex_opcode(ex_opcode),
    .ex_wbmux(ex_wbmux),
    .ex_fwd1(ex_fwd1),                  // ★ 锁存后的前递选择 1
    .ex_fwd2(ex_fwd2)                   // ★ 锁存后的前递选择 2
  );


  // ##########################################################################
  //  EX 阶段（Execute — 执行）
  //
  //  核心功能：
  //    1. 前递 MUX：根据锁存的 ex_fwd1/ex_fwd2 选择实际操作数
  //    2. ALU 运算：对非 M 操作执行算术/逻辑运算
  //    3. M-Unit 运算：对 M 操作启动多周期乘除法单元
  //    4. 分支判决：branch_unit 确认分支是否实际跳转
  //    5. 预测错误检测：比较预测值与实际值
  //
  //  【前递架构详解】
  //
  //  前递选择信号（ex_fwd1/ex_fwd2）在 ID 阶段由 bypass_unit 计算，
  //  在 ID/EX 流水线寄存器中锁存，在 EX 阶段用于 MUX 选择：
  //
  //    2'b00 → ex_rs1_data / ex_rs2_data（寄存器文件原始值，无前递）
  //    2'b01 → mem_alu_result（前递 MEM 阶段的 ALU 结果）
  //            场景：生产者指令正在 EX→MEM 传递，ALU 结果尚未写入寄存器文件
  //    2'b10 → wb_write_data（前递 WB 阶段的写回数据）
  //            场景：生产者指令在 MEM→WB 传递或正在 WB 阶段写回
  //
  //  注意：WB 阶段的结果已经写入寄存器文件（reg_file 在 WB 阶段写），
  //  但由于写操作在时钟上升沿，读操作是组合逻辑，同周期读可获取新值。
  //  前递 2'b10 提供了一个更快的路径（纯组合逻辑 MUX，无需经过 reg_file 读）。
  //
  //  【停顿机制详解】
  //
  //  停顿由 stall_unit 统一管理，产生 pipeline_stall：
  //
  //    pipeline_stall = imem_ready_stall || dmem_ready_stall
  //                   || bypass_stall || m_unit_stall
  //
  //    - imem_ready_stall：I-Cache miss，指令未就绪 → 冻结 IF 和 IF/ID
  //    - dmem_ready_stall：D-Cache miss 或总线未响应 → 冻结 MEM 及之前所有阶段
  //    - bypass_stall：Load-Use 冒险 → 冻结 IF 和 IF/ID，在 ID/EX 插入 NOP
  //    - m_unit_stall：M 单元正在计算 → 冻结 IF、IF/ID、ID/EX
  //
  //  停顿期间，被冻结的流水线寄存器保持当前值，PC 不更新。
  //  解冻后，流水线从停顿点继续执行。
  //
  //  【M-Unit 集成详解】
  //
  //  M 扩展指令的 ALU 操作码高两位为 2'b10：
  //    - ALU_MUL(4'b1010), ALU_MULH(4'b1011), ALU_MULHSU(4'b1100),
  //      ALU_MULHU(4'b1101), ALU_DIV(4'b1110), ALU_DIVU(4'b1111),
  //      ALU_REM(4'b1110), ALU_REMU(4'b1111)
  //
  //  检测条件：ex_is_m_op = ex_aluop_orig[3] && ex_aluop_orig[2]
  //
  //  M 操作流程：
  //    1. ex_is_m_op=1 → ALU 不执行实际运算（输出 ALU_ADD 的直通结果），
  //       M 单元被启动（m_unit_start=1）
  //    2. M 单元拉高 busy → m_unit_stall=1 → pipeline_stall=1，流水线冻结
  //    3. 多周期后 M 单元完成 → m_unit_result 有效，busy 拉低 → 流水线解冻
  //    4. ex_alu_result 选择 m_unit_result 而非 alu_raw_result
  //
  //  乘法的典型延迟：1 周期（单周期乘法器）
  //  除法的典型延迟：多周期（取决于实现，通常 2-33 周期）
  // ##########################################################################

  // --------------------------------------------------------------------------
  //  EX 阶段前递多路选择器
  //
  //  使用锁存在 ID/EX 寄存器中的前递选择信号（ex_fwd1/ex_fwd2），
  //  从以下三个来源中选择操作数：
  //    1. ex_rs1_data / ex_rs2_data：寄存器文件原始值（ID 阶段读出）
  //    2. mem_alu_result：MEM 阶段 ALU 结果（上一条指令在 EX 的结果）
  //    3. wb_write_data：WB 阶段即将写回的数据（上上条指令的结果）
  //
  //  优先级隐含在 bypass_unit 的编码中：
  //    2'b00（无前递）< 2'b01（EX前递）< 2'b10（MEM前递）
  //    MEM 前递优先级最高，因为数据更新
  // --------------------------------------------------------------------------
  wire [31:0] ex_fwd_rs1 = (ex_fwd1 == 2'b01) ? mem_alu_result :   // EX→EX 前递
                            (ex_fwd1 == 2'b10) ? wb_write_data :    // MEM→EX 前递
                            ex_rs1_data;                            // 寄存器文件默认值
  wire [31:0] ex_fwd_rs2 = (ex_fwd2 == 2'b01) ? mem_alu_result :   // EX→EX 前递
                            (ex_fwd2 == 2'b10) ? wb_write_data :    // MEM→EX 前递
                            ex_rs2_data;                            // 寄存器文件默认值

  // ALU 操作数 1：始终使用前递后的 rs1 值
  assign ex_alu_operand1 = ex_fwd_rs1;

  // ALU 操作数 2：根据 ex_alu_src 选择
  //   - ex_alu_src=1：使用立即数（I-type 算术/逻辑指令、S-type 地址计算、LUI/AUIPC）
  //   - ex_alu_src=0：使用前递后的 rs2 值（R-type 指令、B-type 分支比较）
  assign ex_alu_operand2 = ex_alu_src ? ex_imm : ex_fwd_rs2;

  // --------------------------------------------------------------------------
  //  M-extension 操作检测
  //
  //  M 操作检测：ex_aluop_orig[3]=1 且 (bit[2]=1 或 bit[1]=1)
  //
  //  M-extension ALU 操作码及其 bit 模式：
  //    ALU_MUL     = 4'b1010  → bit[3]=1, bit[2]=0, bit[1]=1
  //    ALU_MULH    = 4'b1011  → bit[3]=1, bit[2]=0, bit[1]=1
  //    ALU_MULHSU  = 4'b1100  → bit[3]=1, bit[2]=1, bit[1]=0
  //    ALU_MULHU   = 4'b1101  → bit[3]=1, bit[2]=1, bit[1]=0
  //    ALU_DIV     = 4'b1110  → bit[3]=1, bit[2]=1, bit[1]=1
  //    ALU_DIVU    = 4'b1111  → bit[3]=1, bit[2]=1, bit[1]=1
  //
  //  【Bug 修复】旧条件 ex_aluop_orig[3] && ex_aluop_orig[2] 只检测 bit[2]=1
  //  的情况（1100~1111），漏掉了 ALU_MUL(1010) 和 ALU_MULH(1011)。
  //  修复后使用 bit[2] || bit[1]，覆盖所有 bit[3]=1 的 M 操作码（1010~1111）。
  //
  //  注意：DIV 和 REM 共用 aluop 编码，mul_div_unit 内部通过 funct3 区分。
  // --------------------------------------------------------------------------
  assign ex_is_m_op = ex_aluop_orig[3] && (ex_aluop_orig[2] || ex_aluop_orig[1]);

  // --------------------------------------------------------------------------
  //  ALU：算术逻辑单元
  //
  //  当 ex_is_m_op=1 时，ALU 接收 ALU_ADD（直通 operand1），其结果被忽略
  //  （因为最终 ex_alu_result 选择 m_unit_result）。
  //  这是一种资源节约策略：不为 M 操作设计独立的数据通路，而是复用 ALU。
  //
  //  当 ex_is_m_op=0 时，ALU 执行正常的 RV32I 操作：
  //    ALU_ADD/ALU_SUB/ALU_XOR/ALU_OR/ALU_AND/
  //    ALU_SLL/ALU_SRL/ALU_SRA/ALU_SLT/ALU_SLTU
  // --------------------------------------------------------------------------
  ALU u_alu (
    .operand_1(ex_alu_operand1),
    .operand_2(ex_alu_operand2),
    .aluop(ex_is_m_op ? `ALU_ADD : ex_aluop_orig),  // M 操作时 ALU 忽略，执行 ADD
    .out(alu_raw_result)                             // ALU 原始结果
  );
  // --------------------------------------------------------------------------
  //  mul_div_unit：RV32M 乘除法单元
  //
  //  启动条件：m_unit_start = ex_is_m_op && !pipeline_stall
  //    - ex_is_m_op=1：当前 EX 指令是 M 操作
  //    - !pipeline_stall：流水线未停顿（避免重复启动）
  //
  //  工作流程：
  //    1. start=1 → 锁存操作数和操作类型，开始计算
  //    2. busy=1 → 通知 stall_unit 冻结流水线（m_unit_stall=1）
  //    3. 计算完成 → result 有效，busy=0 → 流水线解冻
  //
  //  操作类型（md_op）：
  //    - 乘法：MUL(低32位), MULH(高32位有符号), MULHSU(有符号×无符号高32位),
  //            MULHU(高32位无符号)
  //    - 除法：DIV(有符号), DIVU(无符号)
  //    - 余数：REM(有符号), REMU(无符号)
  // --------------------------------------------------------------------------
  assign m_unit_start = ex_is_m_op && !pipeline_stall;

  mul_div_unit u_mul_div (
    .clk(clk),
    .reset(reset),
    .op_a(ex_alu_operand1),             // 操作数 A（被乘数/被除数）
    .op_b(ex_alu_operand2),             // 操作数 B（乘数/除数）
    .md_op(ex_aluop_orig),              // 操作类型（4-bit ALU 操作码）
    .funct3_in(ex_funct3),              // ★ funct3 字段 → 区分 DIV/REM
    .start(m_unit_start),               // 启动信号
    .result(m_unit_result),             // 运算结果
    .busy(m_unit_stall)                 // 忙标志 → 停顿流水线
  );

  // --------------------------------------------------------------------------
  //  EX 阶段结果 MUX
  //
  //  M 操作：选择 m_unit_result（来自多周期乘除法单元）
  //  非 M 操作：选择 alu_raw_result（来自单周期 ALU）
  //
  //  这个 MUX 实现了 M 操作和非 M 操作的数据通路合并，
  //  后续流水线阶段无需关心结果来源。
  // --------------------------------------------------------------------------
  assign ex_alu_result = ex_is_m_op ? m_unit_result : alu_raw_result;


  // --------------------------------------------------------------------------
  //  branch_unit：分支判决单元（位于 EX 阶段）
  //
  //  输入：funct3（分支条件类型）、opcode（指令类型）、
  //        source_1/source_2（前递后的操作数）
  //
  //  输出：
  //    - branch_out：分支是否跳转（实际结果）
  //      BEQ：source_1 == source_2
  //      BNE：source_1 != source_2
  //      BLT：$signed(source_1) < $signed(source_2)
  //      BGE：$signed(source_1) >= $signed(source_2)
  //      BLTU：source_1 < source_2
  //      BGEU：source_1 >= source_2
  //    - jal_enab：JAL/JALR 跳转使能（始终为 1）
  //
  //  注意：分支操作数使用前递后的值（ex_fwd_rs1/ex_fwd_rs2），
  //  确保分支比较使用的是最新的数据。
  // --------------------------------------------------------------------------
  branch_unit u_branch (
    .funct3_in(ex_funct3),              // 分支条件类型
    .opcode_in(ex_opcode),              // 指令操作码
    .source_1(ex_fwd_rs1),              // 前递后的操作数 1
    .source_2(ex_fwd_rs2),              // 前递后的操作数 2
    .branch_out(branch_taken_ex),       // 分支实际跳转结果
    .jal_enab(jalr_enable_ex)           // JAL/JALR 使能
  );

  // --------------------------------------------------------------------------
  //  分支预测错误检测
  //
  //  设计原理：
  //    1. ID 阶段 branch_prediction_unit 预测分支方向（branch_taken）
  //    2. 预测值经过 1 周期延迟（branch_taken_d），与 EX 阶段实际结果对齐
  //    3. EX 阶段 branch_unit 给出实际分支方向（branch_taken_ex | jalr_enable_ex）
  //    4. wrong_predict = 预测值 XOR 实际值：
  //       - 预测跳转(1) 实际不跳转(0) → wrong_predict=1（需冲刷错误路径指令）
  //       - 预测不跳转(0) 实际跳转(1) → wrong_predict=1（需冲刷错误路径指令）
  //       - 预测正确 → wrong_predict=0
  //
  //  为什么需要 branch_taken_d（延迟 1 周期）？
  //    预测在 ID 阶段（Cycle N），但验证在 EX 阶段（Cycle N+1）。
  //    Cycle N+1 的 EX 阶段对应的是 Cycle N 的 ID 指令。
  //    若不加延迟，会用 Cycle N+1 的预测值比较 Cycle N+1 的实际值，
  //    时序上不匹配。延迟 1 周期确保比较的是同一条指令。
  //
  //  wrong_predict 的作用：
  //    - if_flush=1：冲刷 IF/ID 寄存器中的错误指令
  //    - prog_counter 内部修正 PC 到正确地址
  //    - bypass_unit 复位前递状态
  // --------------------------------------------------------------------------
  reg branch_taken_d;                   // 延迟 1 周期的分支预测值
  always @(posedge clk or posedge reset)
    if (reset)
      branch_taken_d <= 1'b0;
    else if (!pipeline_stall)           // 停顿时不更新（保持对齐）
      branch_taken_d <= branch_taken;

  // 预测错误 = 延迟预测值 XOR EX 阶段实际值
  // branch_taken_ex | jalr_enable_ex：任何跳转（分支或 JAL/JALR）都视为跳转
  assign wrong_predict = branch_taken_d ^ (branch_taken_ex | jalr_enable_ex);


  // ##########################################################################
  //  EX/MEM 流水线寄存器
  //
  //  功能：锁存 EX 阶段的运算结果和控制信号，传递给 MEM 阶段
  //
  //  关键设计：
  //    - ex_mem_write_data 使用 ex_fwd_rs2（前递后的 rs2 值），
  //      确保存储指令写入的是最新数据
  //    - ex_pc_plus_4 在连接前计算：ex_pc + 4，
  //      用于 JAL/JALR 指令在 WB 阶段写回 PC+4
  //    - flush 固定为 0：EX/MEM 阶段不需要冲刷
  //      （分支预测错误在 IF 和 ID/EX 层面处理）
  // ##########################################################################

  wire [31:0] ex_pc_plus_4 = ex_pc + 32'd4;  // EX 阶段 PC+4

  ex_mem_stage u_ex_mem (
    .clk(clk),
    .reset(reset),
    .stall(pipeline_stall),             // 流水线停顿时保持
    .flush(1'b0),                       // EX/MEM 不冲刷
    // ---- EX 阶段输入 ----
    .ex_alu_result(ex_alu_result),      // EX 阶段最终结果
    .ex_mem_write_data(ex_fwd_rs2),     // ★ 使用前递后的 rs2 作为存储数据
    .ex_rd(ex_rd),                      // 目的寄存器地址
    .ex_mem_write(ex_mem_write),        // 存储器写使能
    .ex_mem_read(ex_mem_read),          // 存储器读使能
    .ex_reg_write(ex_reg_write),        // 寄存器写使能
    .ex_wbmux(ex_wbmux),                // 写回选择
    .ex_pc_plus_4(ex_pc_plus_4),        // PC+4
    // ---- MEM 阶段输出 ----
    .ex_mem_alu_result(mem_alu_result),
    .ex_mem_mem_write_data(mem_write_data),
    .ex_mem_rd(mem_rd),
    .ex_mem_mem_write(mem_mem_write),
    .ex_mem_mem_read(mem_mem_read),
    .ex_mem_reg_write(mem_reg_write),
    .ex_mem_wbmux(mem_wbmux),
    .ex_mem_pc_plus_4(mem_pc_plus_4)
  );


  // ##########################################################################
  //  MEM 阶段（Memory Access — 访存）
  //
  //  核心功能：
  //    1. 总线接口映射：将内部信号连接到 SoC 总线端口
  //    2. load_unit：处理读回数据（字节/半字/字对齐、符号/零扩展）
  //    3. store_unit：处理写入数据（字节/半字/字对齐、写掩码生成）
  //    4. funct3 追踪：两级移位寄存器传递 funct3 从 ID 到 MEM
  //
  //  【总线接口设计】
  //
  //  内部信号 → 外部端口直连（Cache 旁路模式）：
  //    mem_addr  = mem_alu_result   // 存储器地址 = ALU 结果
  //    mem_wdata = mem_write_data   // 写数据 = 前递后的 rs2
  //    mem_we    = mem_mem_write    // 写使能
  //    mem_re    = mem_mem_read     // 读使能
  //    mem_rdata_int = mem_rdata    // 读回数据来自外部总线
  //
  //  总线就绪信号（dmem_ready）处理：
  //    若既不是读也不是写（空闲），则视为就绪（避免停顿）。
  //    否则等待 mem_ready = 1。
  //
  //  【funct3 追踪设计】
  //
  //  由于 funct3 没有包含在标准流水线寄存器中（只有 funct3[2:0] 在 ID/EX
  //  中作为 ex_funct3 传递，但未进一步传递到 EX/MEM），需要独立的追踪机制：
  //
  //    Cycle N:   id_funct3 → id_ex_funct3（锁存 ID 的 funct3）
  //    Cycle N+1: id_ex_funct3 → ex_mem_funct3（传递到 EX/MEM 阶段）
  //
  //  ex_mem_funct3 在 MEM 阶段被 load_unit 和 store_unit 使用：
  //    - load_unit：决定字节/半字/字 加载方式，符号/零扩展
  //    - store_unit：决定字节/半字/字 存储方式，生成写掩码
  //
  //  冲刷处理（hazard_ctrl_mux_sel=1）：
  //    两级寄存器清零，防止分支预测错误后残留的 funct3 影响后续指令。
  // ##########################################################################

  // 内部存储器接口信号 → 直接映射到外部端口（旁路 Cache）
  assign mem_addr_int  = mem_alu_result;   // 地址 = ALU 结果（load/store 的有效地址）
  assign mem_wdata_int = mem_write_data;    // 写数据 = 前递后的 rs2 数据
  assign mem_we_int    = mem_mem_write;     // 写使能
  assign mem_re_int    = mem_mem_read;      // 读使能

  // ★ BYPASS D-Cache：内部信号直连外部总线端口
  assign mem_addr  = mem_addr_int;
  assign mem_wdata = mem_wdata_int;
  assign mem_we    = mem_we_int;
  assign mem_re    = mem_re_int;
  assign mem_rdata_int = mem_rdata;         // 读回数据来自外部总线

  // ★【Bug 修复】字节写使能输出
  //   原 TOP_levelSOc.v 中 BRAM.be 硬编码为 4'b1111，导致 SB/SH 指令破坏相邻字节。
  //   修复后 store_unit 的 store_mask 通过 mem_be 输出到顶层，BRAM 正确实现字节写入。
  //   仅在写操作时输出有效掩码，非写操作时输出 4'b0000（安全值）。
  assign mem_be = mem_mem_write ? store_mask : 4'b0000;

  // 数据存储器就绪信号
  // 若无读写请求（空闲周期），视为就绪；否则等待总线 mem_ready 握手
  wire dmem_ready;
  assign dmem_ready = (!mem_we_int && !mem_re_int) || mem_ready;

  // --------------------------------------------------------------------------
  //  ★ 两级移位寄存器：funct3 追踪（ID → ID/EX → EX/MEM）
  //
  //  设计必要性：
  //    主流水线寄存器（id_ex_stage, ex_mem_stage）传递了 ex_funct3，
  //    但该字段在 ex_mem_stage 中未连接输出（mem 阶段没有直接的 funct3 信号）。
  //    因此需要这两级寄存器将 funct3 从 ID 阶段传递到 MEM 的 load_unit/store_unit。
  //
  //  时序：
  //    Cycle N:   id_funct3（当前 ID 指令的 funct3）
  //               → 在时钟上升沿锁存到 id_ex_funct3
  //    Cycle N+1: id_ex_funct3 的值
  //               → 在时钟上升沿锁存到 ex_mem_funct3
  //    Cycle N+2: ex_mem_funct3 被 load_unit / store_unit 使用
  //
  //  冲刷：hazard_ctrl_mux_sel=1 时两级寄存器均清零，
  //        防止分支预测错误后残留的 funct3 影响后续正确指令
  //  停顿：pipeline_stall=1 时寄存器保持当前值
  // --------------------------------------------------------------------------
  always @(posedge clk or posedge reset)
    if (reset) begin
      id_ex_funct3  <= 3'b000;
      ex_mem_funct3 <= 3'b000;
    end else if (hazard_ctrl_mux_sel) begin
      // 控制冒险冲刷：清零两条流水线级的 funct3 追踪
      id_ex_funct3  <= 3'b000;
      ex_mem_funct3 <= 3'b000;
    end else if (!pipeline_stall) begin
      // 正常流水：数据沿流水线向前推进
      id_ex_funct3  <= id_funct3;        // Cycle N: ID → ID/EX
      ex_mem_funct3 <= id_ex_funct3;     // Cycle N+1: ID/EX → EX/MEM
    end

  // --------------------------------------------------------------------------
  //  load_unit：加载单元
  //
  //  功能：根据 funct3（load_funct3_in）和地址低 2 位（addr_low），
  //        从 32 位存储器读回数据中提取正确的字节/半字/字，并进行符号/零扩展。
  //
  //  funct3 编码：
  //    3'b000: LB  — 字节，符号扩展
  //    3'b001: LH  — 半字，符号扩展
  //    3'b010: LW  — 字（32位直通）
  //    3'b100: LBU — 字节，零扩展
  //    3'b101: LHU — 半字，零扩展
  //
  //  地址低 2 位（addr_low = mem_alu_result[1:0]）：
  //    用于非对齐访问的字节移位（RISC-V 允许非对齐访问，由硬件处理）
  // --------------------------------------------------------------------------
  load_unit u_load (
    .load_funct3_in(ex_mem_funct3),     // 来自两级追踪的 funct3
    .load_in(mem_mem_read),             // load 使能（=1 表示当前是 load 指令）
    .data_in(mem_rdata_int),            // 32 位原始读回数据
    .addr_low(mem_alu_result[1:0]),     // 地址低 2 位（字节偏移）
    .load_output(load_result),          // 处理后的加载数据
    .load_mux(load_mux_sel)             // 输出选择（保留，当前未使用）
  );

  // --------------------------------------------------------------------------
  //  store_unit：存储单元
  //
  //  功能：根据 funct3 和地址低 2 位，将 32 位 rs2 数据对齐到正确的字节位置，
  //        并生成字节写掩码（wb_mask_out）。
  //
  //  funct3[1:0] 编码：
  //    2'b00: SB — 字节存储
  //    2'b01: SH — 半字存储
  //    2'b10: SW — 字存储
  //
  //  输出：
  //    - rs2_out：对齐后的存储数据（重复/移位 rs2 到正确的字节通道）
  //    - wb_mask_out：4-bit 字节写掩码（每位对应 1 字节，1=写入）
  //    - wr_req_out：写请求信号（用于 Cache 写分配，当前未使用）
  // --------------------------------------------------------------------------
  store_unit u_store (
    .funct3_in(ex_mem_funct3[1:0]),     // funct3 低 2 位（SB/SH/SW）
    .addr_low(mem_alu_result[1:0]),     // 地址低 2 位（字节偏移）
    .rs2_in(mem_write_data),            // 原始 rs2 数据（来自 EX/MEM 寄存器）
    .mem_write(mem_mem_write),          // 存储写使能
    .rs2_out(store_data_out),           // 对齐后的存储数据
    .wb_mask_out(store_mask),           // 字节写掩码
    .wr_req_out(store_req)              // 写请求输出（保留）
  );


  // ##########################################################################
  //  MEM/WB 流水线寄存器
  //
  //  功能：锁存 MEM 阶段的结果，传递给 WB 阶段
  //
  //  传递的数据包括三路可能的写回来源：
  //    1. mem_alu_result → wb_alu_result（ALU 或 M 单元结果）
  //    2. load_result → wb_load_data（存储器加载数据）
  //    3. mem_pc_plus_4 → wb_pc_plus_4（JAL/JALR 返回地址）
  //
  //  flush 固定为 0：MEM/WB 阶段不需要冲刷
  // ##########################################################################

  mem_wb_stage u_mem_wb (
    .clk(clk),
    .reset(reset),
    .stall(pipeline_stall),             // 流水线停顿时保持
    .flush(1'b0),                       // MEM/WB 不冲刷
    // ---- MEM 阶段输入 ----
    .mem_alu_result(mem_alu_result),
    .mem_load_data(load_result),        // 经 load_unit 处理后的数据
    .mem_pc_plus_4(mem_pc_plus_4),
    .mem_rd_addr(mem_rd),
    .mem_reg_write(mem_reg_write),
    .mem_wbmux(mem_wbmux),
    // ---- WB 阶段输出 ----
    .wb_alu_result(wb_alu_result),
    .wb_load_data(wb_load_data),
    .wb_pc_plus_4(wb_pc_plus_4),
    .wb_rd_addr(wb_write_addr),
    .wb_reg_write(wb_write_enable),
    .wb_wbmux(wb_wbmux)
  );


  // ##########################################################################
  //  WB 阶段（Write Back — 写回）
  //
  //  纯组合逻辑：根据 wb_wbmux 选择最终的写回数据
  //
  //  wb_wbmux 编码（在 ID 阶段由 control_unit 产生，沿流水线传递）：
  //    2'b00（默认）→ wb_alu_result：算术/逻辑/移位/乘除法结果
  //    2'b01       → wb_load_data：存储器加载数据（LB/LH/LW/LBU/LHU）
  //    2'b10       → wb_pc_plus_4：JAL/JALR 返回地址（PC+4）
  //    2'b11       → 保留（当前未使用）
  //
  //  写回数据 + wb_write_addr + wb_write_enable → reg_file 的写端口
  //  写操作在下一个时钟上升沿生效。
  //
  //  设计说明：WB 阶段没有流水线寄存器，因为 reg_file 的写操作本身就是
  //  时序逻辑（在 clk 上升沿写入）。WB 阶段的组合逻辑 MUX 在同一个时钟
  //  周期内完成，不增加额外的周期延迟。
  // ##########################################################################

  assign wb_write_data = (wb_wbmux == 2'b01) ? wb_load_data  :   // MEM 加载数据
                         (wb_wbmux == 2'b10) ? wb_pc_plus_4  :   // JAL/JALR 返回地址
                                               wb_alu_result;     // ALU/M 结果（默认）

endmodule
