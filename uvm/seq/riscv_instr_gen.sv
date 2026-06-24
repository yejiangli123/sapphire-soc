// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  riscv_instr_gen.sv — 受约束随机 RISC-V 指令生成器                              ║
// ║  ★ 可配置指令类型混合比例 + 9 种指令格式自动汇编 + 寄存器依赖追踪               ║
// ╚══════════════════════════════════════════════════════════════════════════════════╝
//
// 【模块定位 — 随机指令生成器（Instruction Random Generator）】
//   本模块用于生成随机但合法的 RISC-V 指令序列，并将它们通过 AXI 写入 SoC 的指令内存。
//   这是"随机验证"（Random Verification）的核心组件——不依赖人工编写固件，
//   而是自动生成大量随机指令，以发现边界情况 bug。
//
//   与传统定向测试的区别：
//     定向测试（firmware_m_smoke_test）：人工编写的 5~20 条指令，验证已知场景
//     随机测试（本模块）：自动生成 50~500 条随机指令，探索未知场景
//
// 【两阶段工作流程】
//   Stage 1 — 指令生成（gen_alu / gen_br / gen_mem / gen_jmp）：
//     随机选择指令类型 → 随机寄存器/立即数 → 创建 riscv_instruction 对象
//
//   Stage 2 — 指令汇编（assemble()）：
//     将 riscv_instruction 对象 → 32-bit RISC-V 机器码 → 写入 mc 字段
//
//   Stage 3 — 写入 SoC（body() 中的 AXI 写）：
//     将 mc 通过 AXI 写入 BRAM（从地址 0x40000000 开始，递增 4 字节）
//
// 【指令类型混合比例】
//   alu_pct(40%) + br_pct(20%) + mem_pct(25%) + jmp_pct(15%) = 100%
//   可配置——例如增加 br_pct 到 30% 来重点测试分支预测，减少 alu_pct
//
//   ★ 为什么不是纯均匀随机？
//     真实程序的指令分布是不均匀的（ALU 指令占比最高，跳转最少）。
//     按比例混合能产生更接近真实工作负载的指令序列。
//
// 【寄存器依赖追踪（rv_v[] / rv[]）】
//   rv_v[i] = 1：寄存器 i 有有效值
//   rv[i]     ：寄存器 i 的当前值
//   当前实现简化——只标记寄存器为"有效"，但不保证"值的正确性"。
//
//   ★ 局限性：随机值之间没有逻辑关系——MUL 的结果不会在后续 ADD 中使用。
//     这是随机验证的固有限制——发现的 bug 往往是"组合式"而非"逻辑式"的。
//     如需发现逻辑相关的 bug，需要基于约束求解的指令生成。
//
// 【RISC-V 指令立即数格式速查】
//   I-type:     12-bit 符号扩展（imm[11:0] = inst[31:20]）
//   S-type:     12-bit 拼接（imm[11:5] = inst[31:25], imm[4:0] = inst[11:7]）
//   B-type:     13-bit 拼接 + LSB=0（imm[12|10:5|4:1|11] = inst[31|30:25|11:8|7]）
//   U-type:     20-bit 截断（imm[31:12] = inst[31:12]，低 12 位 = 0）
//   J-type:     21-bit 拼接 + LSB=0（imm[20|10:1|11|19:12] = inst[31|19:12|20|30:21]）
//
// 【assemble() 函数 — 手动汇编器】
//   因为 UVM 环境中没有外部的 RISC-V 汇编器，所以直接在 SystemVerilog 中
//   按 RISC-V 指令格式手动拼接各字段。每个 case 对应一种指令格式。
//
//   ★ 为什么不直接用交叉编译工具链？
//     交叉编译工具链（riscv-gcc）是"软件级"的——从 C 编译到 hex，流程重。
//     在仿真环境中，用一个 SV 函数完成"随机字段 → 机器码"更高效——
//     100 行代码替代了整个编译工具链。
//
// 【面试要点】
//   Q: 你怎么做随机验证？
//   A: 用 SystemVerilog 的 constraint random 生成随机 RISC-V 指令，
//      通过 AXI 写入 BRAM，让 CPU 执行。同时 regmodel/scoreboard 比对结果。
//      可以配置指令类型的混合比例来针对性地测试某个模块。
// ═══════════════════════════════════════════════════════════════════════════════

// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  riscv_instruction — 单条 RISC-V 指令的抽象表示                                 ║
// ║  ★ 随机字段 + 手动汇编 + 9 种指令格式                                          ║
// ╚══════════════════════════════════════════════════════════════════════════════╝
class riscv_instruction extends uvm_sequence_item;
    `uvm_object_utils(riscv_instruction)

    // ★ 指令类型枚举 — 覆盖 RV32I 全部 9 种格式
    typedef enum {
        R_TYPE,      // 寄存器-寄存器运算（ADD/SUB/SLL/...）
        I_TYPE,      // 立即数算逻（ADDI/XORI/ORI/ANDI/...）
        I_LOAD,      // Load（LB/LH/LW/LBU/LHU）
        S_TYPE,      // Store（SB/SH/SW）
        B_TYPE,      // Branch（BEQ/BNE/BLT/BGE/BLTU/BGEU）
        JAL,         // Jump And Link
        JALR,        // Jump And Link Register
        LUI,         // Load Upper Immediate
        AUIPC        // Add Upper Immediate to PC
    } kind_e;

    // ── 随机字段 ──
    rand kind_e    kind;          // ★ 指令类型
    rand bit [4:0] rd, rs1, rs2;  // ★ 寄存器地址（5-bit，32 个寄存器）
    rand bit [2:0] f3;            // funct3（细分操作类型）
    rand bit [6:0] f7;            // funct7（R-type 中区分 ADD/SUB、SRL/SRA）
    rand bit [31:0] imm;          // 立即数（各格式截取不同位宽）

    // ── 输出字段 ──
    bit [31:0] mc;                // ★ assembled Machine Code（32-bit RISC-V 指令）

    // ═══════════════════════════════════════════════════════════════════════
    //  约束
    // ═══════════════════════════════════════════════════════════════════════

    // ★ rd != 0：目的寄存器不能是 x0（x0 硬连线为 0，写回无意义）
    constraint c_rd { rd != 0; }

    // ★ funct7 合法值：0x00（ADD/SLL/SLT/SLTU/XOR/SRL/OR/AND）或 0x20（SUB/SRA）
    //   M 扩展的 funct7=0x01 暂不支持（可扩展）
    constraint c_f7 { f7 inside { 7'h00, 7'h20 }; }

    // ═══════════════════════════════════════════════════════════════════════
    //  assemble() — 手动汇编器（SystemVerilog 版）
    //
    //  将随机生成的字段（rd/rs1/rs2/f3/f7/imm）按 RISC-V 指令格式
    //  拼接为 32-bit 机器码。
    //
    //  ★ RISC-V 指令格式标准：
    //    R-type:  funct7[31:25] | rs2[24:20] | rs1[19:15] | funct3[14:12] | rd[11:7] | opcode[6:0]
    //    I-type:  imm[31:20] | rs1[19:15] | funct3[14:12] | rd[11:7] | opcode[6:0]
    //    S-type:  imm[11:5][31:25] | rs2[24:20] | rs1[19:15] | funct3[14:12] | imm[4:0][11:7] | opcode[6:0]
    //    B-type:  imm[12|10:5][31|30:25] | rs2[24:20] | rs1[19:15] | funct3[14:12] | imm[4:1|11][11:8|7] | opcode[6:0]
    //    U-type:  imm[31:12][31:12] | rd[11:7] | opcode[6:0]
    //    J-type:  imm[20|10:1|11|19:12][31|30:21|20|19:12] | rd[11:7] | opcode[6:0]
    //
    //  ★ 注意：S/B/J 型指令的立即数在指令中是**非连续拼接**的！
    //    例如 B-type 的 bit 布局是 RISC-V 规范特意设计的——
    //    为了最小化硬件复杂度（所有格式中 rs1/rs2 位置相同）。
    //    这导致 assemble() 中的 bit 操作看起来很复杂——但它完全遵循规范。
    // ═══════════════════════════════════════════════════════════════════════
    function void assemble();
        case (kind)
            // ── R-type (0110011)：{funct7, rs2, rs1, funct3, rd, opcode} ──
            R_TYPE: mc = {f7, rs2, rs1, f3, rd, 7'b0110011};

            // ── I-type ALU (0010011)：{imm[11:0], rs1, funct3, rd, opcode} ──
            I_TYPE: mc = {imm[11:0], rs1, f3, rd, 7'b0010011};

            // ── I-type Load (0000011)：同 I-type，opcode 不同 ──
            I_LOAD: mc = {imm[11:0], rs1, f3, rd, 7'b0000011};

            // ── S-type (0100011)：{imm[11:5], rs2, rs1, funct3, imm[4:0], opcode} ──
            S_TYPE: mc = {imm[11:5], rs2, rs1, f3, imm[4:0], 7'b0100011};

            // ── B-type (1100011)：{imm[12|10:5], rs2, rs1, funct3, imm[4:1|11], opcode} ──
            //   ★ B-type 的立即数布局最复杂——每个 bit 在指令中位置都不连续
            //     imm[12]    → inst[31]
            //     imm[10:5]  → inst[30:25]
            //     imm[4:1]   → inst[11:8]
            //     imm[11]    → inst[7]
            //     imm[0] 保持隐式的 0（不需要存储）
            B_TYPE: mc = {imm[12], imm[10:5], rs2, rs1, f3, imm[4:1], imm[11], 7'b1100011};

            // ── J-type (1101111)：{imm[20|10:1|11|19:12], rd, opcode} ──
            JAL: mc = {imm[20], imm[10:1], imm[11], imm[19:12], rd, 7'b1101111};

            // ── I-type JALR (1100111)：{imm[11:0], rs1, 000, rd, opcode} ──
            //   ★ 注意 JALR 的 funct3 固定为 000（不可变）
            JALR: mc = {imm[11:0], rs1, 3'b000, rd, 7'b1100111};

            // ── U-type LUI (0110111)：{imm[31:12], rd, opcode} ──
            //   ★ imm 的高 20 位直接放入 inst[31:12]，低 12 位硬件补 0
            LUI: mc = {imm[31:12], rd, 7'b0110111};

            // ── U-type AUIPC (0010111)：同 LUI，opcode 不同 ──
            AUIPC: mc = {imm[31:12], rd, 7'b0010111};
        endcase
    endfunction

endclass


// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  riscv_rand_instr_seq — 随机指令生成 Sequence                                 ║
// ║  ★ 按比例混合 ALU/Branch/LoadStore/Jump → 汇编 → AXI 写入 → CPU 执行          ║
// ╚══════════════════════════════════════════════════════════════════════════════╝
class riscv_rand_instr_seq extends uvm_sequence#(axi_transaction);
    `uvm_object_utils(riscv_rand_instr_seq)

    // ═══════════════════════════════════════════════════════════════════════
    //  配置参数（可随机化）
    // ═══════════════════════════════════════════════════════════════════════
    rand int N        = 50;           // ★ 生成的指令总数（10~500 可配）
    rand int alu_pct  = 40;           // ALU 类比例（R-type + I-type ALU + LUI/AUIPC）
    rand int br_pct   = 20;           // Branch 类比例
    rand int mem_pct  = 25;           // Load/Store 类比例
    rand int jmp_pct  = 15;           // Jump 类比例（JAL + JALR）

    // ★ 约束：四种比例之和必须为 100（否则有未覆盖的区间）
    constraint c_sum  { alu_pct + br_pct + mem_pct + jmp_pct == 100; }
    constraint c_N    { N inside { [10:500] }; }

    // ═══════════════════════════════════════════════════════════════════════
    //  寄存器状态追踪
    //
    //  rv[i]    ：寄存器 i 的当前值（32-bit）
    //  rv_v[i]  ：寄存器 i 是否有有效值（1 = 已被写入过）
    //
    //  ★ 当前实现只标记寄存器为"有效"并赋一个随机值。
    //    不模拟真实的 ALU 计算——这意味着生成的指令序列中
    //    数据依赖关系是"随机"的，不能保证 MUL(5,3)=15 被后续 ADD 使用。
    //
    //  ★ 改进方向（作为后续工作）：
    //    集成 riscv_golden_model.execute() 来实时更新寄存器真实值——
    //    这样生成的指令序列就具有真实的数据依赖关系。
    // ═══════════════════════════════════════════════════════════════════════
    bit [31:0] rv[32];                 // ★ 寄存器值
    bit        rv_v[32];               // ★ 寄存器有效标志
    bit [31:0] cpc = 32'h40000000;     // ★ Current PC（当前 AXI 写入地址）

    // ═══════════════════════════════════════════════════════════════════════
    //  body() — 主生成循环
    //
    //  流程：
    //    1. 初始化：x0 = 0（始终有效），其他寄存器标记无效
    //    2. 循环 N 次：
    //       a. 随机选择指令类型（根据比例参数）
    //       b. gen_alu / gen_br / gen_mem / gen_jmp → 创建 riscv_instruction
    //       c. assemble() → 生成 32-bit 机器码
    //       d. AXI 写入 → 机器码写入 BRAM（地址 = cpc，递增 4 字节）
    // ═══════════════════════════════════════════════════════════════════════
    task body();
        // ── 初始化：x0=0（硬连线），其余寄存器无效 ──
        rv_v[0] = 1;                    // ★ x0 始终"有效"（虽然是常量 0）
        rv[0]   = 0;
        for (int i = 1; i < 32; i++)
            rv_v[i] = 0;                // ★ 其他寄存器初始无有效值

        repeat (N) begin
            riscv_instruction instr = riscv_instruction::type_id::create("instr");
            int r = $urandom_range(0, 99);   // ★ 0~99 随机数用于比例分配

            // ── 按配置比例选择指令类型 ──
            //   区间分配：
            //     [0, alu_pct)            → ALU 类
            //     [alu_pct, alu_pct+br_pct) → Branch
            //     [alu_pct+br_pct, alu_pct+br_pct+mem_pct) → Load/Store
            //     [alu_pct+br_pct+mem_pct, 100) → Jump
            if (r < alu_pct)
                gen_alu(instr);
            else if (r < alu_pct + br_pct)
                gen_br(instr);
            else if (r < alu_pct + br_pct + mem_pct)
                gen_mem(instr);
            else
                gen_jmp(instr);

            // ★ 汇编：字段 → 32-bit 机器码
            instr.assemble();

            // ★ AXI 写入指令内存（从 0x40000000 开始）
            axi_transaction wr = new;
            start_item(wr);
            wr.randomize() with {
                addr == cpc;             // ★ 当前指令地址
                kind == 1;               // 写
                data == instr.mc;         // ★ 指令机器码
            };
            finish_item(wr);
            cpc += 4;                    // ★ 下一条指令地址
        end

        `uvm_info("INSTRGEN",
            $sformatf("Generated %0d instructions", N), UVM_LOW)
    endtask

    // ═══════════════════════════════════════════════════════════════════════
    //  gen_alu() — 生成 ALU 类指令
    //
    //  ★ 4 种子类型（随机）：
    //    0: R-type ADD   — x[rd] = x[rs1] + x[rs2]  (f7=0x00, f3=000)
    //    1: R-type XOR   — x[rd] = x[rs1] ^ x[rs2]  (f7=0x00, f3=100)
    //    2: I-type ADDI  — x[rd] = x[rs1] + imm     (imm: -2048~2047)
    //    3: LUI          — x[rd] = imm << 12         (imm: 20-bit 对齐)
    //
    //   寄存器选择：随机（1~31，排除 x0）
    //   ★ 写入后标记寄存器为有效 + 赋随机值
    // ═══════════════════════════════════════════════════════════════════════
    function void gen_alu(ref riscv_instruction i);
        i.rs1 = $urandom_range(1, 31);    // ★ 源寄存器 1（排除 x0）
        i.rs2 = $urandom_range(1, 31);    // ★ 源寄存器 2（排除 x0）
        i.rd  = $urandom_range(1, 31);    // ★ 目的寄存器（排除 x0）
        case ($urandom_range(0, 3))
            0: begin
                i.kind = R_TYPE;           // ADD：f7=0, f3=0
                i.f3   = 0;
                i.f7   = 0;
            end
            1: begin
                i.kind = R_TYPE;           // XOR：f7=0, f3=4
                i.f3   = 4;
                i.f7   = 0;
            end
            2: begin
                i.kind = I_TYPE;           // ADDI：12-bit 符号扩展立即数
                i.f3   = 0;
                i.imm  = $urandom_range(-2048, 2047);
            end
            3: begin
                i.kind = LUI;              // LUI：20-bit 立即数
                i.imm  = $urandom_range(0, 1048575) << 12;
                // ★ << 12：LUI 的立即数在指令高 20 位，低 12 位 = 0
            end
        endcase
        rv_v[i.rd] = 1;                   // ★ 标记寄存器为有效
        rv[i.rd]   = $urandom();           // ★ 赋随机值（简化：不模拟真实 ALU 计算）
    endfunction

    // ═══════════════════════════════════════════════════════════════════════
    //  gen_br() — 生成 Branch 指令
    //
    //  ★ B-type 指令的立即数是 13-bit 有符号值（LSB=0，范围 -4096~4094）
    //    funct3 随机（0~5），覆盖 BEQ/BNE/BLT/BGE/BLTU/BGEU
    //    ★ 没有检查 rs1/rs2 的依赖关系——可能分支不成立
    // ═══════════════════════════════════════════════════════════════════════
    function void gen_br(ref riscv_instruction i);
        i.kind = B_TYPE;
        i.f3   = $urandom_range(0, 5);     // ★ BEQ~BGEU
        i.rs1  = $urandom_range(1, 31);
        i.rs2  = $urandom_range(1, 31);
        i.imm  = ($urandom_range(-4096, 4095)) << 1;
        // ★ << 1：B-type 立即数的 LSB 固定为 0（指令对齐），
        //   实际偏移 = imm（字节偏移）
    endfunction

    // ═══════════════════════════════════════════════════════════════════════
    //  gen_mem() — 生成 Load/Store 指令
    //
    //  ★ 50% 概率 Load（LB/LH/LW），50% 概率 Store（SB/SH/SW）
    //    funct3=2 → LW/SW（4 字节）
    //    立即数：0~1020（字对齐的偏移）
    // ═══════════════════════════════════════════════════════════════════════
    function void gen_mem(ref riscv_instruction i);
        if ($urandom_range(0, 1)) begin
            // ── Load ──
            i.kind = I_LOAD;
            i.f3   = 2;                    // ★ LW（load word）
            i.rd   = $urandom_range(1, 31);
        end
        else begin
            // ── Store ──
            i.kind = S_TYPE;
            i.f3   = 2;                    // ★ SW（store word）
            i.rs2  = $urandom_range(1, 31);
        end
        i.rs1 = $urandom_range(1, 31);     // ★ 基址寄存器
        i.imm = $urandom_range(0, 1020);   // ★ 偏移（0~1020，4 字节对齐）
    endfunction

    // ═══════════════════════════════════════════════════════════════════════
    //  gen_jmp() — 生成 JAL/JALR 指令
    //
    //  ★ 50% 概率 JAL（PC-relative 跳转），50% 概率 JALR（寄存器间接跳转）
    //    JAL 立即数：21-bit 有符号（-524288~524286，LSB=0）
    //    JALR 立即数：12-bit 有符号（-2048~2047）
    // ═══════════════════════════════════════════════════════════════════════
    function void gen_jmp(ref riscv_instruction i);
        if ($urandom_range(0, 1)) begin
            // ── JAL：PC += imm（21-bit 有符号，LSB=0）──
            i.kind = JAL;
            i.rd   = $urandom_range(1, 31);   // ★ 链接寄存器（存储返回地址）
            i.imm  = ($urandom_range(-524288, 524287)) << 1;
        end
        else begin
            // ── JALR：PC = (rs1 + imm) & ~1 ──
            i.kind = JALR;
            i.rd   = $urandom_range(1, 31);   // ★ 链接寄存器
            i.rs1  = $urandom_range(1, 31);   // ★ 基址寄存器
            i.imm  = $urandom_range(-2048, 2047);
        end
    endfunction

endclass
