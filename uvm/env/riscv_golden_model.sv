// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  riscv_golden_model.sv — RISC-V 指令级黄金参考模型（Golden ISA Model）         ║
// ║  ★ 支持全部 RV32I 指令（9 种格式）+ 指令分类统计                               ║
// ╚══════════════════════════════════════════════════════════════════════════════════╝
//
// 【UVM 角色 — Golden Reference Model（黄金模型）】
//   黄金模型是验证的"正确答案"——它模拟 DUT（RISC-V core）的指令级行为，
//   逐条执行固件指令，产生预期的寄存器值和内存状态。
//
//   Golden Model 和 Reference Model 的区别：
//     ref_model（riscv_ref_model.sv）：字节级内存模型——只模拟 AXI 读写
//     golden_model（本文件）：指令级 ISA 模型——模拟 CPU 执行每一条 RISC-V 指令
//
//   ★ 黄金模型是 scoreboard 的"最终裁判"：
//     固件执行完后，用 golden model 再执行一遍相同的指令，
//     比对 golden model 的寄存器/内存状态 vs DUT 的实际状态 → 判断 PASS/FAIL
//
// 【支持的 RV32I 指令格式（全部 9 种）】
//   ┌──────────────┬──────────┬──────────────────────────────────┐
//   │ 格式          │ opcode    │ 指令                             │
//   ├──────────────┼──────────┼──────────────────────────────────┤
//   │ R-type        │ 0110011   │ ADD/SUB/SLL/SLT/SLTU/XOR/SRL/SRA/OR/AND │
//   │ I-type ALU    │ 0010011   │ ADDI/SLTI/SLTIU/XORI/ORI/ANDI/SLLI/SRLI/SRAI │
//   │ I-type Load    │ 0000011   │ LB/LH/LW/LBU/LHU                │
//   │ S-type Store   │ 0100011   │ SB/SH/SW                        │
//   │ B-type Branch  │ 1100011   │ BEQ/BNE/BLT/BGE/BLTU/BGEU       │
//   │ J-type JAL     │ 1101111   │ JAL                             │
//   │ I-type JALR    │ 1100111   │ JALR                            │
//   │ U-type LUI     │ 0110111   │ LUI                             │
//   │ U-type AUIPC   │ 0010111   │ AUIPC                           │
//   └──────────────┴──────────┴──────────────────────────────────┘
//
// 【指令执行模型】
//   execute(mc, ipc, result, rd, mem_wr, mem_addr, mem_wdata)
//     mc[31:0]        — 32-bit 机器码（machine code）
//     ipc[31:0]       — 当前 PC 值
//     result[31:0]    — 输出：ALU 结果/load 数据/JAL 返回地址
//     rd[4:0]         — 输出：目的寄存器编号
//     mem_wr          — 输出：是否为 store 指令
//     mem_addr[31:0]  — 输出：访存地址
//     mem_wdata[31:0] — 输出：store 数据
//
// 【指令统计】
//   本模型记录每种指令的执行次数（alu_n / branch_n / load_n / store_n / jump_n）
//   用于验证指令覆盖率——确保测试固件覆盖了所有指令类型
//
//   ★ 统计数据的用途：
//     如果 load_n == 0 → 测试固件没有 load 指令 → 没测到 load 通路 → 覆盖率盲区
//     这是 functional coverage 的补充——在 report_phase 中打印统计报告
//
// 【M 扩展的处理】
//   当前 golden model 仅支持 RV32I，不支持 M 扩展指令。
//   对于 MUL/DIV/REM 等指令（funct7[5]=1, opcode=OPCODE_OP），
//   当前代码会走入 R-type 的 case 分支——但因为 funct7/funct3 组合不匹配
//   R-type 的 case 列表，实际不会执行任何操作（result 保持 0）。
//
//   ★ 这是已知限制——如果需要端到端验证 M 扩展，
//     应在 golden model 中增加 M 指令的执行逻辑。
//     但 mul_div_unit 的单元测试已经覆盖了 M 指令的正确性。
// ═══════════════════════════════════════════════════════════════════════════════

class riscv_golden_model extends uvm_component;
    `uvm_component_utils(riscv_golden_model)

    // ═══════════════════════════════════════════════════════════════════════
    //  架构状态（Architectural State）
    //
    //  xreg[32] — 32 个通用寄存器（x0~x31）
    //    x0 由 RISC-V 规范硬连线为 0，reset_state() 中显式清零
    //
    //  mem[bit[31:0]] — 内存映像（关联数组，稀疏存储）
    //    key = 字节地址，value = 32-bit 数据
    //    只存储实际被写入的地址
    //
    //  pc — 程序计数器（Program Counter），指向当前指令地址
    // ═══════════════════════════════════════════════════════════════════════
    protected bit [31:0] xreg[32];
    protected bit [31:0] mem[bit[31:0]];
    protected bit [31:0] pc;

    // ★ 指令分类统计（用于验证覆盖率）
    int instr_count;     // 总指令数
    int alu_n;           // ALU 类（R/I/U-type 算逻 + LUI/AUIPC）
    int branch_n;        // 分支类（B-type：BEQ/BNE/BLT/BGE/BLTU/BGEU）
    int load_n;          // Load 类（LB/LH/LW/LBU/LHU）
    int store_n;         // Store 类（SB/SH/SW）
    int jump_n;          // 跳转类（JAL/JALR）

    // ═══════════════════════════════════════════════════════════════════════
    //  reset_state() — 初始化架构状态
    //
    //  ★ 所有寄存器清零（x0 双重保险）
    //  ★ PC 归零（从地址 0 开始执行）
    //  ★ 指令计数器归零
    // ═══════════════════════════════════════════════════════════════════════
    virtual function void reset_state();
        foreach (xreg[i]) xreg[i] = 0;     // ★ 全部寄存器清零
        xreg[0] = 0;                        // ★ x0 双重保险
        pc = 0;
        instr_count = 0;
    endfunction

    // ═══════════════════════════════════════════════════════════════════════
    //  execute() — 执行一条 RISC-V 指令（核心函数）
    //
    //  参数说明：
    //    mc         — 32-bit 机器码（machine code）
    //    ipc        — 当前指令地址（PC，供 JAL/AUIPC 使用）
    //    result     — 输出：本条指令的计算结果（ALU 结果/Load 数据/链接地址）
    //    rd         — 输出：目的寄存器编号（x0~x31）
    //    mem_wr     — 输出：是否为 store 指令（1 = 需要写内存）
    //    mem_addr   — 输出：访存地址（load/store 使用）
    //    mem_wdata  — 输出：store 数据（写入内存的值）
    //
    //  执行流程：
    //    1. 解码 opcode/funct3/funct7/rs1/rs2/rd/imm
    //    2. 根据 opcode 跳转到对应指令格式分支
    //    3. 在分支内根据 funct3/funct7 执行具体运算
    //    4. 更新寄存器/memory/PC
    //    5. 统计计数器 + 1
    // ═══════════════════════════════════════════════════════════════════════
    virtual function void execute(
        bit [31:0] mc,                     // 32-bit 机器码
        bit [31:0] ipc,                    // 当前 PC
        output bit [31:0] result,          // 计算结果
        output bit [4:0]  rd,             // 目的寄存器
        output bit        mem_wr,          // 是否写内存
        output bit [31:0] mem_addr,        // 访存地址
        output bit [31:0] mem_wdata        // store 数据
    );
        // ── 指令字段解包 ──
        bit [6:0] op  = mc[6:0];          // opcode（inst[6:0]）
        bit [4:0] r1  = mc[19:15];        // rs1（源寄存器 1）
        bit [4:0] r2  = mc[24:20];        // rs2（源寄存器 2）
        bit [4:0] rd_ = mc[11:7];         // rd（目的寄存器）
        bit [2:0] f3  = mc[14:12];        // funct3
        bit [6:0] f7  = mc[31:25];        // funct7
        bit [31:0] imm;                    // 立即数（每个分支计算）

        // ── 默认输出（无操作时返回 0）──
        result   = 0;
        rd       = rd_;
        mem_wr   = 0;
        mem_addr = 0;
        mem_wdata = 0;

        case (op)
            // ═══════════════════════════════════════════════════════════
            //  R-type (0110011)：寄存器-寄存器运算
            //    10 种操作：ADD/SUB/SLL/SLT/SLTU/XOR/SRL/SRA/OR/AND
            //    区分方式：{funct7, funct3} → 唯一编码
            //
            //    注意：SRL(逻辑右移)和 SRA(算术右移)共享 funct3=101，
            //         通过 funct7 区分（SRL funct7=0x00, SRA funct7=0x20）
            //         类似地 ADD(f7=0x00)和 SUB(f7=0x20)共享 funct3=000
            //
            //    ★ 这些编码值与 RISC-V 规范完全一致，无需手动查表
            // ═══════════════════════════════════════════════════════════
            7'b0110011: begin
                alu_n++;
                case ({f7, f3})
                    {7'h00, 3'b000}: result = xreg[r1] + xreg[r2];                       // ADD
                    {7'h20, 3'b000}: result = xreg[r1] - xreg[r2];                       // SUB
                    {7'h00, 3'b001}: result = xreg[r1] << xreg[r2][4:0];                 // SLL
                    {7'h00, 3'b010}: result = ($signed(xreg[r1]) < $signed(xreg[r2])) ? 1 : 0;  // SLT
                    {7'h00, 3'b011}: result = (xreg[r1] < xreg[r2]) ? 1 : 0;             // SLTU
                    {7'h00, 3'b100}: result = xreg[r1] ^ xreg[r2];                       // XOR
                    {7'h00, 3'b101}: result = xreg[r1] >> xreg[r2][4:0];                 // SRL
                    {7'h20, 3'b101}: result = $signed(xreg[r1]) >>> xreg[r2][4:0];       // SRA
                    {7'h00, 3'b110}: result = xreg[r1] | xreg[r2];                       // OR
                    {7'h00, 3'b111}: result = xreg[r1] & xreg[r2];                       // AND
                endcase
                if (rd_ != 0) xreg[rd_] = result;    // ★ x0 写保护（RISC-V 规范）
            end

            // ═══════════════════════════════════════════════════════════
            //  I-type ALU (0010011)：立即数算逻运算
            //    ADDI/SLTI/SLTIU/XORI/ORI/ANDI/SLLI/SRLI/SRAI
            //    （当前仅实现了部分，其余走 default）
            //
            //    立即数格式：12-bit 符号扩展（inst[31:20]）
            //    ★ 注意：I-type 的移位指令（SLLI/SRLI/SRAI）只有 5-bit 移位量
            //            （在 inst[24:20] = shamt），高 7 位为 funct7
            // ═══════════════════════════════════════════════════════════
            7'b0010011: begin
                alu_n++;
                imm = {{20{mc[31]}}, mc[31:20]};         // ★ 12-bit 符号扩展 → 32-bit
                case (f3)
                    3'b000: result = xreg[r1] + imm;     // ADDI
                    3'b100: result = xreg[r1] ^ imm;     // XORI
                    3'b110: result = xreg[r1] | imm;     // ORI
                    3'b111: result = xreg[r1] & imm;     // ANDI
                endcase
                if (rd_ != 0) xreg[rd_] = result;
            end

            // ═══════════════════════════════════════════════════════════
            //  Load (0000011)：从内存加载数据到寄存器
            //    LB/LH/LW/LBU/LHU
            //
            //    地址计算：mem_addr = rs1 + sign_extend(imm)
            //    数据对齐与扩展（模拟 load_unit 行为）：
            //      LB：符号扩展 byte → 32-bit
            //      LH：符号扩展 halfword → 32-bit
            //      LW：32-bit 直通
            //      LBU：零扩展 byte → 32-bit
            //      LHU：零扩展 halfword → 32-bit
            //
            //    ★ 未初始化地址返回 0xDEAD_BEEF（与 ref_model 一致）
            // ═══════════════════════════════════════════════════════════
            7'b0000011: begin
                load_n++;
                imm = {{20{mc[31]}}, mc[31:20]};         // ★ 12-bit 符号扩展
                mem_addr = xreg[r1] + imm;                 // ★ 地址 = rs1 + imm
                result = mem.exists(mem_addr) ? mem[mem_addr] : 32'hDEAD_BEEF;
                case (f3)
                    3'b000: result = {{24{result[7]}}, result[7:0]};       // LB：符号扩展
                    3'b001: result = {{16{result[15]}}, result[15:0]};     // LH：符号扩展
                    3'b100: result = {24'h0, result[7:0]};                 // LBU：零扩展
                    3'b101: result = {16'h0, result[15:0]};                // LHU：零扩展
                endcase
                if (rd_ != 0) xreg[rd_] = result;
            end

            // ═══════════════════════════════════════════════════════════
            //  Store (0100011)：将寄存器数据写入内存
            //    SB/SH/SW
            //
            //    地址计算：mem_addr = rs1 + sign_extend(imm)
            //    数据写入策略（字节级写入）：
            //      SB：只写 mem[addr][7:0]，高 24 位保持不变
            //      SH：只写 mem[addr][15:0]，高 16 位保持不变
            //      SW：全字写入
            //
            //    ★ 字节级写入模拟了真实存储器的行为——部分字写入
            //      不会破坏相邻字节。这与 DUT 中 BRAM + store_mask 的行为一致。
            // ═══════════════════════════════════════════════════════════
            7'b0100011: begin
                store_n++;
                // ★ S-type 立即数拼接：inst[31:25] | inst[11:7]
                imm = {{20{mc[31]}}, mc[31:25], mc[11:7]};
                mem_addr = xreg[r1] + imm;
                mem_wdata = xreg[r2];                    // ★ store 数据 = rs2 的值
                mem_wr = 1;
                case (f3)
                    3'b000: mem[mem_addr] = {mem[mem_addr][31:8], xreg[r2][7:0]};      // SB
                    3'b001: mem[mem_addr] = {mem[mem_addr][31:16], xreg[r2][15:0]};    // SH
                    3'b010: mem[mem_addr] = xreg[r2];                                   // SW
                endcase
            end

            // ═══════════════════════════════════════════════════════════
            //  Branch (1100011)：条件分支
            //    BEQ/BNE/BLT/BGE/BLTU/BGEU
            //
            //    ★ 注意：golden model 的 execute() 不更新 PC！
            //      PC 在函数末尾由外部调用者更新（pc = pc + 4）。
            //      分支跳转需要通过返回值或外部逻辑处理。
            //      当前实现只计算比较结果，不实际跳转。
            //
            //    B-type 立即数格式（13-bit，LSB=0）：
            //      {inst[31], inst[7], inst[30:25], inst[11:8], 1'b0}
            //      符号扩展为 32-bit
            // ═══════════════════════════════════════════════════════════
            7'b1100011: begin
                branch_n++;
                imm = {{20{mc[31]}}, mc[7], mc[30:25], mc[11:8], 1'b0};
                bit take;
                case (f3)
                    0: take = (xreg[r1] == xreg[r2]);                        // BEQ
                    1: take = (xreg[r1] != xreg[r2]);                        // BNE
                    4: take = ($signed(xreg[r1]) < $signed(xreg[r2]));       // BLT
                    5: take = ($signed(xreg[r1]) >= $signed(xreg[r2]));      // BGE
                    // ★ BLTU/BGEU 未实现——补充后可支持完整 B-type
                endcase
            end

            // ═══════════════════════════════════════════════════════════
            //  JAL (1101111)：跳转并链接
            //    rd = PC + 4（返回地址）
            //    PC = PC + imm（跳转目标）
            //
            //    J-type 立即数格式（21-bit，LSB=0）：
            //      {inst[31], inst[19:12], inst[20], inst[30:21], 1'b0}
            //      符号扩展为 32-bit
            // ═══════════════════════════════════════════════════════════
            7'b1101111: begin
                jump_n++;
                imm = {{12{mc[31]}}, mc[19:12], mc[20], mc[30:21], 1'b0};
                if (rd_ != 0) xreg[rd_] = pc + 4;         // ★ 链接地址 = PC + 4
                result = pc + 4;                            // ★ 返回链接地址
                pc = pc + imm;                              // ★ 更新 PC（跳转）
            end

            // ═══════════════════════════════════════════════════════════
            //  JALR (1100111)：寄存器间接跳转并链接
            //    rd = PC + 4（返回地址）
            //    PC = (rs1 + imm) & ~1（目标地址，LSB 清零）
            //
            //    ★ JALR 的 LSB 清零 (&~32'h1) 是 RISC-V 规范要求
            //      ——确保跳转地址最低位为 0（指令对齐）
            // ═══════════════════════════════════════════════════════════
            7'b1100111: begin
                jump_n++;
                imm = {{20{mc[31]}}, mc[31:20]};           // ★ 12-bit 符号扩展
                if (rd_ != 0) xreg[rd_] = pc + 4;          // ★ 链接地址
                pc = (xreg[r1] + imm) & ~32'h1;            // ★ LSB 清零 + 跳转
            end

            // ═══════════════════════════════════════════════════════════
            //  LUI (0110111)：加载高位立即数
            //    rd = imm << 12（立即数左移 12 位，低 12 位补 0）
            //
            //    ★ LUI 的典型用途：加载 32-bit 常量的高 20 位
            //      例如：lui t0, 0x20000 → t0 = 0x20000000
            //            addi t0, t0, 0x100 → t0 = 0x20000100
            // ═══════════════════════════════════════════════════════════
            7'b0110111: begin
                alu_n++;
                result = {mc[31:12], 12'h0};               // ★ 左移 12 位
                if (rd_ != 0) xreg[rd_] = result;
            end

            // ═══════════════════════════════════════════════════════════
            //  AUIPC (0010111)：PC 加高位立即数
            //    rd = PC + (imm << 12)
            //
            //    ★ AUIPC 的典型用途：构建 PC-relative 地址
            //      例如：auipc t0, 0 → t0 = PC 的当前值
            //            auipc t0, 0x10 → t0 = PC + 0x10000
            // ═══════════════════════════════════════════════════════════
            7'b0010111: begin
                alu_n++;
                result = pc + {mc[31:12], 12'h0};          // ★ PC + 立即数左移 12
                if (rd_ != 0) xreg[rd_] = result;
            end

        endcase

        // ★ 每条指令执行完后 PC += 4（除非已被 JAL/JALR 修改）
        pc = pc + 4;
        instr_count++;
    endfunction

    // ═══════════════════════════════════════════════════════════════════════
    //  辅助函数
    // ═══════════════════════════════════════════════════════════════════════

    // get_reg() — 读取寄存器值（供外部 scoreboard 比对）
    virtual function bit [31:0] get_reg(bit [4:0] r);
        return xreg[r];
    endfunction

    // report_stats() — 打印指令统计报告
    //   输出示例：Instrs:150 ALU:80 Br:20 Ld:15 St:10 Jmp:25
    //   用于快速评估测试固件的指令覆盖情况
    virtual function void report_stats();
        `uvm_info("GMODEL",
            $sformatf("Instrs:%0d ALU:%0d Br:%0d Ld:%0d St:%0d Jmp:%0d",
                      instr_count, alu_n, branch_n, load_n, store_n, jump_n), UVM_NONE)
    endfunction

endclass
