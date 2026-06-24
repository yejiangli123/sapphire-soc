// ╔══════════════════════════════════════════════════════════════════════╗
// ║              Bypass_unit.v — 数据旁路 + Load-Use 冒险检测            ║
// ║                                                                    ║
// ║  【流水线冒险类型】                                                  ║
// ║  本模块处理经典 5 级流水线中的两类数据冒险（Data Hazard）：            ║
// ║                                                                    ║
// ║  1. RAW（Read After Write）数据冒险 — 通过转发（Forwarding）解决      ║
// ║     场景：前一条指令的 rd 写入尚未到达写回阶段，而后一条指令的        ║
// ║     rs1/rs2 已经需要读取该值。                                       ║
// ║     示例：                                                          ║
// ║       add x1, x2, x3    // EX 阶段计算 x1，结果在 MEM 末端才写回     ║
// ║       sub x4, x1, x5    // ID 阶段就需要 x1 的值 → RAW 冒险          ║
// ║     解决：将 EX/MEM 或 MEM/WB 阶段的结果"旁路"到 ID 阶段的操作数。   ║
// ║                                                                    ║
// ║  2. Load-Use 冒险（RAW 特例）— 必须停顿（Stall）                     ║
// ║     场景：前一条指令是 Load（在 MEM 阶段才从内存取数据），后一条指令  ║
// ║     在 EX 阶段就需要该值。此时数据在 MEM 末端才可用，即使转发也来不及。 ║
// ║     示例：                                                          ║
// ║       lw  x1, 0(x2)     // EX 算地址，MEM 才取数                     ║
// ║       add x4, x1, x5    // EX 阶段就需要 x1 → 转发也来不及           ║
// ║     解决：插入 1 个气泡（stall 1 周期），让 Load 完成 MEM 后再执行。  ║
// ║                                                                    ║
// ║  【检测逻辑详解】                                                    ║
// ║                                                                    ║
// ║  - 转发匹配条件（ex_hit / mem_hit）：                                ║
// ║    1. 目标指令确实要写寄存器（reg_write = 1）。                       ║
// ║       排除 Store/Branch/JALR 等不写 rd 的指令。                       ║
// ║    2. 目标指令的 rd 与当前指令的 rs1/rs2 相等。                      ║
// ║       rd 是 5-bit 寄存器号，直接相等比较。                            ║
// ║    3. rd ≠ x0（即 rd ≠ 5'b00000）。                                 ║
// ║       RISC-V 规定 x0 硬连线为 0，不可改写。"写 x0"实际上不影响        ║
// ║       寄存器 0 的值，因此不能转发也不会引起冒险。                     ║
// ║       (|ex_rd) 是缩减或运算：ex_rd 任意位为 1 即为真。               ║
// ║                                                                    ║
// ║  - Load-Use 停顿条件（load_use_hazard）：                             ║
// ║    1. EX 阶段指令是 Load（ex_load = 1）。                            ║
// ║    2. EX 阶段 Load 的 rd 被 ID 阶段的 rs1 或 rs2 使用。             ║
// ║    满足即拉高 stall_out，流水线控制逻辑在 ID 阶段插入气泡。           ║
// ║                                                                    ║
// ║  【旁路路径与优先级编码】                                             ║
// ║                                                                    ║
// ║  两条旁路数据来源：                                                   ║
// ║    - EX 旁路 (2'b01)：数据来自 EX/MEM 流水线寄存器的 alu_out。        ║
// ║    - MEM 旁路 (2'b10)：数据来自 MEM/WB 流水线寄存器的 mem_rdata       ║
// ║                        或 alu_out。                                  ║
// ║    - 无旁路 (2'b00)：使用寄存器堆读出的值。                            ║
// ║                                                                    ║
// ║  ★ 优先级：EX > MEM（2'b01 优先于 2'b10）                            ║
// ║    原因：当 EX 和 MEM 的 rd 恰好相同（例如背靠背写同一寄存器），       ║
// ║    EX 阶段的数据是更新的值。如果取了 MEM 的数据，就会得到旧值，        ║
// ║    产生 WAW（Write After Write）错误。                                ║
// ║    示例：                                                           ║
// ║      add x1, x2, x3   // EX 阶段 → x1 = x2+x3（新值）               ║
// ║      sub x1, x4, x5   // ID 阶段（上一个 add 在 MEM）→ x1 应取      ║
// ║                       //   MEM 的旧值还是 EX 的新值？当然是 EX！      ║
// ║    转发 MUX 用嵌套条件表达式实现优先级：先检查 ex_hit，再检查 mem_hit。║
// ║                                                                    ║
// ║  编码选择理由：                                                      ║
// ║    - 2'b00 为无旁路（默认）。这样 MUX 默认通路是寄存器堆输出，         ║
// ║      当所有旁路条件都不满足时，自然回退到正常读寄存器。                ║
// ║    - 2'b01 为 EX 旁路，2'b10 为 MEM 旁路。编码本身不隐含优先级        ║
// ║      （优先级由生成逻辑保证），但便于下游 MUX 直接用作选择信号。       ║
// ║                                                                    ║
// ║  【与流水线各阶段的交互】                                             ║
// ║                                                                    ║
// ║  本模块位于 ID 阶段，是组合逻辑（纯 wire + always @(*)）。             ║
// ║                                                                    ║
// ║  输入来自：                                                          ║
// ║    ID 阶段：rs1, rs2（当前指令的源寄存器号，来自寄存器堆读端口）        ║
// ║    ID/EX 流水线寄存器输出：ex_rd, ex_reg_write, ex_load               ║
// ║      （EX 阶段指令的信息，即 ID 阶段的下一条指令）                     ║
// ║    EX/MEM 流水线寄存器输出：mem_rd, mem_reg_write                     ║
// ║      （MEM 阶段指令的信息，即 ID 阶段的下下条指令）                    ║
// ║                                                                    ║
// ║  输出去向：                                                          ║
// ║    irmux1[1:0], irmux2[1:0] → 操作数选择 MUX                          ║
// ║      控制 EX 阶段 ALU 的操作数来源：寄存器堆 or 旁路数据。             ║
// ║    stall_out → 流水线控制器（通常在 cpu_top 或 pipeline_ctrl 中）      ║
// ║      高有效时：PC 不更新、ID/EX 寄存器插入 NOP（气泡）。               ║
// ║    u_type_contrl_in → U-type 指令（LUI/AUIPC）的转发控制               ║
// ║      U-type 指令将立即数直接作为结果，不需要 ALU 计算，故转发路径不同。 ║
// ║                                                                    ║
// ║  时序示意（以 RAW 转发为例）：                                        ║
// ║    周期 n  ：add x1,x2,x3 在 EX，sub x4,x1,x5 在 ID                  ║
// ║             bypass_unit 检测到 ex_hit_rs1，输出 irmux1=2'b01          ║
// ║    周期 n+1：sub 进入 EX，MUX 选择 EX/MEM 的 alu_out 作为操作数 1     ║
// ║             → sub 正确使用了 add 的结果，无需停顿                      ║
// ║                                                                    ║
// ║  时序示意（以 Load-Use 停顿为例）：                                   ║
// ║    周期 n  ：lw x1,0(x2) 在 EX，add x4,x1,x5 在 ID                   ║
// ║             bypass_unit 检测到 load_use_hazard，拉高 stall_out         ║
// ║             流水线控制器：PC 不变，ID/EX 插入 NOP                      ║
// ║    周期 n+1：lw 在 MEM（取数），add 被冻结在 ID，NOP 进入 EX           ║
// ║    周期 n+2：lw 在 WB（写回），add 仍在 ID，现在 mem_hit_rs1 有效     ║
// ║             → irmux1=2'b10，MEM 旁路数据可用                           ║
// ║    周期 n+3：add 进入 EX，MUX 选择旁路数据 → 正常执行                  ║
// ║                                                                    ║
// ║  【输入信号说明】                                                     ║
// ║  sb_type_in     : Store/Branch 类型标志。这两类指令 rd 无意义，        ║
// ║                   但本模块未直接使用（检测已由 ex_reg_write 处理）。    ║
// ║  in_u_type      : U-type 标志（LUI/AUIPC），其结果为立即数而非 ALU 结果 ║
// ║  bypass_ir1_in  : 操作数 1 旁路使能，不需要旁路时可强制关闭             ║
// ║  bypass_ir2_in  : 操作数 2 旁路使能                                    ║
// ║  op_format_contrl_in : 指令格式控制（保留，当前逻辑未使用）             ║
// ║  ir_mux_in[1:0] : 操作数 2 的默认 MUX 选择（当不需要旁路时使用）       ║
// ║                                                                    ║
// ║  ★ 架构特点：使用显式流水线寄存器输入（ex_* / mem_*）                 ║
// ║    而非模块内部推断移位寄存器。这样流水线寄存器由上层模块显式           ║
// ║    管理（ID/EX、EX/MEM 寄存器），本模块只做组合逻辑比较。             ║
// ║    好处：清晰的数据流向，便于综合工具优化，避免意外锁存器。            ║
// ╚══════════════════════════════════════════════════════════════════════╝

module bypass_unit (
    input  wire        clk,
    input  wire        reset,
    input  wire        flush,              // 流水线刷新信号（分支预测失败等场景）
    // 指令阶段控制信号
    input  wire        sb_type_in,          // Store/Branch 类型标志（rd 无效）
    input  wire        in_u_type,           // U-type 指令标志（LUI/AUIPC）
    input  wire        bypass_ir1_in,       // 操作数 1 旁路使能（全局控制）
    input  wire        bypass_ir2_in,       // 操作数 2 旁路使能（全局控制）
    input  wire [2:0]  op_format_contrl_in, // 指令格式控制码（保留扩展）
    input  wire [1:0]  ir_mux_in,           // 操作数 2 默认选择（无旁路时的 MUX 值）
    // ID 阶段源寄存器索引
    input  wire [4:0]  rs1,                // 当前 ID 阶段指令的源寄存器 1
    input  wire [4:0]  rs2,                // 当前 ID 阶段指令的源寄存器 2
    // ★ EX 阶段指令信息（来自 ID/EX 流水线寄存器输出）
    //   EX 阶段的指令紧跟在 ID 阶段指令之后，是"上一条"指令
    input  wire [4:0]  ex_rd,               // EX 阶段指令的目标寄存器 rd
    input  wire        ex_reg_write,        // EX 阶段指令是否写寄存器堆
    input  wire        ex_load,             // EX 阶段指令是否为 Load（lw/lb/lbu/lh/lhu）
    // ★ MEM 阶段指令信息（来自 EX/MEM 流水线寄存器输出）
    //   MEM 阶段的指令是"上上条"指令，比 EX 还早一拍
    input  wire [4:0]  mem_rd,              // MEM 阶段指令的目标寄存器 rd
    input  wire        mem_reg_write,       // MEM 阶段指令是否写寄存器堆
    // 输出
    output wire [1:0]  irmux1,              // 操作数 1 的旁路选择信号
    output wire [1:0]  irmux2,              // 操作数 2 的旁路选择信号
    output reg         stall_out,           // Load-Use 停顿请求（高有效）
    output reg         u_type_contrl_in     // U-type 转发控制标志
);

    // ═══════════════════════════════════════════════════════════════════
    // ★ 原始检测逻辑 — 组合转发匹配
    // ═══════════════════════════════════════════════════════════════════
    //
    // 对 ID 阶段的每条指令，同时检查 EX 和 MEM 阶段的指令是否与
    // 当前指令的 rs1/rs2 存在数据依赖：
    //
    // 三个必要条件（全部满足才命中）：
    //   (a) 目标阶段指令确实要写寄存器（ex_reg_write / mem_reg_write）
    //       → 排除 Store、Branch 等不写回的指令
    //   (b) 目标阶段指令的 rd 与当前 ID 指令的 rs1/rs2 相等
    //       → 5-bit 寄存器地址直接比较
    //   (c) rd ≠ x0（即 rd 不为 5'b00000）
    //       → RISC-V 架构规定 x0 恒为 0，写入 x0 不影响寄存器值
    //       → (|ex_rd) 是 Verilog 缩减或：rd 任何位为 1 即表示非零
    //
    // 注意：这些信号是纯组合逻辑，在 rs1/rs2/ex_rd/mem_rd 变化时立即更新。

    // --- EX 阶段命中检测 ---
    // EX 阶段的指令刚好在 ID 指令前一个周期发射
    wire ex_hit_rs1  = ex_reg_write  && (ex_rd  == rs1) && (|ex_rd);
    wire ex_hit_rs2  = ex_reg_write  && (ex_rd  == rs2) && (|ex_rd);

    // --- MEM 阶段命中检测 ---
    // MEM 阶段的指令在 ID 指令前两个周期发射
    wire mem_hit_rs1 = mem_reg_write && (mem_rd == rs1) && (|mem_rd);
    wire mem_hit_rs2 = mem_reg_write && (mem_rd == rs2) && (|mem_rd);


    // ═══════════════════════════════════════════════════════════════════
    // ★ Load-Use 冒险检测
    // ═══════════════════════════════════════════════════════════════════
    //
    // Load-Use 是 RAW 冒险中唯一无法通过转发完全消除的情况：
    //   - Load 指令在 MEM 阶段才从数据内存读出数据
    //   - 而 ALU 指令在 EX 阶段就需要操作数
    //   - 即使将 MEM/WB 的结果转发，时间上也晚了一拍
    //
    // 解决方式：检测到 Load-Use 时，流水线控制器插入一个气泡（NOP），
    // 暂停 ID 及之前的阶段一个周期，让 Load 完成 MEM 阶段。
    //
    // 检测条件（同时满足）：
    //   (1) EX 阶段指令是 Load（ex_load = 1）
    //   (2) Load 的目标寄存器被 ID 阶段指令的 rs1 或 rs2 使用
    //       （复用了 ex_hit_rs1 / ex_hit_rs2 的逻辑）
    //
    wire load_use_hazard = ex_load && (ex_hit_rs1 || ex_hit_rs2);


    // ═══════════════════════════════════════════════════════════════════
    // ★ 旁路选择 MUX 编码生成 — EX 优先于 MEM
    // ═══════════════════════════════════════════════════════════════════
    //
    // 编码规则（2-bit，下游 MUX 直接使用）：
    //   2'b00 — 无旁路，使用寄存器堆读出的默认值
    //   2'b01 — EX/MEM 旁路，取 EX 阶段 ALU 计算结果（数据更新）
    //   2'b10 — MEM/WB 旁路，取 MEM 阶段读出的数据或 ALU 结果
    //
    // ★ 为什么 EX 优先于 MEM（ex_hit 先于 mem_hit 被检查）？
    //   考虑以下指令序列：
    //     add x1, x2, x3    // 周期 n 在 EX，周期 n+1 在 MEM
    //     sub x1, x4, x5    // 周期 n 在 ID，周期 n+1 在 EX
    //     and x6, x1, x7    // 周期 n+1 在 ID
    //   对于周期 n+1 的 and 指令：
    //     - ex_hit_rs1 为真：sub 的 rd=x1 在 EX 阶段（值 = x4-x5，最新）
    //     - mem_hit_rs1 也为真：add 的 rd=x1 在 MEM 阶段（值 = x2+x3，旧值）
    //   必须选择 EX 旁路（x4-x5），否则会用到已经过期的旧数据。
    //
    // 实现：嵌套三元运算符（ex_hit 先判断，然后才是 mem_hit）。

    wire [1:0] fwd1 = ex_hit_rs1  ? 2'b01 :   // 优先：EX 阶段转发
                       mem_hit_rs1 ? 2'b10 :   // 其次：MEM 阶段转发
                                     2'b00;    // 默认：无转发

    wire [1:0] fwd2 = ex_hit_rs2  ? 2'b01 :   // 同上，针对操作数 2
                       mem_hit_rs2 ? 2'b10 :
                                     2'b00;

    // --- 输出选通 ---
    // bypass_ir1_in / bypass_ir2_in 是全局旁路使能：
    //   - 为 1 时，输出上面生成的 fwd1/fwd2 编码
    //   - 为 0 时，强制关闭旁路（操作数 1 走寄存器，操作数 2 走 ir_mux_in 指定通路）
    // 这样可以在不需要旁路的指令类型（如立即数操作）上节省功耗和路径延迟。
    assign irmux1 = bypass_ir1_in ? fwd1 : 2'b00;
    assign irmux2 = bypass_ir2_in ? fwd2 : ir_mux_in;


    // ═══════════════════════════════════════════════════════════════════
    // ★ 停顿与 U-type 控制（组合逻辑输出）
    // ═══════════════════════════════════════════════════════════════════
    //
    // stall_out:
    //   Load-Use 冒险发生时拉高。流水线控制器收到后：
    //     - PC 保持当前值（不取新指令）
    //     - ID/EX 流水线寄存器写入 NOP（插入气泡）
    //     - IF/ID 流水线寄存器保持（或也写 NOP，取决于实现）
    //   这样 Load 指令可以继续推进到 MEM → WB，
    //   而被依赖的指令停留在 ID 阶段等待数据就绪。
    //
    // u_type_contrl_in:
    //   U-type（LUI/AUIPC）指令的结果是立即数，不经过 ALU 计算。
    //   当检测到 EX 或 MEM 阶段的 U-type 指令产生的结果被当前指令使用时，
    //   需要特殊旁路控制：直接取立即数而非 ALU 输出。
    //   条件：当前指令是 U-type（in_u_type=1）且存在 EX 阶段的数据依赖。
    //
    always @(*) begin
        stall_out        = load_use_hazard;
        u_type_contrl_in = in_u_type && (ex_hit_rs1 || ex_hit_rs2);
    end

endmodule
