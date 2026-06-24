
/*
module prog_counter(clk,immbj,jt,pcmux,pc);

input [31:0]immbj,jt;
input clk;
input [1:0]pcmux;
output reg[31:0] pc;

always@(posedge clk)
begin
   case(pcmux)

  2'b11 : pc = pc + 4;

  2'b10 : pc = pc + immbj;

  2'b01 : pc = jt;

  2'b00 : pc = 32'h0000_0000;

  default: pc = pc + 4;
   endcase
end
endmodule
*/

/*
module prog_counter(reset_in,pc_mux,trap_ret_in,jt,ready_in,trap_address_in,branch_in,pc_plus_4_out,pc_in,misaligned_instr_out,pc_out,pc_mux_out);

input [31:0]	jt,trap_ret_in,trap_address_in,pc_in;
input reset_in,branch_in,ready_in;
input [1:0] pc_mux;
output [31:0] pc_out,pc_plus_4_out;
output [31:0] pc_mux_out;
output misaligned_instr_out;
reg [31:0] pc_mux_intermediate;
wire [31:0] pc_plus_4_intermediate,inter_pc,next_pc;

parameter Boot_addr = 32'h0000_0000;

assign pc_plus_4_intermediate = pc_in + 4;
assign pc_plus_4_out = pc_plus_4_intermediate;
assign next_pc = (branch_in)? jt: pc_plus_4_intermediate;

always@(branch_in or pc_plus_4_intermediate or jt )
begin

case(branch_in)

1'b0:	next_pc = pc_plus_4_intermediate;

1'b1:	next_pc = jt;

endcase
end

always@(pc_mux or next_pc or trap_address_in or trap_ret_in)
begin

case(pc_mux)

2'b00:	pc_mux_intermediate = Boot_addr;

2'b01:	pc_mux_intermediate = trap_ret_in;

2'b10:	pc_mux_intermediate = trap_address_in;

2'b11:	pc_mux_intermediate = next_pc;

default: pc_mux_intermediate = Boot_addr;
endcase
end

assign pc_mux_out = pc_mux_intermediate;
assign inter_pc = (ready_in) ? pc_mux_intermediate : pc_out;

assign pc_out = (reset_in)? Boot_addr : inter_pc;

assign misaligned_instr_out = next_pc[1] & branch_in;

endmodule
*/
/*
module prog_count(pc_mux,add_mux,clk,immbj,trap_ret_in,trap_address_in,pc_plus_4_out,pc);

input [1:0] pc_mux,add_mux;
input clk;
input [31:0] immbj,trap_ret_in,trap_address_in;

output [31:0] pc_plus_4_out,pc;
wire [31:0] add_intermed;
reg [31:0] pc_intermed;
parameter plus_4_in = 32'h0000_0004;
//always@(add_mux or )

//assign add_intermed = (add_mux)? immbj : 32'h0000_0004;
always@(add_mux or immbj)

assign addr_in = add_intermed;
assign pc_plus_4_out = add_intermed;

always@(pc_mux or jt or immbj or trap_ret_in or trap_address_in)
begin

case(pc_mux)

	2'b00:	pc_intermed = 32'h0000_0000;

	2'b01:	pc_intermed = trap_ret_in;

	2'b10:	pc-intermed = trap_address_in;

	2'b11:	pc-intermed = addr_in;

endcase
end


endmodule
*/

////////////////////////////////////////////////////////////////////////////////
// 模块名: prog_counter
// 功能概述: 流水线程序计数器 (Program Counter)
//
// 本模块是五级流水线处理器的程序计数器单元，负责生成下一条指令的地址。
// 作为取指阶段 (IF, Instruction Fetch) 的核心组件，它需要处理以下场景:
//   1. 顺序执行:   PC = PC + 4
//   2. 分支跳转:   PC = PC + 立即数偏移 (分支预测单元判定跳转成立时)
//   3. 间接跳转:   PC = JALR 目标地址 (寄存器 + 立即数)
//   4. 流水线冲刷: PC = 启动地址 (boot_addr, 复位或异常时)
//   5. 流水线停顿: PC 保持不变 (数据相关或访存等待时)
//   6. 分支误预测恢复: 回退到分支指令时的 PC 值
//
// 数据通路 (组合逻辑链):
//   pc_enter ──→ [PC+4 / PC+imm 计算] ──→ [分支选择] ──→ [启动地址选择]
//   ──→ [JALR 选择] ──→ [停顿选择] ──→ [寄存器打拍] ──→ pc_out
//
// 时序说明:
//   - pc_out 是寄存输出的 PC，在时钟上升沿更新
//   - reg_pc 保存上一个周期 pc_out 的值，用于误预测恢复
//   - 所有地址计算 (PC+4, PC+imm) 均为组合逻辑，在同一周期内完成
////////////////////////////////////////////////////////////////////////////////
module prog_counter(clk,reset,jalr_type_in,immbj,stall_in,pc_mux_in,branch_taken_in,ready_in,wrong_predict_in,pc_plus4_in,pc_imm,pc_out);

//==============================================================================
// 端口列表
//==============================================================================

// --- 时钟与复位 ---
input clk;                          // 系统时钟，上升沿触发
input reset;                        // 异步复位，高有效；复位后将 PC 置为 boot_addr (32'h0000_0000)

// --- JALR 相关 ---
input [31:0] jalr_type_in;          // JALR 指令的目标地址 (来自 jalr_type_adder 模块)
                                    // 格式: {reg_1 + imm, 1'b0}，即 (寄存器+立即数) 并强制最低位为0
                                    // 用于 JALR 类指令的间接跳转寻址

// --- 分支 / 跳转偏移 ---
input [31:0] immbj;                 // 分支/跳转指令的立即数偏移量，已做符号扩展的32位值
                                    // 用于计算 PC + imm (条件分支或 JAL 的目标地址)

// --- 控制信号 ---
input stall_in;                     // 流水线停顿信号，高有效
                                    // 当流水线需要等待 (如数据相关、访存延迟) 时置1
                                    // stall_in=1 时 PC 保持当前值不变

input pc_mux_in;                    // 启动地址选择信号 (1位)
                                    // 1'b1: 选择 boot_addr (32'h0000_0000)，用于复位或异常入口
                                    // 1'b0: 选择正常计算的下一条 PC (next_pc)

input branch_taken_in;              // 分支预测成立信号 (来自分支预测单元)
                                    // 1'b1: 预测分支成立，next_pc = PC + immbj
                                    // 1'b0: 预测分支不成立，next_pc = PC + 4

input ready_in;                     // JALR 目标地址就绪信号
                                    // 1'b1: JALR 地址计算完成，选择 jalr_type_in 作为下一PC
                                    // 1'b0: 选择正常的 next_pc (经 pc_mux 选择后的值)

input wrong_predict_in;             // 分支误预测信号 (来自执行阶段)
                                    // 1'b1: 检测到误预测，pc_enter 回退到 reg_pc (分支时的PC)
                                    // 1'b0: 正常使用 pc_out 作为地址计算基准

// --- 输出 ---
output [31:0] pc_plus4_in;          // PC+4 的当前值 (组合逻辑输出)
                                    // 送给后续流水级，用于 jal 指令的返回地址 (ra = PC+4)
                                    // 也作为顺序执行时的默认下一条指令地址

output [31:0] pc_imm;               // PC+立即数 的当前值 (组合逻辑输出)
                                    // 送给分支预测单元或执行单元作为分支目标地址
                                    // = pc_enter + immbj (带符号扩展的偏移量)

output reg [31:0] pc_out;           // 程序计数器输出 (寄存器输出)
                                    // 这是送入指令存储器的实际 PC 值，在时钟上升沿更新
                                    // 工作流程: 上升沿时将 pc_stall_out 的值打入此寄存器

//==============================================================================
// 内部寄存器与线网
//==============================================================================

reg [31:0] pc_inter_out;            // PC 多路选择中间结果
                                    // 由 pc_mux_in 选择: boot_addr 或 next_pc
                                    // 该值再经 ready_in 条件决定是否被 JALR 目标覆盖

reg [31:0] next_pc;                 // 分支选择后的下一条 PC
                                    // 由 branch_taken_in 选择: pc_imm (跳转) 或 pc_plus4_in (顺序)
                                    // 代表 "正常执行流" 下的下一条指令地址

reg [31:0] pc_mux_out;              // JALR 选择后的 PC 值
                                    // ready_in=1 时选 jalr_type_in (间接跳转)
                                    // ready_in=0 时选 pc_inter_out (启动地址或正常下一PC)

reg [31:0] pc_enter;                // 地址计算的基准 PC
                                    // wrong_predict_in=1 时选 reg_pc (回退到分支时的旧PC)
                                    // wrong_predict_in=0 时选 pc_out (当前PC)
                                    // 所有地址计算 (PC+4, PC+imm) 均基于此值

reg [31:0] pc_stall_out;            // 停顿选择后的 PC 值
                                    // stall_in=1 时保持 pc_out 不变
                                    // stall_in=0 时通过 pc_mux_out (正常更新)
                                    // 在下一时钟沿被打入 pc_out

reg [31:0] reg_pc;                  // 前一周期 PC 值的备份寄存器
                                    // 每个时钟沿将 pc_out 的当前值存入此寄存器
                                    // 用途: 分支误预测恢复时，回退到分支指令时的 PC
                                    // 因为误预测信号 (wrong_predict_in) 到达时，pc_out 已经
                                    // 沿错误路径推进了若干周期，需要 reg_pc 来恢复正确路径

//==============================================================================
// 参数定义
//==============================================================================

parameter boot_addr = 32'h0000_0000; // 启动/复位地址
                                     // 处理器上电或异常复位后，从地址 0x0000_0000 取第一条指令
                                     // 该地址存放引导代码或中断向量表的首项

//==============================================================================
// 第一级组合逻辑: 地址计算
//
// 设计意图: PC+4 和 PC+imm 是两条并行计算路径
//   - PC+4:   顺序执行时使用，也用作 jal 指令的返回地址 (x[rd] = PC+4)
//   - PC+imm: 分支/跳转的目标地址，由分支预测单元决定是否采纳
//
// 基址选用 pc_enter 而非 pc_out:
//   正常情况下 pc_enter = pc_out，行为等价
//   误预测恢复时 pc_enter = reg_pc，允许从分支时刻的正确 PC 重新计算地址
//   这避免了在误预测路径上继续滚动 pc_out 的问题
//==============================================================================

//assign branch_mux_in = (branch_taken_in)
//branch_taken => predict unit     // branch_taken_in 来自分支预测单元
// branch => branch unit           // 实际分支结果来自分支执行单元

assign pc_plus4_in = pc_enter + 4;      // 组合逻辑: 顺序下一条指令地址
assign pc_imm      = pc_enter + immbj;  // 组合逻辑: 分支/跳转目标地址

//==============================================================================
// 第二级组合逻辑: 分支选择 (branch_taken_in 控制)
//
// 功能: 根据分支预测单元的输出，选择下一条指令地址
//   branch_taken_in = 1: 预测分支成立 → 选 pc_imm (跳转目标)
//   branch_taken_in = 0: 预测分支不成立 → 选 pc_plus4_in (顺序执行)
//
// 注意: 这里使用的是分支预测 (speculative) 结果，而非实际执行结果
//       如果预测错误，wrong_predict_in 会在后续级触发恢复流程
//       此时 pc_enter 回退到 reg_pc，从正确路径重新取指
//==============================================================================
always@(branch_taken_in or pc_imm or pc_plus4_in)
begin
	case(branch_taken_in)
	1'b1:next_pc = pc_imm;           // 预测跳转成立，目标地址 = PC + 立即数偏移
	1'b0:next_pc = pc_plus4_in;      // 预测跳转不成立，顺序执行 PC+4
	default:next_pc = pc_plus4_in;   // 安全默认值: 顺序执行 (防止不定态传播)
	endcase
end
/*
if(branch_taken_in)
   next_pc = pc_imm;
else
   next_pc = pc_plus4_in;
end
*/

//==============================================================================
// 第三级组合逻辑: 启动地址选择 (pc_mux_in 控制)
//
// 功能: 选择是使用启动/复位地址，还是正常的下一条指令地址
//   pc_mux_in = 1: 选择 boot_addr (复位、异常、初始化场景)
//   pc_mux_in = 0: 选择 next_pc   (正常运行场景)
//
// 设计考虑:
//   - 使用 1 位而非 2 位的 pc_mux_in 说明当前版本简化了异常处理
//     早期的2位版本支持 trap_ret / trap_address 等更复杂的异常返回路径
//   - boot_addr 硬编码为 0x0000_0000，适合简单嵌入式系统的内存映射
//==============================================================================
always@(pc_mux_in or next_pc)
begin
	case(pc_mux_in)
	1'b1:pc_inter_out = boot_addr;   // 复位/初始化: 从0地址开始取指
	1'b0:pc_inter_out = next_pc;     // 正常运行: 使用分支选择后的下一条PC
	default:pc_inter_out = next_pc;  // 安全默认值: 正常执行
	endcase
end
/*
if(pc_mux_in)
   pc_inter_out = boot_addr;
else
   pc_inter_out = next_pc;
end
*/

//==============================================================================
// 第四级组合逻辑: JALR 目标地址选择 (ready_in 控制)
//
// 功能: 当 JALR 指令的目标地址计算完成时，覆盖正常的 PC 选择
//   ready_in = 1: 选 jalr_type_in (JALR 目标地址，寄存器+立即数)
//   ready_in = 0: 选 pc_inter_out (上一级的输出，启动地址或正常下一PC)
//
// 时序考量:
//   - JALR 需要读取寄存器文件，因此目标地址会在流水线后续级数才就绪
//   - ready_in 信号表示 "JALR 地址已算好，可以写入 PC"
//   - JALR 地址的最低固定为0 (见 jalr_type_adder)，保证指令地址对齐
//==============================================================================
always@(ready_in or jalr_type_in or pc_inter_out)
begin
	if(ready_in)
	   pc_mux_out = jalr_type_in;    // JALR 就绪: 跳转到寄存器+立即数指定的地址
	else
	   pc_mux_out = pc_inter_out;    // 非JALR或JALR未就绪: 使用正常PC
end

//==============================================================================
// 第五级组合逻辑: 流水线停顿处理 (stall_in 控制)
//
// 功能: 当流水线需要停顿时，PC 必须保持不变
//   stall_in = 1: pc_stall_out = pc_out (保持当前值，阻止PC更新)
//   stall_in = 0: pc_stall_out = pc_mux_out (正常通过)
//
// 停顿的典型场景:
//   - 数据相关: 前一条指令的结果尚未生成，后续指令需要等待
//   - 指令缓存未命中: 取指阶段需要等待存储器返回数据
//   - 多周期指令: 乘法、除法等需要多个周期的执行单元
//
// 保持 PC 不变意味着取指阶段会重复取同一条指令，
// 该指令在后续级数被冻结 (或化为 NOP) 直到停顿解除
//==============================================================================
always@(stall_in or pc_out or pc_mux_out)
begin
	if(stall_in)
	   pc_stall_out = pc_out;        // 停顿: PC 保持不变，下周期重新取同一条指令
	else
	   pc_stall_out = pc_mux_out;    // 正常: PC 更新为计算出的下一地址
end

//==============================================================================
// 时序逻辑: PC 寄存器组 (clk / reset)
//
// 这是模块中唯一的时序逻辑块，负责在时钟沿将组合逻辑计算结果打入寄存器
//
// 寄存器行为分析:
//
//   (1) 复位状态 (reset=1, 异步复位):
//       reg_pc <= boot_addr   — 备份寄存器清零
//       pc_out <= boot_addr   — 程序计数器从 0 地址启动
//       组合逻辑会基于 boot_addr 立即计算出 PC+4=4, PC+imm=immbj
//       但 pc_out 要在下一个时钟沿才会从 0 更新为下一个值
//
//   (2) 停顿状态 (reset=0, stall_in=1):
//       reg_pc <= reg_pc      — 备份寄存器保持不变 (不记录重复取指时的旧PC)
//       pc_out <= pc_stall_out — 注意: stall_in=1 时 pc_stall_out = pc_out
//       结果: pc_out 保持原值不变，取指阶段重复取同一地址
//
//   (3) 正常更新 (reset=0, stall_in=0):
//       reg_pc <= pc_out      — 将当前 PC 值保存到备份寄存器
//       pc_out <= pc_stall_out — 将计算好的下一PC打入输出寄存器
//       这种 "先备份、后更新" 的顺序保证 reg_pc 存放的是 "刚刚被替换掉" 的PC
//
//   为什么需要 reg_pc 备份寄存器:
//       分支预测错误时，pc_out 已经沿错误路径推进了1到多个周期
//       需要 reg_pc 保存分支指令时的 PC 值，以便回退重取
//       停顿期间 reg_pc 不变的原因: 停滞时没有真正的"旧PC"需要备份，
//       维持 reg_pc 原有值可在恢复时回退到更早的正确点
//
//   为什么用非阻塞赋值 (<=):
//       标准同步时序逻辑规范，保证信号在时钟沿同时采样、下一周期同时更新
//       避免组合逻辑环和仿真竞争
//==============================================================================
always@(posedge clk or posedge reset)
begin
	if(reset)
	begin
	   reg_pc <= boot_addr;      // 复位: 备份PC清零
	   pc_out <= boot_addr;      // 复位: 程序计数器从0x0000_0000启动
	end
	else if(stall_in)
	begin
	   reg_pc <= reg_pc;         // 停顿: 备份寄存器保持 (不推进历史记录)
	   pc_out <= pc_stall_out;   // 停顿: pc_stall_out = pc_out，即保持不变
	end
	else
	begin
	   reg_pc <= pc_out;         // 正常: 先将当前PC备份到 reg_pc
	   pc_out <= pc_stall_out;   // 正常: 再将计算好的下一PC打入输出寄存器
	end
end

//==============================================================================
// 第六级组合逻辑: 误预测恢复的基址选择 (wrong_predict_in 控制)
//
// 功能: 控制地址计算的基准 PC (pc_enter) 是使用当前 PC 还是回退到旧 PC
//   wrong_predict_in = 1: pc_enter = reg_pc (回退到分支时的PC，从正确路径重新取指)
//   wrong_predict_in = 0: pc_enter = pc_out (正常使用当前PC进行地址计算)
//
// 恢复流程:
//   1. 分支预测单元预测分支成立 → next_pc = pc_imm, 流水线沿跳转路径取指
//   2. 若干周期后，执行单元计算出实际分支结果 → 发现预测错误
//   3. wrong_predict_in 置1
//   4. pc_enter 立即切换为 reg_pc (分支指令时的PC值)
//   5. pc_plus4_in / pc_imm 基于 reg_pc 重新计算
//   6. 后续组合逻辑链基于正确的基准PC重新生成下一地址
//   7. 下一时钟沿将正确地址打入 pc_out
//
// 注意: 此信号是组合逻辑输入，不需要等待时钟沿就能生效
//       误预测恢复是一个紧急过程，应尽快纠正取指方向
//==============================================================================
always@(wrong_predict_in or reg_pc or pc_out)
begin
	if(wrong_predict_in)
	   pc_enter = reg_pc;          // 误预测: 回退到分支时的PC，重新计算正确路径
	else
	   pc_enter = pc_out;          // 正常: 以当前PC为基准计算下一步
end

endmodule


////////////////////////////////////////////////////////////////////////////////
// 模块名: jalr_type_adder
// 功能概述: JALR 指令目标地址加法器
//
// 本模块专用于计算 JALR (Jump And Link Register) 类指令的目标地址。
// RISC-V 架构中，JALR 指令的语义为:
//   rd = PC + 4  (返回地址)
//   PC = (rs1 + sign_extend(imm)) & ~1  (目标地址，最低位清零以保证对齐)
//
// 输入:
//   reg_1  [31:0]: 基址寄存器的值 (rs1)
//   imm    [31:0]: 立即数值 (已做符号扩展的12位立即数)
//
// 输出:
//   jalr_out [31:0]: JALR 目标地址
//     = { (reg_1 + imm)[31:1], 1'b0 }
//     即: 加法结果的低1位强制清零，保证指令地址按2字节对齐
//     在 RV32I 中指令必须按4字节对齐，但这里按2字节对齐处理是为兼容压缩指令集扩展
//
// 时序: 纯组合逻辑，无寄存器。计算结果在同一周期内送达 prog_counter 模块
////////////////////////////////////////////////////////////////////////////////
module jalr_type_adder(reg_1,imm,jalr_out);//to pc

input [31:0] reg_1,imm;             // reg_1: rs1寄存器值, imm: 符号扩展后的12位立即数
output [31:0] jalr_out;             // JALR 目标地址 = {加法结果[31:1], 1'b0}
wire [31:0] jalr_inter;             // 中间加法结果 (32位全宽)

assign jalr_inter = reg_1 + imm;    // 第一步: 基址寄存器 + 立即数偏移
assign jalr_out = {jalr_inter[31:1],1'b0}; // 第二步: 取高31位，最低位置0
                                           // {jalr_inter[31:1], 1'b0} 等价于 (jalr_inter & ~1)
                                           // 保证跳转地址的最低有效位为0 (地址对齐)
                                           // RISC-V 规范要求 JALR 目标地址 LSB 清零

endmodule
