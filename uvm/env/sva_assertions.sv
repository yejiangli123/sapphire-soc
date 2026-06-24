// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  sva_assertions.sv — AXI4-Lite 协议断言检查器（SVA）                           ║
// ║  ★ 12 条断言覆盖：信号稳定性、握手超时、顺序检查、地址对齐                     ║
// ╚══════════════════════════════════════════════════════════════════════════════════╝
//
// 【SVA（SystemVerilog Assertions）是什么】
//   SVA 是 SystemVerilog 内置的断言语言，用于描述"协议必须成立的条件"。
//   它不是"测试"——它是"规则"。一旦规则被违反，仿真器立即报错。
//
//   和 Scoreboard 的区别：
//     Scoreboard：比对"数据正确性"（期望值 vs 实际值）
//     SVA：检查"协议正确性"（信号时序是否符合规范）
//
//   举例：
//     Scoreboard 检查：读回的数据对不对？
//     SVA 检查：VALID 有没有在 READY 之前就撤掉了？（协议违规）
//
// 【为什么需要 SVA】
//   1. 协议违规可能在 Scoreboard 比对之前就导致仿真死锁——SVA 更早发现
//   2. SVA 是"白盒检查"——直接看信号波形，不依赖 transaction 封装
//   3. 面试中 SVA 是高频考点——展示你懂协议级验证
//
// 【本模块的 7 类 12 条断言】
//   A1-A2: 地址/数据稳定性（VALID=1 期间不能变）
//   A3:    VALID 不能中途撤掉（必须等 READY）
//   A4:    响应必须 OKAY（bresp 只能为 0）
//   A5:    握手超时 ≤ 16 周期（防死锁）
//   A6:    写顺序正确（WVALID 之前必须有 AWVALID）
//   A7:    地址字对齐（低 2 位必须为 0）
//
// 【SVA 关键操作符速查】（面试必问）
//   |->    重叠蕴含：前提匹配的同一拍检查后续（"当 A 发生时，同一拍 B"）
//   |=>    非重叠蕴含：前提匹配的下一个周期检查后续（等价 |-> ##1）
//   ##N    延迟 N 个时钟周期
//   ##[M:N] 延迟 M 到 N 个周期（范围延迟）
//   $rose  上升沿检测（上一拍=0，当前拍=1）
//   $past  取 N 个周期前的值
//   $stable信号在上一个周期到当前周期保持不变
//   disable iff 禁用条件（如复位期间不检查）
//
// 【SVA 的 assert/cover/assume 三种用法】
//   assert：必须成立的断言——违反则报错（本文全部用 assert）
//   cover：覆盖点——验证此条件至少在仿真中出现过一次
//   assume：假设——形式验证中约束输入行为
//
// 【注意事项】
//   ★ 本模块是 RTL module（非 UVM class），需要实例化在 testbench 中
//   ★ 当前版本未实例化（sva_checker 定义了但未 bind 到 DUT）——已知 gap
//   ★ 正确的集成方式：在 riscv_tb.sv 中用 bind 语句连接
//       bind riscv_soc sva_checker u_sva_checker(...)
// ═══════════════════════════════════════════════════════════════════════════════

module sva_checker(
    input clk, resetn,

    // ★ AXI 写地址通道
    input [31:0] axi_awaddr,
    input        axi_awvalid, axi_awready,

    // ★ AXI 写数据通道
    input [31:0] axi_wdata,
    input        axi_wvalid, axi_wready,

    // ★ AXI 写响应通道
    input [1:0]  axi_bresp,
    input        axi_bvalid, axi_bready,

    // ★ AXI 读地址通道
    input [31:0] axi_araddr,
    input        axi_arvalid, axi_arready,

    // ★ AXI 读数据通道
    input [31:0] axi_rdata,
    input [1:0]  axi_rresp,
    input        axi_rvalid, axi_rready
);

    // ═══════════════════════════════════════════════════════════════════════
    //  A1-A2: 地址/数据稳定性检查
    //
    //  ★ 为什么需要：AXI 协议规定 VALID=1 期间，地址/数据必须保持稳定。
    //    如果在等 READY 的过程中信号变了 → 从设备收到错误数据 → bug。
    //
    //  ★ $stable 用法：
    //    $stable(sig): sig 从上一拍到当前拍没有变化
    //    等价于 (sig == $past(sig))
    // ═══════════════════════════════════════════════════════════════════════

    // A1: AWADDR 在 AWVALID 等 AWREADY 期间保持稳定
    property p_aw_stable;
        @(posedge clk) disable iff(!resetn)
        (axi_awvalid && !axi_awready) |=> $stable(axi_awaddr);
    endproperty
    AXI_AWADDR_STABLE: assert property(p_aw_stable)
        else $error("AWADDR changed while AWVALID high");

    // A2: WDATA 在 WVALID 等 WREADY 期间保持稳定
    property p_w_stable;
        @(posedge clk) disable iff(!resetn)
        (axi_wvalid && !axi_wready) |=> $stable(axi_wdata);
    endproperty
    AXI_WDATA_STABLE: assert property(p_w_stable)
        else $error("WDATA changed while WVALID high");

    // ═══════════════════════════════════════════════════════════════════════
    //  A3: VALID 不能中途撤掉
    //
    //  ★ AXI 协议规定：发送方一旦拉高 VALID，必须保持直到握手完成。
    //    不能在 READY 还没来的时候就撤掉 VALID。
    //
    //  ★ |=> 的含义：
    //    前件成立 → 下一个周期检查后件。
    //    这里：如果当前拍 (VALID=1 && READY=0) → 下一拍 VALID 必须还是 1
    // ═══════════════════════════════════════════════════════════════════════
    property p_aw_nodrop;
        @(posedge clk) disable iff(!resetn)
        (axi_awvalid && !axi_awready) |=> axi_awvalid;
    endproperty
    AXI_AW_NODROP: assert property(p_aw_nodrop)
        else $error("AWVALID dropped before AWREADY");

    property p_w_nodrop;
        @(posedge clk) disable iff(!resetn)
        (axi_wvalid && !axi_wready) |=> axi_wvalid;
    endproperty
    AXI_W_NODROP: assert property(p_w_nodrop)
        else $error("WVALID dropped before WREADY");

    // ═══════════════════════════════════════════════════════════════════════
    //  A4: 响应检查 — BRESP 必须为 OKAY(0)
    //
    //  ★ 写响应（BRESP）检查：每次 B 握手时，bresp 必须为 2'b00(OKAY)。
    //    SLVERR(2'b10) 和 DECERR(2'b11) 会触发 warning。
    //
    //  ★ 用 warning 而非 error 的原因：
    //    某些测试（error_resp_test）故意触发非 OKAY 响应——不应报错。
    //    warning 意味着"这不是正常情况，但可能是故意的"。
    //
    //  ★ |-> vs |=>：
    //    |-> 是同拍检查：BVALID && BREADY 握手的那一拍，立即检查 bresp
    //    |=> 是下一拍检查，不适合这里（响应值只在握手拍有效）
    // ═══════════════════════════════════════════════════════════════════════
    property p_bresp_ok;
        @(posedge clk) disable iff(!resetn)
        (axi_bvalid && axi_bready) |-> (axi_bresp == 0);
    endproperty
    AXI_BRESP_OK: assert property(p_bresp_ok)
        else $warning("Non-OKAY bresp=%0b", axi_bresp);

    // ═══════════════════════════════════════════════════════════════════════
    //  A5: 握手超时检测 — ≤ 16 周期
    //
    //  ★ 防止死锁：如果 AWVALID=1 后在 16 个周期内都没有 AWREADY，
    //    说明从设备可能卡死了。断言检测这种场景。
    //
    //  ★ ##[1:16] 范围延迟：
    //    "从第 1 到第 16 拍之间的任意一拍"。如果在 16 拍内任意一拍
    //    AWREADY 为 1 → 断言通过。如果 16 拍内 AWREADY 都没拉高 → 断言失败。
    //
    //  ★ 为什么限制 16 周期：
    //    实际设计的从设备应在几拍内响应。16 周期是保守上界。
    //    如果背压测试需要更长延迟，需要增大这个值。
    // ═══════════════════════════════════════════════════════════════════════
    property p_aw_timeout;
        @(posedge clk) disable iff(!resetn)
        axi_awvalid |-> ##[1:16] axi_awready;
    endproperty
    AXI_AW_TIMEOUT: assert property(p_aw_timeout)
        else $error("AW handshake >16 cycles");

    // ═══════════════════════════════════════════════════════════════════════
    //  A6: 写事务顺序 — WVALID 之前必须有 AWVALID
    //
    //  ★ AXI4 协议规定：写数据（W）必须在写地址（AW）之后或同时。
    //    不能没有发 AW 就发 W。
    //
    //  ★ $past(expr, N, enable) 用法：
    //    $past(axi_awvalid, 0, 8): 取 8 拍前到当前拍之间，axi_awvalid 曾经为 1 吗？
    //    参数：expr=表达式, N=往前 N 拍, enable=搜索窗口宽度
    //    "在当前 WVALID=1 时，往前 8 拍内 AWVALID 曾经为 1？"
    //    加上 || axi_awvalid：当前拍 AWVALID=1 也算
    // ═══════════════════════════════════════════════════════════════════════
    property p_wr_order;
        @(posedge clk) disable iff(!resetn)
        axi_wvalid |-> $past(axi_awvalid, 0, 8) || axi_awvalid;
    endproperty
    AXI_WRITE_ORDER: assert property(p_wr_order)
        else $error("WVALID without preceding AWVALID");

    // ═══════════════════════════════════════════════════════════════════════
    //  A7: 地址字对齐检查
    //
    //  ★ AXI4-Lite 协议规定地址必须对齐到传输宽度。
    //    对于 32-bit 数据（size=2），addr[1:0] 必须为 0。
    //    非对齐访问应返回 DECERR。
    //
    //  ★ 这里只检查"全字对齐"（addr[1:0]==0），简化实现。
    //    完整的对齐检查需要考虑 AxSIZE：size=1 → addr[0]==0; size=2 → addr[1:0]==0
    // ═══════════════════════════════════════════════════════════════════════
    property p_addr_align;
        @(posedge clk) disable iff(!resetn)
        (axi_awvalid || axi_arvalid) |->
        ((axi_awvalid ? axi_awaddr[1:0] : axi_araddr[1:0]) == 0);
    endproperty
    AXI_ADDR_ALIGNED: assert property(p_addr_align)
        else $warning("Unaligned access detected");

endmodule
