// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  riscv_ref_model.sv — 参考模型（Reference Model）                             ║
// ║  ★ 字节级可写的内存镜像 + wstrb 感知 + predict() 预测函数                     ║
// ╚══════════════════════════════════════════════════════════════════════════════════╝
//
// 【UVM 角色 — Reference Model】
//   参考模型是 Scoreboard 的"预测引擎"——它模拟 DUT 的预期行为，根据输入预测输出。
//
//   工作流程：
//     Monitor 捕获 AXI 事务 → Scoreboard 调用 ref_model.predict(obs)
//     → ref_model 根据 obs 的 kind/addr/data/wstrb 计算预期结果
//     → 返回 pred（预期 transaction）→ Scoreboard 比对 pred vs obs
//
// 【为什么需要参考模型】
//   Scoreboard 不能"只知道 DUT 输出了什么"——它必须知道"DUT 应该输出什么"。
//   参考模型提供这个"应该是什么"的基准。如果 DUT 输出 != 参考模型预测 → bug。
//
// 【内存模型设计】
//   本参考模型维护一个**字节可写的内存镜像**（associative array）：
//
//     mem[bit[31:0]] — 关联数组（稀疏存储，适合大地址空间）
//     ★ 为什么用关联数组而非定长数组？
//       SoC 地址空间 4GB（32-bit），但只用了几个外设地址范围。
//       定长数组需要 4G/4 = 1G 个条目（4GB），不现实。
//       关联数组只存储实际访问过的地址——高效且仿真友好。
//
//   写操作（mem_write）：
//     1. 字对齐地址：wa = addr & 32'hFFFFFFFC（清零低 2 位）
//     2. 读取旧值（如果存在）
//     3. 根据 wstrb 逐字节更新——只有 wstrb[i]=1 的字节被写入
//     4. 写回 mem[wa]
//
//   读操作（mem_read）：
//     1. 字对齐地址
//     2. 返回 mem[wa]（如果未被写过，返回 32'hDEAD_BEEF）
//
//   ★ 关键设计：mem_write 使用 Monitor 采样的**实际 wstrb**。
//     这意味着参考模型不仅预测正确功能，还能暴露 wstrb 相关的 bug。
//
// 【predict() 函数】
//   输入：obs（Monitor 采样的 AXI transaction）
//   输出：pred（预测的 transaction）
//
//   预测逻辑：
//     · 如果是写操作（kind=1）→ 更新内存镜像 → 返回 resp=OKAY
//     · 如果是读操作（kind=0）→ 从内存镜像读取 → 返回 data + resp=OKAY
//     · 如果是复位中止（resp=2'b11）→ 跳过预测，resp 原样传递
//
//   ★ predict 是 function（不是 task），不能消耗时间，纯组合逻辑。
//     这保证了 Scoreboard 的比对是即时完成的，不引入延迟。
// ═══════════════════════════════════════════════════════════════════════════════

`include "uvm_macros.svh"
import uvm_pkg::*;

class riscv_ref_model extends uvm_component;
    `uvm_component_utils(riscv_ref_model)

    // ★ 内存镜像：关联数组（sparse memory）
    //   key = 32-bit 字节地址 → value = 32-bit 存储值
    //   用关联数组的好处：只存储实际访问的地址，不占全 4GB 空间
    protected bit [31:0] mem[bit [31:0]];

    uvm_analysis_port#(axi_transaction) pred_ap;   // ★ 预测结果广播端口（备用）

    function new(string n, uvm_component p);
        super.new(n, p);
    endfunction

    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        pred_ap = new("pred_ap", this);
    endfunction

    // ── reset()：清空内存镜像（每次新测试前调用）──
    virtual function void reset();
        mem.delete();                              // ★ 关联数组全清
    endfunction

    // ═══════════════════════════════════════════════════════════════════════
    //  mem_write() — 按字节使能写入内存镜像
    //
    //  ★ 字节写掩码（wstrb）感知：
    //    wstrb[0]=1 → 写 data[7:0]   到 mem[wa][7:0]
    //    wstrb[1]=1 → 写 data[15:8]  到 mem[wa][15:8]
    //    wstrb[2]=1 → 写 data[23:16] 到 mem[wa][23:16]
    //    wstrb[3]=1 → 写 data[31:24] 到 mem[wa][31:24]
    //
    //  ★ 每个字节独立判断——支持部分写入（如 SB 只写 1 字节，其余 3 字节不变）
    //
    //  如果 wstrb=4'b0000：不写入任何字节（静默返回）
    //  这是为什么 Scoreboard 中要检查 |obs.wstrb != 0 的原因
    // ═══════════════════════════════════════════════════════════════════════
    function void mem_write(bit [31:0] addr, bit [31:0] data, bit [3:0] wstrb);
        bit [31:0] wa = addr & 32'hFFFFFFFC;       // ★ 字对齐（清零 bit1:0）
        bit [31:0] w  = mem.exists(wa) ? mem[wa] : 32'h0;  // ★ 读旧值（首次写 = 0）
        if (wstrb[0]) w[7:0]   = data[7:0];        // byte 0
        if (wstrb[1]) w[15:8]  = data[15:8];       // byte 1
        if (wstrb[2]) w[23:16] = data[23:16];      // byte 2
        if (wstrb[3]) w[31:24] = data[31:24];      // byte 3
        mem[wa] = w;
    endfunction

    // ── mem_read()：从内存镜像读取 ──
    function bit [31:0] mem_read(bit [31:0] addr, bit [2:0] size, bit [1:0] off);
        bit [31:0] wa = addr & 32'hFFFFFFFC;
        // ★ 如果地址未写入过，返回 0xDEAD_BEEF（便于识别未初始化访问）
        return mem.exists(wa) ? mem[wa] : 32'hDEAD_BEEF;
    endfunction

    // ── copy_fields()：手动逐字段复制 transaction ──
    function void copy_fields(axi_transaction dst, axi_transaction src);
        dst.kind  = src.kind;
        dst.addr  = src.addr;
        dst.data  = src.data;
        dst.size  = src.size;
        dst.wstrb = src.wstrb;
        dst.resp  = src.resp;
        dst.done  = src.done;
    endfunction

    // ═══════════════════════════════════════════════════════════════════════
    //  predict() — 核心预测函数
    //
    //  输入：obs — Monitor 采样的实际 AXI 事务
    //  输出：pred — 预测的"DUT 应该返回什么"
    //
    //  预测逻辑分支：
    //    1. resp=2'b11（复位中止）→ 不做任何预测，resp 原样传递
    //    2. kind=1（写）→ mem_write → 返回 resp=0（OKAY）
    //    3. kind=0（读）→ mem_read → 返回预测的 data + resp=0
    //
    //  ★ 为什么只返回 resp=0（OKAY）？
    //    当前版本的参考模型不模拟错误响应（SLVERR/DECERR）。
    //    如果需要覆盖错误场景，应在 ref_model 中加入地址边界检查逻辑。
    //    这是已知简化——scoreboard 对 error_resp 测试需要特殊处理。
    // ═══════════════════════════════════════════════════════════════════════
    virtual function axi_transaction predict(axi_transaction obs);
        axi_transaction pred = axi_transaction::type_id::create("pred");
        copy_fields(pred, obs);

        // ★ 复位中止事务：不做预测，原样返回 resp
        if (obs.resp == 2'b11) begin
            pred.resp = 2'b11;
            return pred;
        end

        if (obs.kind == 1) begin  // ★ 写操作
            // ★ 使用 Monitor 采样的实际 wstrb（不是 Driver 生成的）
            if (|obs.wstrb)       // | = reduction OR: wstrb 任意位为 1 → 有效写
                mem_write(obs.addr, obs.data, obs.wstrb);
            pred.resp = 0;        // OKAY
        end
        else begin                // ★ 读操作
            pred.data = mem_read(obs.addr, obs.size, obs.addr[1:0]);
            pred.resp = 0;
        end
        return pred;
    endfunction

endclass
