// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  riscv_if.sv — RISC-V SoC 虚拟接口（Virtual Interface）                       ║
// ║  ★ AXI 5 通道 + GPIO + UART + Timer IRQ + Clocking Blocks + 复位等待          ║
// ╚══════════════════════════════════════════════════════════════════════════════════╝
//
// 【SystemVerilog Interface 是什么】
//   Interface 是 SystemVerilog 引入的一种"信号包装"机制。
//   它把一组相关信号打包在一起，用一个 interface 实例传递，而不是分别连接每个信号。
//
//   传统方式（Verilog）：每个信号分别连接
//     .awaddr(awaddr), .awvalid(awvalid), .awready(awready), ...
//   Interface 方式（SystemVerilog）：
//     .vif(my_interface)  —— 一个 interface 搞定全部信号
//
//   好处：
//     1. 减少端口声明冗余（几十个信号只需一个 interface）
//     2. 统一时钟域（interface 内部可以有时钟块和时序约束）
//     3. 可以在 interface 中定义 task（如 wait_reset_release）
//
// 【Virtual Interface 为什么必须用】
//   UVM 的 class（driver/monitor/scoreboard）是动态对象——在堆上分配，
//   不能直接引用静态的 module 级信号。
//
//   Virtual interface 是指向实际 interface 实例的**指针/句柄**——
//   它桥接了"动态 UVM class"和"静态 RTL 信号"之间的鸿沟。
//
//   在 testbench 中：
//     riscv_if dut_if(clk, resetn);   ← 实例化 interface
//     uvm_config_db#(virtual riscv_if)::set(null, "*", "vif", dut_if);
//     ← 将 virtual interface 注入 config_db
//   Driver/Monitor 中：
//     uvm_config_db#(virtual riscv_if)::get(this, "", "vif", vif);
//     ← 从 config_db 取出 virtual interface
//
// 【Clocking Block（时钟块）的作用】
//   clocking block 是 SystemVerilog 中解决"信号竞争（race condition）"的机制。
//
//   问题：在仿真中，时序逻辑用 @(posedge clk) 驱动信号，组合逻辑也在同一时刻
//   读取信号——谁先执行？结果可能不确定。
//
//   clocking block 通过**时序偏移（skew）**解决这个问题：
//
//     driver_cb:   output #0（Driver 在时钟沿后 0 延迟驱动——非阻塞）
//                  input #1step（Driver 读信号时，读的是时钟沿前 1 步的值）
//     monitor_cb:  input #1（Monitor 在时钟沿前采样——看到的是稳定值）
//
//   ★ 简单理解：
//     Driver 用 driver_cb 驱动（在时钟沿"之后"写）→ 给信号稳定的时间
//     Monitor 用 monitor_cb 采样（在时钟沿"之前"读）→ 看到 Driver 之前写的值
//
// 【为什么需要两个 clocking block（driver_cb 和 monitor_cb）】
//   Driver 和 Monitor 有时序需求相反：
//     Driver：希望在时钟沿之后尽快驱动（output #0），尽快收到反馈（input #1step）
//     Monitor：希望在时钟沿之前采样（input #1），看到稳定值
//
//   用一个 clocking block 无法同时满足两者的时序需求。
//   两个 clocking block 让 Driver 和 Monitor 独立控制自己的时序。
//
// 【wait_reset_release() task】
//   提供统一的复位等待逻辑——所有 test/sequence 都可以调用。
//   逻辑：
//     1. 如果复位已经释放（resetn=1） → 等 5 拍稳定 → 返回
//     2. 如果复位还在（resetn=0） → 等 posedge resetn → 等 5 拍 → 返回
//   等 5 拍是为了确保复位释放后 DUT 内部状态稳定。
//
// 【面试要点】
//   Q: wire 和 logic 在 interface 中用哪个？
//   A: logic 是首选——它可以在 always 块中赋值（reg 的功能），
//      也可以连接到 module 端口。只有在多驱动场景（如双向总线）才用 wire。
//
//   Q: clocking block 的 skew 怎么设？
//   A: Driver 用 output #0, input #1step（尽快驱动，采样上个 delta cycle 的值）
//      Monitor 用 input #1（采样时钟沿前的值，避免竞争）
// ═══════════════════════════════════════════════════════════════════════════════

interface riscv_if(input bit clk, input bit resetn);

    // ═══════════════════════════════════════════════════════════════════════
    //  AXI 写地址通道（AW）
    // ═══════════════════════════════════════════════════════════════════════
    logic [31:0] axi_awaddr;       // 写地址
    logic [2:0]  axi_awsize;       // ★ 传输宽度编码（0=1B, 1=2B, 2=4B）
    logic        axi_awvalid;      // Master 写地址有效
    logic        axi_awready;      // Slave 写地址就绪

    // ═══════════════════════════════════════════════════════════════════════
    //  AXI 写数据通道（W）
    // ═══════════════════════════════════════════════════════════════════════
    logic [31:0] axi_wdata;        // 写数据
    logic [3:0]  axi_wstrb;        // ★ 字节写掩码（每 bit 对应 1 字节）
    logic        axi_wvalid;       // Master 写数据有效
    logic        axi_wready;       // Slave 写数据就绪

    // ═══════════════════════════════════════════════════════════════════════
    //  AXI 写响应通道（B）
    // ═══════════════════════════════════════════════════════════════════════
    logic [1:0]  axi_bresp;        // ★ 写响应码：00=OKAY, 10=SLVERR, 11=DECERR
    logic        axi_bvalid;       // Slave 写响应有效
    logic        axi_bready;       // Master 写响应就绪

    // ═══════════════════════════════════════════════════════════════════════
    //  AXI 读地址通道（AR）
    // ═══════════════════════════════════════════════════════════════════════
    logic [31:0] axi_araddr;       // 读地址
    logic [2:0]  axi_arsize;       // ★ 传输宽度编码
    logic        axi_arvalid;      // Master 读地址有效
    logic        axi_arready;      // Slave 读地址就绪

    // ═══════════════════════════════════════════════════════════════════════
    //  AXI 读数据通道（R）
    // ═══════════════════════════════════════════════════════════════════════
    logic [31:0] axi_rdata;        // 读数据
    logic [1:0]  axi_rresp;        // ★ 读响应码
    logic        axi_rvalid;       // Slave 读数据有效
    logic        axi_rready;       // Master 读数据就绪

    // ═══════════════════════════════════════════════════════════════════════
    //  外设信号（直接连接到 DUT 的顶层端口）
    // ═══════════════════════════════════════════════════════════════════════
    logic [31:0] gpio_out;         // GPIO 输出（DUT → 外部）
    logic [31:0] gpio_in;          // GPIO 输入（外部 → DUT）
    logic        uart_tx;          // UART 发送（DUT → 外部）
    logic        uart_rx;          // UART 接收（外部 → DUT）
    logic        timer_irq;        // Timer 中断（DUT → PLIC）

    // ╔══════════════════════════════════════════════════════════════════════════╗
    // ║  Driver Clocking Block — 用于 Driver 驱动信号                               ║
    // ║  ★ output #0：Driver 写的信号在时钟沿后尽快生效（非阻塞赋值）               ║
    // ║  ★ input #1step：Driver 读的信号为上一个 delta cycle 的值（避免竞争）       ║
    // ╚══════════════════════════════════════════════════════════════════════════════╝
    clocking driver_cb @(posedge clk);
        default output #0;         // ★ 驱动延迟：0 → 使用非阻塞赋值
        default input  #1step;     // ★ 采样延迟：1step → 读上一个 delta cycle 的值
        // --- Driver 驱动的信号（output）---
        output axi_awaddr, axi_awsize, axi_awvalid;
        output axi_wdata, axi_wstrb, axi_wvalid;
        output axi_bready;
        // --- Driver 读取的信号（input）---
        input  axi_awready, axi_wready, axi_bvalid, axi_bresp;
        output axi_araddr, axi_arsize, axi_arvalid, axi_rready;
        input  axi_arready, axi_rvalid, axi_rdata, axi_rresp;
    endclocking

    // ╔══════════════════════════════════════════════════════════════════════════╗
    // ║  Monitor Clocking Block — 用于 Monitor 采样信号                            ║
    // ║  ★ input #1：Monitor 采样的信号为时钟沿前的值（看到稳定值）                  ║
    // ╚══════════════════════════════════════════════════════════════════════════════╝
    clocking monitor_cb @(posedge clk);
        default input #1;          // ★ 采样延迟：1 → 读时钟沿前的值
        // --- Monitor 只采样，不驱动（全部 input）---
        input  axi_awaddr, axi_awsize, axi_awvalid, axi_awready;
        input  axi_wdata, axi_wstrb, axi_wvalid, axi_wready;
        input  axi_bresp, axi_bvalid, axi_bready;
        input  axi_araddr, axi_arsize, axi_arvalid, axi_arready;
        input  axi_rdata, axi_rresp, axi_rvalid, axi_rready;
        input  gpio_out, gpio_in, uart_tx, uart_rx, timer_irq;
    endclocking

    // ═══════════════════════════════════════════════════════════════════════
    //  wait_reset_release() — 等待复位释放
    //
    //  ★ 为什么需要这个 task？
    //    测试开始前，DUT 可能还在复位中。如果 sequence 在复位期间发送激励，
    //    DUT 不会响应（或行为不可预测）。wait_reset_release 确保复位已释放。
    //
    //  ★ 额外等 5 拍：确保复位释放后的亚稳态窗口结束，DUT 内部状态稳定。
    // ═══════════════════════════════════════════════════════════════════════
    task wait_reset_release();
        if (resetn === 1'b1) begin            // ★ 已经释放：直接等 5 拍
            repeat (5) @(posedge clk);
            return;
        end
        @(posedge resetn);                    // ★ 等 posedge resetn
        repeat (5) @(posedge clk);           // ★ 额外等 5 拍
    endtask

endinterface
