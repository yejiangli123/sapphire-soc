// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  error_injection_seq.sv — 7 类错误注入测试序列                                   ║
// ║  ★ 非对齐访问 + 只读寄存器写 + 越界地址 + 背靠背写 + RAW + 非法 size + UART flood│
// ╚══════════════════════════════════════════════════════════════════════════════════╝
//
// 【UVM 角色 — uvm_sequence（错误注入型）】
//   本 Sequence 继承了 riscv_base_vseq（拥有 Virtual Sequencer + RAL + VIF），
//   专门设计 7 个错误场景来验证 DUT 的**错误处理能力**。
//
//   正常测试（smoke/stress）只验证"正确输入 → 正确输出"。
//   错误注入测试验证"非法输入 → 合法报错（不崩溃/不死锁）"。
//
//   这两种测试互补——缺一不可：
//     正常测试：验证功能正确性
//     错误注入：验证鲁棒性（Robustness）
//
// 【绕过约束的技巧】
//   有些测试需要"非法值"——但 axi_transaction 的 constraint 限制了合法范围。
//   绕过方法：start_item 和 finish_item 之间**直接赋值**字段（而非 randomize）。
//   因为 randomize() 会触发 constraint 检查——直接赋值的字段不受 constraint 约束。
//
//   例：Test 1 需要非对齐地址 0x40000001，但 addr_align_c 约束要求 size=2 时 addr[1:0]=0。
//   绕过：`start_item(tr); tr.addr = 'h40000001; tr.size = 2; finish_item(tr);`
//   注意顺序——先 start_item 锁定发送权，再赋值，最后 finish_item 发送。
//
//   注意：如果赋值后再调用 randomize()，被赋值的字段可能被覆盖——取决于 constraint。
//   对于这个场景，check_resp 只检查响应码，不检查数据值，所以不影响结果。
//
// 【7 个测试场景】
//   Test 1 — 非对齐地址：word write 到 0x40000001 → 期望 SLVERR
//   Test 2 — 只读寄存器写：写入 GPIO_IN(0x20000008) → 期望 SLVERR
//   Test 3 — 越界地址读：读 0xF0000000 → 期望 DECERR
//   Test 4 — 背靠背连续写：100 次 Timer 寄存器写 → 期望全部 OKAY
//   Test 5 — RAW 冒险验证：写入 0xCAFEBABE → 立即读回比对
//   Test 6 — 非法传输大小：size=3（AXI-Lite 仅支持 0/1/2）→ 期望 SLVERR
//   Test 7 — UART 批量发送：20 字节连续写 UART DATA 寄存器 → 期望全部 OKAY
// ═══════════════════════════════════════════════════════════════════════════════

`include "uvm_macros.svh"
import uvm_pkg::*;

class axi_error_injection_seq extends riscv_base_vseq;
    `uvm_object_utils(axi_error_injection_seq)

    function new(string name = "axi_error_injection_seq");
        super.new(name);
    endfunction

    // ═══════════════════════════════════════════════════════════════════════
    //  check_resp() — 检查 AXI 响应码
    //
    //  参数：
    //    tr       — AXI 事务（包含 resp 字段）
    //    test_name— 测试名称（用于 log）
    //    exp_resp — 期望的响应码（默认 SLVERR = 2'b10）
    //
    //  ★ 为什么单独抽成函数？
    //    7 个测试都需要检查响应——如果不封装，每个测试都要写 if-else。
    //    抽成 check_resp() 后，每个测试只需一行调用。
    //    这是"不要重复自己"（DRY）原则在验证代码中的体现。
    // ═══════════════════════════════════════════════════════════════════════
    function void check_resp(axi_transaction tr, string test_name,
                             bit [1:0] exp_resp = 2'b10);  // ★ 默认期望 SLVERR
        if (tr.resp != exp_resp)
            `uvm_error("ERRINJ",
                $sformatf("%s: expected resp=%0b, got resp=%0b",
                          test_name, exp_resp, tr.resp))
        else
            `uvm_info("ERRINJ",
                $sformatf("%s: PASSED", test_name), UVM_MEDIUM)
    endfunction

    task body();
        axi_transaction tr;
        super.body();                    // ★ 复位释放

        `uvm_info("ERRINJ", "Starting 7 error injection tests", UVM_LOW)

        // ═══════════════════════════════════════════════════════════════
        //  Test 1: 非对齐地址写入
        //
        //  场景：地址 0x40000001 的 word 写（size=2 要求 addr[1:0]=0）
        //  期望：DUT 检测非对齐 → 返回 SLVERR(2'b10)
        //
        //  ★ 绕过对齐约束：start_item 后直接赋值 addr/size
        // ═══════════════════════════════════════════════════════════════
        tr = axi_transaction::type_id::create("tr");
        start_item(tr, , p_sequencer.axi_sqr);
        tr.addr = 'h40000001;            // ★ 非对齐地址（word 写，addr[1:0]≠0）
        tr.kind = 1;                     // 写
        tr.size = 2;                     // 4 字节（要求字对齐）
        finish_item(tr);
        check_resp(tr, "Test1: Unaligned write", 2'b10);

        // ═══════════════════════════════════════════════════════════════
        //  Test 2: 写只读寄存器
        //
        //  场景：写 GPIO_IN(0x20000008)，此寄存器只读
        //  期望：SLVERR
        //
        //  ★ 注意：DUT 是否真的保护只读寄存器？如果 DUT 没有这个保护，
        //    此测试会"假阳性"PASS（实际是 bug，但期望值也设错了）。
        //    这是错误注入测试的风险——需要预先确认 DUT 的实际行为。
        // ═══════════════════════════════════════════════════════════════
        tr = axi_transaction::type_id::create("tr");
        start_item(tr, , p_sequencer.axi_sqr);
        assert(tr.randomize() with {
            addr == 'h20000008;          // GPIO_IN 寄存器
            kind == 1;                   // 写
            data == 'hFFFFFFFF;          // 尝试全写 1
        });
        finish_item(tr);
        check_resp(tr, "Test2: RO register write", 2'b10);

        // ═══════════════════════════════════════════════════════════════
        //  Test 3: 越界地址读取
        //
        //  场景：读地址 0xF0000000（不在任何外设地址空间内）
        //  期望：DECERR(2'b11) — 地址译码错误
        // ═══════════════════════════════════════════════════════════════
        tr = axi_transaction::type_id::create("tr");
        start_item(tr, , p_sequencer.axi_sqr);
        tr.addr = 'hF0000000;            // ★ 越界地址（绕过地址约束）
        tr.kind = 0;                     // 读
        tr.size = 2;
        finish_item(tr);
        check_resp(tr, "Test3: Out-of-range read", 2'b11);  // ★ DECERR

        // ═══════════════════════════════════════════════════════════════
        //  Test 4: 100 次背靠背连续写
        //
        //  场景：连续写 100 次 Timer 寄存器（递增地址），每次检查响应
        //  期望：全部返回 OKAY(2'b00)
        //
        //  ★ 验证点：DUT 是否能处理"背靠背"（back-to-back）写——
        //    在无空闲周期的情况下，Slave VIP 是否能正确处理每次写。
        //    这是吞吐量测试——对应真实场景中 CPU 连续写外设寄存器。
        // ═══════════════════════════════════════════════════════════════
        `uvm_info("ERRINJ", "Test4: 100 back-to-back writes", UVM_MEDIUM)
        for (int i = 0; i < 100; i++) begin
            tr = axi_transaction::type_id::create("tr");
            start_item(tr, , p_sequencer.axi_sqr);
            assert(tr.randomize() with {
                addr == 'h40000000 + i * 4;  // ★ 递增地址
                kind == 1;                   // 写
            });
            finish_item(tr);
            if (tr.resp != 2'b00)
                `uvm_error("ERRINJ",
                    $sformatf("Test4: Write %0d resp=%0b", i, tr.resp))
        end
        `uvm_info("ERRINJ", "Test4: 100 back-to-back writes done", UVM_MEDIUM)

        // ═══════════════════════════════════════════════════════════════
        //  Test 5: RAW 数据冒险（Write-After-Write → Read-After-Write）
        //
        //  场景：写 0xCAFEBABE 到地址 0x40000100 → 立即读回比对
        //  期望：读回的数据 = 写入的数据（0xCAFEBABE）
        //
        //  ★ "RAW" 在这里的含义：
        //    写后读（Read After Write）——如果 DUT 有流水线或缓冲，
        //    可能会"读到了写之前的值"（stale data）。
        //    此测试验证 DUT 是否能正确处理"写后立即读"的时序。
        // ═══════════════════════════════════════════════════════════════
        begin
            axi_transaction wr = axi_transaction::type_id::create("wr");
            axi_transaction rd = axi_transaction::type_id::create("rd");
            // ★ 写入 0xCAFEBABE
            start_item(wr, , p_sequencer.axi_sqr);
            assert(wr.randomize() with {
                addr == 'h40000100;
                kind == 1;
                size == 2;
            });
            wr.data = 32'hCAFEBABE;      // ★ 标志性测试值
            finish_item(wr);
            // ★ 立即读回
            start_item(rd, , p_sequencer.axi_sqr);
            assert(rd.randomize() with {
                addr == 'h40000100;
                kind == 0;
                size == 2;
            });
            finish_item(rd);
            if (rd.data != 32'hCAFEBABE)
                `uvm_error("ERRINJ",
                    $sformatf("Test5: RAW mismatch: wrote=CAFEBABE read=%08h", rd.data))
            else
                `uvm_info("ERRINJ", "Test5: RAW hazard PASSED", UVM_MEDIUM)
        end

        // ═══════════════════════════════════════════════════════════════
        //  Test 6: 非法传输大小
        //
        //  场景：size=3 的写入（AXI4-Lite 仅支持 0/1/2，3 保留）
        //  期望：SLVERR(2'b10)
        //
        //  ★ AXI4-Lite 的 AxSIZE 合法值：0(1B), 1(2B), 2(4B)
        //    size=3(8B) 在 AXI Full 中合法，但在 AXI-Lite 中非法
        // ═══════════════════════════════════════════════════════════════
        tr = axi_transaction::type_id::create("tr");
        start_item(tr, , p_sequencer.axi_sqr);
        tr.addr = 'h40001000;
        tr.kind = 1;
        tr.size = 3;                     // ★ 非法 size（AXI-Lite max=2）
        finish_item(tr);
        check_resp(tr, "Test6: Invalid size", 2'b10);

        // ═══════════════════════════════════════════════════════════════
        //  Test 7: UART 批量发送（flood test）
        //
        //  场景：连续写 20 字节 (0x41='A', 0x42='B', 0x43='C', ...)
        //        到 UART DATA 寄存器(0x30000000)
        //  期望：全部返回 OKAY — 无丢字节、无死锁
        //
        //  ★ 验证点：UART TX 的 FIFO/缓冲是否能处理快速连续写入。
        //    如果 UART 没有 TX FIFO（当前版本），CPU 必须在 baud 周期内
        //    等待上次发送完成才能写下一字节。此测试验证这个"等待"是否正常。
        // ═══════════════════════════════════════════════════════════════
        `uvm_info("ERRINJ", "Test7: UART TX flood (20 writes)", UVM_MEDIUM)
        for (int i = 0; i < 20; i++) begin
            tr = axi_transaction::type_id::create("tr");
            start_item(tr, , p_sequencer.axi_sqr);
            assert(tr.randomize() with {
                addr == 'h30000000;       // UART DATA 寄存器
                kind == 1;
            });
            tr.data[7:0] = 8'h41 + i;    // ★ 'A' + i → 'A','B','C',...
            finish_item(tr);
            if (tr.resp != 2'b00)
                `uvm_error("ERRINJ",
                    $sformatf("Test7: UART %0d resp=%0b", i, tr.resp))
        end

        `uvm_info("ERRINJ", "All 7 error injection tests complete.", UVM_NONE)
    endtask

endclass
