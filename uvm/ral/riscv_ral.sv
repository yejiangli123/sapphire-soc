// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  riscv_ral.sv — RAL 寄存器抽象层（Register Abstraction Layer）                 ║
// ║  ★ GPIO DIR/OUT + Timer Compare 寄存器模型 + 4 地址映射                        ║
// ╚══════════════════════════════════════════════════════════════════════════════════╝
//
// 【UVM RAL 是什么】
//   RAL（Register Abstraction Layer）是 UVM 提供的寄存器建模机制。
//   它把"按地址读写寄存器"抽象为"按名称读写字段"——
//   写代码时不需要关心寄存器在哪个地址、用哪个总线协议。
//
//   没有 RAL：
//     wr_trans(32'h20000004, 32'h000000FF);  // 写 GPIO OUT = 0xFF
//   有 RAL：
//     regmodel.gpio_out.out.write(status, 32'hFF);  // 语义清晰，不关心地址
//
// 【RAL 的好处】
//   1. 抽象总线协议——同一套寄存器模型可以用于 AXI/AHB/APB 等不同总线
//   2. 自动生成前门/后门访问序列（通过 adapter 转换）
//   3. 内建寄存器镜像——自动跟踪"DUT 中寄存器的预期值"
//   4. 支持 mirrored/desired value 比对——寄存器值是否和预期一致
//
// 【本 RAL 模型的组件】
//   ┌─────────────────────────────────────────────────────┐
//   │  riscv_reg_block（顶层块）                           │
//   │  ├── gpio_map  (base=0x20000000)                     │
//   │  │   ├── gpio_dir_reg   @ offset 0x00                │
//   │  │   │   └── dir[31:0]  (RW, reset=0)                │
//   │  │   └── gpio_out_reg   @ offset 0x04                │
//   │  │       └── out[31:0]  (RW, reset=0)                │
//   │  ├── timer_map (base=0x40000000)                     │
//   │  │   └── timer_compare_reg @ offset 0x04             │
//   │  │       └── compare[31:0] (RW, reset=0xFFFFFFFF)    │
//   │  ├── uart_map  (base=0x30000000) — 空（预留）        │
//   │  └── plic_map  (base=0x50000000) — 空（预留）        │
//   └─────────────────────────────────────────────────────┘
//
// 【当前状态】
//   ★ RAL 模型已创建，但 **predictor 和 adapter 未实例化**。
//     这意味着寄存器模型不会自动从总线 Monitor 更新镜像值——是已知 gap。
//   ★ 完整集成需要：
//     1. 创建 uvm_reg_adapter（AXI transaction ↔ uvm_reg_bus_op）
//     2. 创建 uvm_reg_predictor（监听 AXI Monitor，自动更新镜像）
//     3. 在 env 中连接 predictor → adapter → bus monitor
//
// 【面试要点】
//   Q: RAL 的 adapter 是干什么的？
//   A: adapter 是桥接器——把 uvm_reg_bus_op（寄存器操作：读/写 地址+数据）
//      转换成具体总线协议的 transaction（如 axi_transaction）。
//      reg2bus()：寄存器操作 → 总线事务
//      bus2reg()：总线事务 → 寄存器操作
// ═══════════════════════════════════════════════════════════════════════════════

`include "uvm_macros.svh"
import uvm_pkg::*;

// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  gpio_dir_reg — GPIO 方向寄存器（offset 0x00）                               ║
// ║  ★ 32-bit RW，reset = 0（所有引脚默认输入）                                   ║
// ╚══════════════════════════════════════════════════════════════════════════════╝
class gpio_dir_reg extends uvm_reg;
    `uvm_object_utils(gpio_dir_reg)
    rand uvm_reg_field dir;                // ★ 方向字段（32-bit，每 bit 对应一个引脚）

    function new(string n = "gpio_dir_reg");
        super.new(n, 32, UVM_NO_COVERAGE); // ★ 32-bit 寄存器，不收集覆盖率
    endfunction

    virtual function void build();
        dir = uvm_reg_field::type_id::create("dir");
        // ★ configure 参数：
        //   parent, width, lsb, access, volatile, reset, has_reset, is_rand, individually_accessible
        dir.configure(this, 32, 0, "RW", 0, 0, 1, 1, 1);
        //   32-bit, LSB=0, RW, 不挥发, reset=0, 有复位值, 可随机, 可独立访问
    endfunction
endclass

// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  gpio_out_reg — GPIO 输出寄存器（offset 0x04）                                ║
// ║  ★ 32-bit RW，reset = 0                                                      ║
// ╚══════════════════════════════════════════════════════════════════════════════╝
class gpio_out_reg extends uvm_reg;
    `uvm_object_utils(gpio_out_reg)
    rand uvm_reg_field out;

    function new(string n = "gpio_out_reg");
        super.new(n, 32, UVM_NO_COVERAGE);
    endfunction

    virtual function void build();
        out = uvm_reg_field::type_id::create("out");
        out.configure(this, 32, 0, "RW", 0, 0, 1, 1, 1);
    endfunction
endclass

// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  timer_compare_reg — Timer 比较寄存器（offset 0x04）                          ║
// ║  ★ 32-bit RW，reset = 0xFFFFFFFF（最大计数值）                                 ║
// ╚══════════════════════════════════════════════════════════════════════════════╝
class timer_compare_reg extends uvm_reg;
    `uvm_object_utils(timer_compare_reg)
    rand uvm_reg_field compare;

    function new(string n = "timer_compare_reg");
        super.new(n, 32, UVM_NO_COVERAGE);
    endfunction

    virtual function void build();
        compare = uvm_reg_field::type_id::create("compare");
        compare.configure(this, 32, 0, "RW", 0, 'hFFFFFFFF, 1, 1, 1);
        // ★ reset = 0xFFFFFFFF：Timer 上电后计数从 0 开始，默认比较值为最大值
    endfunction
endclass

// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  riscv_reg_block — SoC 寄存器块（顶层容器）                                   ║
// ║  ★ 创建 4 个地址映射 + 3 个寄存器 + lock_model()                              ║
// ╚══════════════════════════════════════════════════════════════════════════════╝
class riscv_reg_block extends uvm_reg_block;
    `uvm_object_utils(riscv_reg_block)

    // ── 地址映射（每个外设一个 map）──
    rand uvm_reg_map gpio_map, uart_map, timer_map, plic_map;

    // ── 寄存器实例 ──
    rand gpio_dir_reg        gpio_dir;
    rand gpio_out_reg        gpio_out;
    rand timer_compare_reg   timer_compare;

    function new(string n = "riscv_reg_block");
        super.new(n, UVM_NO_COVERAGE);       // ★ 寄存器块不收集覆盖率
    endfunction

    virtual function void build();
        // ★ default_map 是 UVM 强制要求的——即使 BRAM 不是寄存器也需要一个默认 map
        default_map = create_map("default_map", 'h00000000, 4, UVM_LITTLE_ENDIAN);

        // ★ 每个外设创建一个独立的地址映射
        gpio_map  = create_map("gpio_map",  'h20000000, 4, UVM_LITTLE_ENDIAN);
        uart_map  = create_map("uart_map",  'h30000000, 4, UVM_LITTLE_ENDIAN);
        timer_map = create_map("timer_map", 'h40000000, 4, UVM_LITTLE_ENDIAN);
        plic_map  = create_map("plic_map",  'h50000000, 4, UVM_LITTLE_ENDIAN);
        // ★ 参数：(名称, 基地址, 地址宽度(byte), 字节序)

        // ── GPIO 寄存器 ──
        gpio_dir = gpio_dir_reg::type_id::create("gpio_dir");
        gpio_dir.configure(this, null, "");  // ★ configure 后再 build
        gpio_dir.build();
        gpio_map.add_reg(gpio_dir, 'h00, "RW");  // ★ 偏移 0x00

        gpio_out = gpio_out_reg::type_id::create("gpio_out");
        gpio_out.configure(this, null, "");
        gpio_out.build();
        gpio_map.add_reg(gpio_out, 'h04, "RW");  // ★ 偏移 0x04

        // ── Timer 寄存器 ──
        timer_compare = timer_compare_reg::type_id::create("timer_compare");
        timer_compare.configure(this, null, "");
        timer_compare.build();
        timer_map.add_reg(timer_compare, 'h04, "RW");

        // ★ lock_model() 之后不可再添加寄存器——确保模型完整性
        lock_model();
    endfunction

endclass
