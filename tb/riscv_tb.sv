// ============================================================
//  riscv_tb.sv — Sapphire SoC RV32IM Testbench
//
//  Architecture (post-upgrade):
//    DUT is now RV32IM + I$/D$ Caches + Bus Arbiter.
//    External AXI bus removed — bus is internal to SoC.
//    Verification strategy: software-driven via firmware
//    on BRAM, checking GPIO/UART output.
//
//  TODO: Update UVM env to software-driven test strategy
//    (replace AXI VIP with GPIO/UART monitors + firmware loader)
// ============================================================
`include "uvm_macros.svh"
import uvm_pkg::*;

module riscv_tb;
  bit clk = 0, resetn = 0;
  riscv_if dut_if(.clk(clk), .resetn(resetn));

  // ★ SoC external ports unchanged — GPIO + UART only
  riscv_soc dut (
    .clk(clk), .reset(~resetn),
    .gpio_out(dut_if.gpio_out), .gpio_in(dut_if.gpio_in),
    .uart_tx(dut_if.uart_tx), .uart_rx(dut_if.uart_rx)
  );

  initial begin clk=0; forever #5 clk=~clk; end
  initial begin #10ms; `uvm_fatal("TB", "Global simulation timeout after 10ms") $finish; end

  initial begin
    `ifdef VCS
      $fsdbDumpfile("novas.fsdb"); $fsdbDumpvars(0, riscv_tb);
      $fsdbDumpMDA(); $fsdbDumpSVA();
    `else
      $dumpfile("riscv_tb.vcd"); $dumpvars(0, riscv_tb);
    `endif
  end
  initial begin
    resetn = 0; #200; resetn = 1;
  end
  initial begin
    uvm_config_db#(virtual riscv_if)::set(null, "*", "vif", dut_if);
    run_test();
  end

  // ★ DEBUG: trace EX stage and D-Cache signals
  always @(posedge clk) begin
    if (dut.core.pipeline_stall == 1'b0) begin
      if (dut.mem_we)
      $display("[STORE] time=%0t PC=%h addr=%h wdata=%h gpio_out=%h",
               $time, dut.core.u_pc.pc_out, dut.mem_addr, dut.mem_wdata, dut.gpio_out);
    end
  end
endmodule
