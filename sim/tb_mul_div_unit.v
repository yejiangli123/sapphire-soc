// ============================================================
//  tb_mul_div_unit.v — RV32M 乘除法单元独立验证
//  绕过所有流水线问题，直接验证 Bug 1.1 和 1.2 修复
// ============================================================
`include "Def.v"

module tb_mul_div_unit;
    reg         clk;
    reg         reset;
    reg  [31:0] op_a;
    reg  [31:0] op_b;
    reg  [3:0]  md_op;
    reg  [2:0]  funct3;
    reg         start;
    wire [31:0] result;
    wire        busy;

    mul_div_unit uut (
        .clk(clk),
        .reset(reset),
        .op_a(op_a),
        .op_b(op_b),
        .md_op(md_op),
        .funct3_in(funct3),
        .start(start),
        .result(result),
        .busy(busy)
    );

    always #5 clk = ~clk;

    integer pass, fail;
    initial begin
        clk = 0; reset = 1; op_a = 0; op_b = 0; md_op = 0; funct3 = 0; start = 0;
        pass = 0; fail = 0;
        #20 reset = 0;
        #10;

        // ============================================================
        // Test 1: MUL 5 * 3 = 15 (Bug 1.1 fix verification)
        // ============================================================
        op_a = 5; op_b = 3; md_op = `ALU_MUL; start = 1;
        #10 start = 0;
        if (result == 15 && !busy) begin
            $display("PASS Test 1 (MUL): 5*3 = %0d", result);
            pass = pass + 1;
        end else begin
            $display("FAIL Test 1 (MUL): expected 15, got %0d, busy=%b", result, busy);
            fail = fail + 1;
        end

        // ============================================================
        // Test 2: MULH — signed high bits (Bug 1.1)
        // 0x10000 * 0x20000 = 0x200000000 → high = 2
        // ============================================================
        op_a = 32'h00010000; op_b = 32'h00020000; md_op = `ALU_MULH; start = 1;
        #10 start = 0;
        if (result == 2 && !busy) begin
            $display("PASS Test 2 (MULH): high(%0d*%0d) = %0d", op_a, op_b, result);
            pass = pass + 1;
        end else begin
            $display("FAIL Test 2 (MULH): expected 2, got %0d", result);
            fail = fail + 1;
        end

        // ============================================================
        // Test 3: MUL with negative (-5 * 7 = -35)
        // ============================================================
        op_a = -5; op_b = 7; md_op = `ALU_MUL; start = 1;
        #10 start = 0;
        if ($signed(result) == -35 && !busy) begin
            $display("PASS Test 3 (MUL neg): (-5)*7 = %0d", $signed(result));
            pass = pass + 1;
        end else begin
            $display("FAIL Test 3 (MUL neg): expected -35, got %0d", $signed(result));
            fail = fail + 1;
        end

        // ============================================================
        // Test 4: DIV 10 / 3 = 3 (Bug 1.2 fix — must return QUOTIENT)
        // ============================================================
        op_a = 10; op_b = 3; md_op = `ALU_DIV; funct3 = `MULDIV_DIV; start = 1;
        #10 start = 0;
        wait (!busy);
        #10;
        if (result == 3) begin
            $display("PASS Test 4 (DIV): 10/3 = %0d", result);
            pass = pass + 1;
        end else begin
            $display("FAIL Test 4 (DIV): expected 3 (quotient), got %0d", result);
            fail = fail + 1;
        end

        // ============================================================
        // Test 5: REM 10 % 3 = 1 (Bug 1.2 fix — must return REMAINDER)
        // ============================================================
        op_a = 10; op_b = 3; md_op = `ALU_DIV; funct3 = `MULDIV_REM; start = 1;
        // Note: md_op = ALU_DIV (same as DIV), but funct3 = REM
        #10 start = 0;
        wait (!busy);
        #10;
        if (result == 1) begin
            $display("PASS Test 5 (REM): 10%%3 = %0d", result);
            pass = pass + 1;
        end else begin
            $display("FAIL Test 5 (REM): expected 1 (remainder), got %0d", result);
            fail = fail + 1;
        end

        // ============================================================
        // Test 6: DIV 100 / 7 = 14
        // ============================================================
        op_a = 100; op_b = 7; md_op = `ALU_DIV; funct3 = `MULDIV_DIV; start = 1;
        #10 start = 0;
        wait (!busy);
        #10;
        if (result == 14) begin
            $display("PASS Test 6 (DIV): 100/7 = %0d", result);
            pass = pass + 1;
        end else begin
            $display("FAIL Test 6 (DIV): expected 14, got %0d", result);
            fail = fail + 1;
        end

        // ============================================================
        // Test 7: REM 100 % 7 = 2
        // ============================================================
        op_a = 100; op_b = 7; md_op = `ALU_DIV; funct3 = `MULDIV_REM; start = 1;
        #10 start = 0;
        wait (!busy);
        #10;
        if (result == 2) begin
            $display("PASS Test 7 (REM): 100%%7 = %0d", result);
            pass = pass + 1;
        end else begin
            $display("FAIL Test 7 (REM): expected 2, got %0d", result);
            fail = fail + 1;
        end

        // ============================================================
        // Summary
        // ============================================================
        $display("=====================================");
        $display("RESULTS: %0d PASS, %0d FAIL (total %0d)", pass, fail, pass+fail);
        if (fail == 0)
            $display("ALL TESTS PASSED — Bug 1.1 & 1.2 fixes verified!");
        $display("=====================================");
        $finish;
    end
endmodule
