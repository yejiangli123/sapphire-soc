#!/bin/bash
for p in /opt/synopsys/setup.sh /eda/synopsys/setup.sh; do [ -f "$p" ] && source "$p" && break; done
mkdir -p logs
TESTS=(riscv_base_test riscv_axi_smoke_test riscv_axi_stress_test riscv_error_injection_test)
PASS=0; FAIL=0; TOTAL=${#TESTS[@]}
for TEST in "${TESTS[@]}"; do
  echo "=== [$((PASS+FAIL+1))/$TOTAL] $TEST ==="
  timeout 600 ./simv +UVM_TESTNAME=$TEST +UVM_VERBOSITY=UVM_LOW -l logs/${TEST}.log -cm line+cond+fsm+tgl -cm_name $TEST -cm_dir simv.vdb > logs/${TEST}_stdout.log 2>&1
  if grep -qE "UVM_FATAL|UVM_ERROR" logs/${TEST}.log 2>/dev/null; then echo "  FAIL: $TEST"; ((FAIL++))
  else echo "  PASS: $TEST"; ((PASS++)); fi
done
echo "========================================="
echo "Regression: $PASS/$TOTAL passed, $FAIL failed"
urg -dir simv.vdb -report merged_coverage -format both 2>/dev/null
exit $FAIL
