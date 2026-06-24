# ╔══════════════════════════════════════════════════════════════════╗
# ║  Makefile — Sapphire SoC RV32IM UVM 验证编译脚本              ║
# ║  Upgraded: M-extension + I$/D$ Caches + Bus Arbiter        ║
# ╚══════════════════════════════════════════════════════════════════╝

# ── 工具配置 ──
VCS        ?= vcs
SIMV       := ./simv
VERDI      ?= verdi
URG        ?= urg
TOP        := riscv_tb
TEST       ?= firmware_m_smoke_test
VERBOSITY  ?= UVM_MEDIUM
SEED       ?= 1
LOG_DIR    := logs
COV_DIR    := simv.vdb
COV_BROWSER ?= xdg-open
PYTHON     ?= python3

VCS_OPTS   = -sverilog -ntb_opts uvm-1.2 -full64 -debug_access+all -kdb \
             +vcs+lic+wait -timescale=1ns/10ps +define+VCS \
             +incdir+../rtl +incdir+../uvm/agent +incdir+../uvm/env \
             +incdir+../uvm/seq +incdir+../uvm/ral +incdir+../uvm/test +incdir+../tb

# ★ 覆盖率开关（COV=0 关掉）
COV ?= 1
ifeq ($(COV),1)
  COV_COMP_OPTS = -cm line+tgl+branch+fsm -cm_dir $(COV_DIR)
  COV_SIM_OPTS  = -cm line+tgl+branch+fsm -cm_dir $(COV_DIR) -cm_name $(TEST)
else
  COV_COMP_OPTS =
  COV_SIM_OPTS  =
endif

RTL_DIR    = ../rtl
UVM_DIR    = ../uvm
TB_DIR     = ../tb

RTL_FILES  = \
  $(RTL_DIR)/rv32i_core.v $(RTL_DIR)/Program_counter.v $(RTL_DIR)/Control_unit.v \
  $(RTL_DIR)/ALU.v $(RTL_DIR)/ALU_Control.v $(RTL_DIR)/Register_file.v \
  $(RTL_DIR)/Decoder.v $(RTL_DIR)/Immediate_Generator.v $(RTL_DIR)/Branch_unit.v \
  $(RTL_DIR)/Branch_predicting_unit.v $(RTL_DIR)/Bypass_unit.v $(RTL_DIR)/Hazard_Detection.v \
  $(RTL_DIR)/Stall.v $(RTL_DIR)/IF_IDpipeline.v $(RTL_DIR)/ID_EXpipeline.v \
  $(RTL_DIR)/EX_Mempipeline.v $(RTL_DIR)/mem_pipeline.v $(RTL_DIR)/wb_pipeline.v \
  $(RTL_DIR)/Def.v $(RTL_DIR)/Load_unit.v $(RTL_DIR)/Store_unit.v \
  $(RTL_DIR)/Inst_memory.v $(RTL_DIR)/Data_mem.v $(RTL_DIR)/bram.v \
  $(RTL_DIR)/mul_div_unit.v $(RTL_DIR)/icache.v $(RTL_DIR)/dcache.v \
  $(RTL_DIR)/bus_arbiter.v $(RTL_DIR)/axi4_interconnect.v \
  $(RTL_DIR)/axi_lite_manager.v $(RTL_DIR)/GPIO.v $(RTL_DIR)/uart_rx.v \
  $(RTL_DIR)/uart_complete.v $(RTL_DIR)/Timer.v $(RTL_DIR)/PLIC.v \
  $(RTL_DIR)/TOP_levelSOc.v $(RTL_DIR)/system_bus.v

UVM_FILES  = \
  $(UVM_DIR)/agent/axi_transaction.sv $(UVM_DIR)/agent/riscv_if.sv \
  $(UVM_DIR)/agent/axi_monitor.sv $(UVM_DIR)/agent/axi_driver.sv \
  $(UVM_DIR)/agent/axi_sequencer.sv $(UVM_DIR)/agent/axi_agent.sv \
  $(UVM_DIR)/agent/axi_slave_vip.sv \
  $(UVM_DIR)/agent/gpio_agent.sv $(UVM_DIR)/agent/uart_agent.sv \
  $(UVM_DIR)/agent/gpio_monitor.sv \
  $(UVM_DIR)/ral/riscv_ral.sv $(UVM_DIR)/seq/riscv_vseq_lib.sv \
  $(UVM_DIR)/seq/firmware_seq_lib.sv \
  $(UVM_DIR)/env/riscv_ref_model.sv $(UVM_DIR)/env/riscv_coverage.sv \
  $(UVM_DIR)/env/enhanced_coverage.sv $(UVM_DIR)/env/riscv_scoreboard.sv \
  $(UVM_DIR)/env/riscv_env.sv $(UVM_DIR)/env/firmware_scoreboard.sv \
  $(UVM_DIR)/seq/error_injection_seq.sv \
  $(UVM_DIR)/test/riscv_test_lib.sv $(UVM_DIR)/test/firmware_test_lib.sv

TB_FILE    = $(TB_DIR)/riscv_tb.sv

# ── Targets ──
.PHONY: compile run sim prog prog_m \
        cov cov_view_test cov_view_path cov_view_dve cov_merge cov_view \
        verdi smoke regression clean help

# ===== 编译+仿真 =====
all: compile

compile:
	@echo "=== VCS Compiling Sapphire SoC UVM ==="
	mkdir -p $(LOG_DIR)
	$(VCS) $(VCS_OPTS) $(COV_COMP_OPTS) $(RTL_FILES) $(UVM_FILES) $(TB_FILE) \
	       -top $(TOP) -o $(SIMV) -l $(LOG_DIR)/elab.log
	@echo "=== Compile Done ==="

run: compile
	@echo "=== Running Test: $(TEST) seed=$(SEED) ==="
	mkdir -p $(LOG_DIR)
	$(SIMV) +UVM_TESTNAME=$(TEST) +ntb_random_seed=$(SEED) \
	        +UVM_VERBOSITY=$(VERBOSITY) $(COV_SIM_OPTS) -l $(LOG_DIR)/$(TEST).log

sim: run

# ===== 覆盖率（URG 报告）=====
cov:
	urg -dir $(COV_DIR) -format both -report cov_report
	@echo "覆盖率报告: cov_report/dashboard.html"

# 当前测试报告 + 浏览器打开
cov_view_test: cov
	@html=""; \
	if [ -f cov_report/dashboard.html ]; then html=cov_report/dashboard.html; \
	elif [ -f cov_report/urgReport/dashboard.html ]; then html=cov_report/urgReport/dashboard.html; \
	fi; \
	[ -z "$$html" ] && { echo "[COV] 未找到 dashboard.html"; exit 1; }; \
	$(COV_BROWSER) "$$html" >/dev/null 2>&1 & true; \
	echo "[COV] 打开 $$html"

# 仅打印路径（远程 VM 无 GUI）
cov_view_path:
	@html=""; \
	if [ -f cov_merge/urgReport/dashboard.html ]; then html=cov_merge/urgReport/dashboard.html; \
	elif [ -f cov_merge/dashboard.html ]; then html=cov_merge/dashboard.html; \
	fi; \
	[ -z "$$html" ] && { echo "[COV] 请先 make cov_merge"; exit 1; }; \
	abs=$$(cd "$$(dirname "$$html")" && pwd)/$$(basename "$$html"); \
	echo "file://$$abs"

# DVE 交互式覆盖率
cov_view_dve:
	dve -full64 -cov -covdir $(COV_DIR) &

# 合并所有 *.vdb
cov_merge:
	@VDB_LIST=$$(find . \( -path ./csrc -o -path ./verdiLog \) -prune -o \
	             -type d -name '*.vdb' -print 2>/dev/null | sort -u); \
	[ -z "$$VDB_LIST" ] && { echo "[COV] 未找到 *.vdb，请先 make run"; exit 1; }; \
	echo "[COV] 合并 VDB → cov_merge/"; \
	urg -dir $$(echo "$$VDB_LIST" | tr '\n' ' ') -format both -report cov_merge

# 合并报告 + 浏览器打开
cov_view: cov_merge
	@html=""; \
	if [ -f cov_merge/urgReport/dashboard.html ]; then html=cov_merge/urgReport/dashboard.html; \
	elif [ -f cov_merge/dashboard.html ]; then html=cov_merge/dashboard.html; \
	fi; \
	[ -z "$$html" ] && { echo "[COV] 未找到 dashboard.html"; exit 1; }; \
	$(COV_BROWSER) "$$html" >/dev/null 2>&1 & true; \
	echo "[COV] 打开 $$html"

# ===== Verdi（波形 + 覆盖率）=====
verdi:
	@vdb=""; \
	if [ -d "$(COV_DIR)" ]; then \
	  vdb=$$(find "$(COV_DIR)" -maxdepth 12 -type d -name '*.vdb' 2>/dev/null | head -1); \
	  [ -z "$$vdb" ] && vdb="$(COV_DIR)"; \
	fi; \
	if [ -n "$$vdb" ]; then \
	  echo "[Verdi] -ssf novas.fsdb -cov -covdir $$vdb"; \
	  $(VERDI) -ssf novas.fsdb -cov -covdir $$vdb -nologo & \
	else \
	  $(VERDI) -ssf novas.fsdb -nologo & \
	fi

# ===== 固件生成 =====
prog:
	$(PYTHON) gen_program.py rv32i

prog_m:
	$(PYTHON) gen_program.py rv32im

# ===== 冒烟 + 回归 =====
smoke: compile prog
	@tests="firmware_m_smoke_test firmware_cache_stress_test"; \
	pass=0; total=0; \
	for t in $$tests; do \
	  total=$$((total+1)); \
	  log="$(LOG_DIR)/smoke_$$t.log"; \
	  echo "[SMOKE] $$t"; \
	  ./$(SIMV) +UVM_TESTNAME=$$t +ntb_random_seed=$(SEED) \
	    +UVM_VERBOSITY=UVM_LOW $(COV_SIM_OPTS) -cm_name $$t -l $$log; \
	  if grep -q "TEST.*PASSED" $$log; then \
	    status=PASS; pass=$$((pass+1)); \
	  else \
	    status=FAIL; \
	  fi; \
	  echo "  $$t → $$status"; \
	done; \
	echo "pass=$$pass/$$total"

regression: compile prog prog_m
	@pass=0; total=0; \
	for s in $(SEEDS); do \
	  total=$$((total+1)); \
	  log="$(LOG_DIR)/reg_seed$$s.log"; \
	  echo "[REG] $$s"; \
	  ./$(SIMV) +UVM_TESTNAME=firmware_m_smoke_test +ntb_random_seed=$$s \
	    +UVM_VERBOSITY=UVM_LOW $(COV_SIM_OPTS) -cm_name reg_seed$$s -l $$log; \
	  if grep -q "TEST.*PASSED" $$log; then \
	    status=PASS; pass=$$((pass+1)); \
	  else \
	    status=FAIL; \
	  fi; \
	  echo "  seed=$$s → $$status"; \
	done; \
	echo "pass=$$pass/$$total"

# ===== 帮助 + 清理 =====
help:
	@echo "========================================================"
	@echo "  Sapphire SoC RV32IM UVM Makefile"
	@echo "========================================================"
	@echo "  make compile          - VCS 编译"
	@echo "  make run              - 编译 + 运行"
	@echo "  make run TEST=xxx     - 运行指定测试"
	@echo "  make run SEED=n       - 指定种子"
	@echo ""
	@echo "  【固件生成】"
	@echo "  make prog             - 生成 RV32I 测试固件"
	@echo "  make prog_m           - 生成 RV32IM 测试固件(M扩展)"
	@echo ""
	@echo "  【覆盖率】"
	@echo "  make cov              - 覆盖率报告"
	@echo "  make cov_view_test    - 报告 + 浏览器"
	@echo "  make cov_merge        - 合并 VDB"
	@echo "  make cov_view         - 合并 + 浏览器"
	@echo ""
	@echo "  make verdi            - Verdi 波形 + 覆盖率"
	@echo "  make smoke            - 所有测试冒烟"
	@echo "  make regression       - 多 seed 回归"
	@echo "  make clean            - 清理"
	@echo "========================================================"

clean:
	rm -rf simv simv.daidir csrc vc_hdrs.h ucli.key novas.* verdiLog \
	       $(COV_DIR) cov_report cov_merge coverage_report \
	       *.log $(LOG_DIR) *.vpd *.vcd *.fsdb DVEfiles
