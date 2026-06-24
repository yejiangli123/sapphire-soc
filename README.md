# Sapphire SoC — RV32IM RISC-V Processor SoC

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

A RV32IM 5-stage pipelined RISC-V processor SoC with UVM 1.2 verification environment, developed from scratch in Verilog and SystemVerilog.

## Architecture

```
┌───────────────────────────────────────────────────┐
│                  Sapphire SoC                     │
│                                                   │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐          │
│  │ I-Cache │  │ D-Cache │  │  BRAM   │          │
│  │  (2KB)  │  │  (2KB)  │  │ (Boot)  │          │
│  └────┬────┘  └────┬────┘  └────┬────┘          │
│       │            │            │                 │
│  ┌────┴────────────┴────────────┴────┐           │
│  │        RV32IM Core                │           │
│  │  IF → ID → EX → MEM → WB         │           │
│  │  Forwarding + Hazard + Branch     │           │
│  └────────────────┬──────────────────┘           │
│                   │                              │
│  ┌────────────────┴──────────────────┐           │
│  │        AXI4 Interconnect          │           │
│  └───┬───────┬────────┬─────────┬────┘           │
│      │       │        │         │                 │
│  ┌───┴──┐ ┌──┴──┐ ┌───┴───┐ ┌──┴───┐           │
│  │ GPIO │ │UART │ │ Timer │ │ PLIC │           │
│  └──────┘ └─────┘ └───────┘ └──────┘           │
└───────────────────────────────────────────────────┘
```

## Features

- **ISA**: RV32I (base integer) + M (multiply/divide) — 48 instructions
- **Pipeline**: 5-stage (IF/ID/EX/MEM/WB) with 3-way forwarding, load-use stall, static branch prediction
- **Cache**: 2KB direct-mapped I-Cache and D-Cache with Critical-Word First fill
- **Bus**: AXI4 interconnect with round-robin arbiter
- **Peripherals**: GPIO, UART (with TX/RX), Timer, PLIC interrupt controller
- **Verification**: UVM 1.2 layered environment with Active/Passive agents, scoreboards, coverage, SVA assertions

## Project Structure

```
sapphire-soc/
├── rtl/               # Verilog RTL (33 modules, ~8000 lines)
│   ├── rv32i_core.v   # Top-level core
│   ├── Program_counter.v
│   ├── ALU.v, ALU_Control.v
│   ├── Control_unit.v, Decoder.v
│   ├── Register_file.v
│   ├── icache.v, dcache.v
│   ├── bus_arbiter.v, axi4_interconnect.v
│   ├── GPIO.v, uart_complete.v, Timer.v, PLIC.v
│   └── ...
├── uvm/               # UVM verification (26 files, ~3000 lines)
│   ├── agent/         # AXI, GPIO, UART agents
│   ├── env/           # Environment, scoreboard, ref model, coverage
│   ├── seq/           # Virtual sequences
│   ├── test/          # Test library
│   └── ral/           # Register model
├── tb/                # Testbench top
├── sim/               # Simulation scripts
│   ├── Makefile       # VCS build/run/regress/cov
│   └── gen_program.py # Firmware generator
├── sw/                # Firmware (assembly test programs)
├── sv_labs/           # SystemVerilog lab exercises (learning path)
└── archive/           # Archived old versions
```

## Quick Start

### Prerequisites

- Synopsys VCS with UVM-1.2
- Synopsys Verdi (for waveform debug)
- Python 3 (for firmware generation)

### Build & Run

```bash
cd sim
make compile          # Compile RTL + UVM
make run              # Run default test
make smoke            # Quick smoke check
make regress          # Multi-seed regression
make cov              # Coverage collection + merge
```

### Run a Specific Test

```bash
make run TEST=firmware_m_smoke_test SEED=42
```

## Verification Highlights

| Metric | Value |
|--------|-------|
| Tests | 7 directed + constrained-random sequences |
| Coverage (code) | 92% (line 96%, toggle 89%, FSM 100%) |
| Coverage (functional) | 96% |
| SVA assertions | 12 AXI4-Lite protocol checks |
| Key bugs found | MUL routing, DIV sign handling, Cache CWF timing |

## License

MIT — see [LICENSE](LICENSE)

---

*Designed and verified as an undergraduate capstone project. All code has line-by-line Chinese comments and accompanying design documentation.*
