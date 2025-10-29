# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is a fork of XiangShan (香山), an open-source high-performance RISC-V processor, with extensions for **DASICS** (Dynamic in-Address-Space Isolation by Code Segments) and **Zicfilp** (Control-Flow Integrity Landing Pads). The project is developed using Chisel (a hardware construction language embedded in Scala) and generates Verilog for hardware simulation and synthesis.

**Key custom features in this fork:**
- **DASICS**: Memory and jump boundary checking for isolation (see `src/main/scala/xiangshan/backend/fu/Dasics.scala`)
- **Zicfilp**: RISC-V CFI extension for landing pad support (check files with "zicfilp" or "cfi" patterns)
- Based on the Nanhu (南湖) microarchitecture

## Build System

This project uses **Mill** as the primary build tool for Scala/Chisel, with **Make** as the top-level orchestrator.

### Environment Setup

Required environment variables:
- `NEMU_HOME`: Absolute path to NEMU project (RISC-V emulator for co-simulation)
- `NOOP_HOME`: Absolute path to this XiangShan project directory
- `AM_HOME`: Absolute path to the AM (Abstract Machine) project

### Common Commands

#### Initial Setup
```bash
make init                    # Initialize git submodules
mill -i mill.bsp.BSP/install # Generate BSP for IDE support
mill -i mill.scalalib.GenIdea/idea # Generate IntelliJ IDEA project files
```

#### Verilog Generation
```bash
make verilog                 # Generate XSTop.v for FPGA (output: build/XSTop.v)
make sim-verilog             # Generate SimTop.v for simulation (output: build/SimTop.v)

# With custom configuration
make verilog CONFIG=MinimalConfig NUM_CORES=1
```

**Configuration options:**
- `CONFIG`: Chisel configuration class (default: `DefaultConfig`)
  - Other options: `MinimalConfig`, `DefaultConfig`, etc.
- `NUM_CORES`: Number of cores to generate (default: 1)
- `SIM_ARGS`: Additional simulation arguments (e.g., `--enable-difftest`, `--disable-all`)

#### Simulation (Verilator)
```bash
make emu                     # Build C++ emulator with Verilator (output: build/emu)
make emu CONFIG=MinimalConfig EMU_THREADS=2 -j10  # Parallel build with config

# Run simulation
./build/emu -b 0 -e 0 -i ./ready-to-run/coremark-2-iteration.bin \
            --diff ./ready-to-run/riscv64-nemu-interpreter-so

make emu-run                 # Build and run emulator
```

The emulator is built by the difftest submodule and supports DiffTest (differential testing against NEMU).

#### Simulation (VCS)
```bash
make simv_rtl                # Build VCS RTL simulator
make simv_rtl-run RUN_BIN=coremark-2-iteration # Run VCS simulation
make verdi_rtl RUN_BIN=coremark-2-iteration    # Open Verdi for waveform debugging
```

**VCS options:**
- `RUN_BIN`: Test binary name from `ready-to-run/` (without .bin extension)
- `CONSIDER_FSDB`: Enable FSDB waveform dumping (default: 1)

#### Cleanup
```bash
make clean                   # Clean build artifacts
```

## Code Architecture

### Directory Structure

```
src/main/scala/
├── xiangshan/              # Core processor implementation
│   ├── frontend/           # Instruction fetch, branch prediction, ICache
│   ├── backend/            # Execution backend
│   │   ├── decode/         # Instruction decode (includes Zicfilp decode logic)
│   │   ├── rename/         # Register renaming
│   │   ├── dispatch/       # Instruction dispatch
│   │   ├── issue/          # Issue queues
│   │   ├── exu/            # Execution units
│   │   ├── fu/             # Functional units (ALU, FPU, CSR, Jump, etc.)
│   │   │   ├── CSR.scala   # CSR file (includes DASICS/Zicfilp CSR support)
│   │   │   ├── Dasics.scala # DASICS implementation
│   │   │   └── Jump.scala  # Jump unit (CFI checking)
│   │   ├── regfile/        # Register files
│   │   └── rob/            # Reorder buffer
│   ├── mem/                # Memory subsystem (LSU, store buffer, load queue)
│   ├── cache/              # Cache hierarchy (DCache, ICache, PTW, TLB, MMU)
│   ├── Bundle.scala        # IO bundle definitions (check for CFI-related signals)
│   ├── Parameters.scala    # Configuration parameters (HasDasics, HasNExtension flags)
│   └── XSCore.scala        # Core module integration
├── top/                    # SoC top-level
│   └── Top.scala           # XSTop and system integration
├── system/                 # SoC components (interconnect, peripherals)
├── device/                 # Virtual devices (CLINT, PLIC, etc.)
└── utils/                  # Utility modules

Submodules:
├── difftest/               # Co-simulation framework with NEMU
├── huancun/                # L2/L3 cache (HuanCun)
├── fudian/                 # Floating-point unit
└── rocket-chip/            # RocketChip dependencies (Diplomacy, TileLink)
```

### Key Architectural Concepts

**Frontend:** Instruction fetch starts at `frontend/IFU.scala`. The frontend includes a sophisticated branch prediction unit (BPU) with FTB (Fetch Target Buffer), TAGE-SC predictor, and RAS (Return Address Stack). For Zicfilp, check `frontend/PreDecode.scala` for landing pad pre-decode logic.

**Backend:** The backend is out-of-order with separate integer/FP/LSU execution pipelines. Decode stage (`backend/decode/DecodeUnit.scala`) handles instruction decoding including CFI extensions. The backend uses a distributed issue queue design.

**Memory System:** Load/Store Unit is in `mem/`, with separated load/store queues. The TLB and PTW are in `cache/mmu/`. DASICS memory checking is integrated into the LSU pipeline.

**CSR and Extensions:**
- CSR implementation in `backend/fu/CSR.scala` includes M/S/U privilege levels
- DASICS CSRs for boundary configuration
- Zicfilp CSRs (SPELP, MPELP in mstatus/sstatus)

**DASICS Implementation:**
- Memory bounds: 16 configurable bounds for load/store checking
- Jump bounds: 4 configurable bounds for indirect jumps
- Granularity: 8-byte aligned boundaries
- Integration points: LSU pipeline, Jump unit, CSR file

**Zicfilp (CFI) Implementation:**
- Landing pad checking in frontend and backend
- Integration with branch prediction and jump execution
- CSR support for enable/disable
- 可以参照../NEMU理解Zicfilp的功能

### Module Hierarchy

`XSTop` (in `top/Top.scala`) is the top-level SoC, containing:
- `XSTile`: One or more processor tiles, each with:
  - `XSCore`: The processor core
    - `Frontend`: IFU + BPU
    - `CtrlBlock`: Control logic (decode, rename, dispatch)
    - `MemBlock`: LSU and DCache
    - `ExuBlock`: Execution units (ALU, MDU, Jump, FPU, etc.)
  - L2 cache (per tile)
- `HuanCun`: L3 cache (optional, shared)
- `SoCMisc`: Peripherals (CLINT, PLIC, debug module)

### Working with Chisel/Scala

- Chisel hardware modules extend `Module`, `XSModule`, or `LazyModule` (for Diplomacy)
- `Parameters` (from RocketChip) configure the design via implicit `p: Parameters`
- Look for `XSCoreParameters` in `Parameters.scala` for core config flags
- Hardware types: `UInt`, `Bool`, `Bundle`, `Vec`
- Connections: `:=` (mono-directional), `<>` (bi-directional)

### Development Branch Strategy

- Main development branch: `dasics-master`
- Current working branch: `dev-zicfilp` (for Zicfilp feature development)
- This repository tracks RISC-V CFI (Control-Flow Integrity) research

## Testing and Debugging

### DiffTest
The processor is tested using DiffTest, which compares execution against NEMU (a golden RISC-V emulator):
- Enable with `--enable-difftest` in SIM_ARGS
- NEMU reference shared object: `$NEMU_HOME/build/riscv64-nemu-interpreter-so`
- DiffTest code is in the `difftest/` submodule

### Waveform Debugging
- For VCS: Use `make verdi_rtl` to open Verdi with FSDB waveforms
- For Verilator: VCD waveforms can be enabled (see simulator help)

## Related Projects

- **NEMU** (https://github.com/OpenXiangShan/NEMU): RISC-V emulator for golden model testing
- **XiangShan-doc** (https://github.com/OpenXiangShan/XiangShan-doc): Official documentation
- Upstream: https://github.com/OpenXiangShan/XiangShan

## Important Notes

- When modifying DASICS or Zicfilp features, search for existing usage patterns first
- CSR changes must be coordinated with `backend/fu/util/CSRConst.scala` for CSR addresses
- Bundle changes (I/O interfaces) often require updates in multiple pipeline stages
- Always rebuild Verilog after Scala changes: `make clean && make sim-verilog`
- Simulation can take significant time; use `CONFIG=MinimalConfig` for faster testing during development
- The build allocates significant memory: Mill uses `-Xmx64G` for JVM heap
- 在生成git commit的时候不要带有claude信息
- 修改代码的时候审查附近的注释是否依旧正确、
- 代码实现的时候尽可能与已经有的代码框架相耦合
- 使用中文和我交互
- zicfilp功能和dasics功能是相互独立的功能，架构设计可以参考，但是功能不要相互耦合
- 代码注释请使用英文
- 如果添加了新的端口，一定要检查模块与模块之间的端口是否已经在上层模块成功连接
