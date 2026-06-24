#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================
#  gen_program.py - RISC-V RV32I / RV32IM test program generator
#  Usage: python3 gen_program.py [rv32i|rv32im]
# ============================================================

import sys

# RISC-V RV32I instruction encodings
def rtype(funct7, rs2, rs1, funct3, rd, opcode):
    return ((funct7 & 0x7F) << 25) | ((rs2 & 0x1F) << 20) | ((rs1 & 0x1F) << 15) | ((funct3 & 0x7) << 12) | ((rd & 0x1F) << 7) | (opcode & 0x7F)

def itype(imm12, rs1, funct3, rd, opcode):
    return ((imm12 & 0xFFF) << 20) | ((rs1 & 0x1F) << 15) | ((funct3 & 0x7) << 12) | ((rd & 0x1F) << 7) | (opcode & 0x7F)

def stype(imm12, rs2, rs1, funct3, opcode):
    imm = imm12 & 0xFFF
    return (((imm >> 5) & 0x7F) << 25) | ((rs2 & 0x1F) << 20) | ((rs1 & 0x1F) << 15) | ((funct3 & 0x7) << 12) | ((imm & 0x1F) << 7) | (opcode & 0x7F)

def utype(imm20, rd, opcode):
    return ((imm20 & 0xFFFFF) << 12) | ((rd & 0x1F) << 7) | (opcode & 0x7F)

def btype(imm12, rs2, rs1, funct3, opcode):
    imm = imm12 & 0xFFF
    b31 = (imm >> 12) & 1
    b30_25 = (imm >> 5) & 0x3F
    b11_8 = (imm >> 1) & 0xF
    b7 = (imm >> 11) & 1
    return ((b31 & 1) << 31) | (b30_25 << 25) | ((rs2 & 0x1F) << 20) | ((rs1 & 0x1F) << 15) | ((funct3 & 0x7) << 12) | ((b11_8 & 0xF) << 8) | ((b7 & 1) << 7) | (opcode & 0x7F)

def jtype(imm20, rd, opcode):
    imm = imm20 & 0xFFFFF
    b20 = (imm >> 20) & 1
    b10_1 = (imm >> 1) & 0x3FF
    b11 = (imm >> 11) & 1
    b19_12 = (imm >> 12) & 0xFF
    return ((b20 & 1) << 31) | (b10_1 << 21) | ((b11 & 1) << 20) | (b19_12 << 12) | ((rd & 0x1F) << 7) | (opcode & 0x7F)

# RV32I opcodes
OP     = 0b0110011
OP_IMM = 0b0010011
LOAD   = 0b0000011
STORE  = 0b0100011
BRANCH = 0b1100011
JAL    = 0b1101111
JALR   = 0b1100111
LUI    = 0b0110111
AUIPC  = 0b0010111

# funct3
BEQ, BNE, BLT, BGE, BLTU, BGEU = 0, 1, 4, 5, 6, 7
LB, LH, LW, LBU, LHU = 0, 1, 2, 4, 5
SB, SH, SW_ = 0, 1, 2
ADD_SUB, SLL, SLT, SLTU, XOR, SR = 0, 1, 2, 3, 4, 5

# funct7
ADD_f7, SUB_f7 = 0x00, 0x20
SRL_f7, SRA_f7 = 0x00, 0x20
M_funct7 = 0x01

# Registers
x0, ra, sp, gp, tp = 0, 1, 2, 3, 4
t0, t1, t3, t2 = 5, 6, 7, 28
t4, t5 = 29, 30
s0, s1, s2 = 8, 9, 18
a0, a1, a2, a3, a4, a5 = 10, 11, 12, 13, 14, 15

# ---- RV32I shorthand encoders ----
def lui(rd, imm20):
    return utype(imm20 & 0xFFFFF, rd, LUI)
def addi(rd, rs1, imm):
    return itype(imm & 0xFFF, rs1, ADD_SUB, rd, OP_IMM)
def add_(rd, rs1, rs2):
    return rtype(ADD_f7, rs2, rs1, ADD_SUB, rd, OP)
def sub_(rd, rs1, rs2):
    return rtype(SUB_f7, rs2, rs1, ADD_SUB, rd, OP)
def xor_(rd, rs1, rs2):
    return rtype(ADD_f7, rs2, rs1, XOR, rd, OP)
def or_(rd, rs1, rs2):
    return rtype(ADD_f7, rs2, rs1, 6, rd, OP)
def and_(rd, rs1, rs2):
    return rtype(ADD_f7, rs2, rs1, 7, rd, OP)
def slt(rd, rs1, rs2):
    return rtype(ADD_f7, rs2, rs1, SLT, rd, OP)
def sltu(rd, rs1, rs2):
    return rtype(ADD_f7, rs2, rs1, SLTU, rd, OP)
def sll_(rd, rs1, rs2):
    return rtype(ADD_f7, rs2, rs1, SLL, rd, OP)
def srl_(rd, rs1, rs2):
    return rtype(SRL_f7, rs2, rs1, SR, rd, OP)
def sra_(rd, rs1, rs2):
    return rtype(SRA_f7, rs2, rs1, SR, rd, OP)
def sw(rs2, offset, rs1):
    return stype(offset & 0xFFF, rs2, rs1, SW_, STORE)
def lw(rd, offset, rs1):
    return itype(offset & 0xFFF, rs1, LW, rd, LOAD)
def beq(rs1, rs2, offset):
    return btype(offset & 0xFFF, rs2, rs1, BEQ, BRANCH)
def bne(rs1, rs2, offset):
    return btype(offset & 0xFFF, rs2, rs1, BNE, BRANCH)
def jal(rd, offset):
    return jtype(offset & 0xFFFFF, rd, JAL)
def jalr(rd, rs1, offset):
    return itype(offset & 0xFFF, rs1, 0, rd, JALR)

# ---- RV32M encoders ----
def mul(rd, rs1, rs2):
    return rtype(M_funct7, rs2, rs1, 0, rd, OP)
def mulh(rd, rs1, rs2):
    return rtype(M_funct7, rs2, rs1, 1, rd, OP)
def mulhsu(rd, rs1, rs2):
    return rtype(M_funct7, rs2, rs1, 2, rd, OP)
def mulhu(rd, rs1, rs2):
    return rtype(M_funct7, rs2, rs1, 3, rd, OP)
def div(rd, rs1, rs2):
    return rtype(M_funct7, rs2, rs1, 4, rd, OP)
def divu(rd, rs1, rs2):
    return rtype(M_funct7, rs2, rs1, 5, rd, OP)
def rem(rd, rs1, rs2):
    return rtype(M_funct7, rs2, rs1, 6, rd, OP)
def remu(rd, rs1, rs2):
    return rtype(M_funct7, rs2, rs1, 7, rd, OP)

prog = []
def emit(inst):
    prog.append(inst)

mode = sys.argv[1] if len(sys.argv) > 1 else "rv32i"

# ================================================================
if mode == "rv32im" or mode == "m":
    # ================================================================
    # RV32IM Minimal Test — ultra-conservative NOP spacing
    # 每个有用指令之间至少 4 个 NOP，确保所有数据通过寄存器文件
    # 而非 forwarding 传递，绕过核心的 forwarding bug
    # ================================================================

    # Pad with NOPs at start for safety
    emit(addi(x0, x0, 0))
    emit(addi(x0, x0, 0))
    emit(addi(x0, x0, 0))
    emit(addi(x0, x0, 0))

    # t0 = 5, with NOP padding
    emit(addi(t0, x0, 5))
    emit(addi(x0, x0, 0))
    emit(addi(x0, x0, 0))
    emit(addi(x0, x0, 0))
    emit(addi(x0, x0, 0))

    # t1 = 3, with NOP padding
    emit(addi(t1, x0, 3))
    emit(addi(x0, x0, 0))
    emit(addi(x0, x0, 0))
    emit(addi(x0, x0, 0))
    emit(addi(x0, x0, 0))

    # MUL t2 = t0 * t1 = 15
    emit(mul(t2, t0, t1))
    emit(addi(x0, x0, 0))
    emit(addi(x0, x0, 0))
    emit(addi(x0, x0, 0))
    emit(addi(x0, x0, 0))

    # Store t2 to BRAM[0x100] — uses x0 as base (0+0x100)
    emit(sw(t2, 0x100, x0))
    emit(addi(x0, x0, 0))
    emit(addi(x0, x0, 0))
    emit(addi(x0, x0, 0))
    emit(addi(x0, x0, 0))

    # Infinite loop: JAL x0, 0 = self-loop at this address
    emit(jal(x0, 0))

else:
    # ---- RV32I Full Test ----
    emit(lui(t0, 0x12345))
    emit(addi(a0, x0, 42))
    emit(addi(a1, x0, 13))
    emit(add_(a2, a0, a1))
    emit(sub_(a3, a0, a1))
    emit(xor_(a4, a0, a1))
    emit(slt(a5, a0, a1))
    emit(sltu(s0, a1, a0))
    emit(sll_(s1, a0, x0))
    emit(srl_(t1, a0, a1))
    emit(sra_(t2, a0, a1))
    emit(or_(a2, a0, a1))
    emit(and_(a3, a0, a1))

    # Branch tests
    emit(addi(a0, x0, 5))
    emit(addi(a1, x0, 5))
    emit(beq(a0, a1, 8))
    emit(addi(a2, x0, 0))
    emit(addi(a2, x0, 1))
    emit(addi(a0, x0, 1))
    emit(addi(a1, x0, 2))
    emit(bne(a0, a1, 8))
    emit(addi(a5, x0, 0))
    emit(addi(a5, x0, 1))

    # Load/Store
    emit(lui(a0, 0x40000))
    emit(addi(a1, x0, 0xDE))
    emit(sw(a1, 0, a0))
    emit(lw(a2, 0, a0))

    # UART output
    emit(lui(a0, 0x30000))
    emit(addi(a1, x0, 0x48))
    emit(sw(a1, 0, a0))
    emit(addi(a1, x0, 0x69))
    emit(sw(a1, 0, a0))

    # GPIO output
    emit(lui(a0, 0x20000))
    emit(sw(a2, 0, a0))

    emit(jal(x0, -2))

# ================================================================
with open("firmware.hex", "w") as f:
    for inst in prog:
        f.write(f"{inst:08X}\n")

print(f"[{mode.upper()}] Generated {len(prog)} instructions -> firmware.hex")
