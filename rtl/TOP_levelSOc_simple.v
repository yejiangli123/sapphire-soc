// ============================================================
//  TOP_levelSOc_simple.v — Minimal SoC for core verification
//  Uses original system_bus + instruction_memory (no caches)
//  Only M-extension changes retained in the core
// ============================================================

// NOTE: This uses the OLD memory interface, not the imem/dmem ports.
// The rv32im_core module currently has imem/dmem bus ports.
// We need a wrapper or the core needs to be adapted.

// For now, this file is a placeholder.
// The quickest path to debug: bypass caches entirely in rv32im_core.
