// ============================================================================
// Inst_memory.v — 2-cycle instruction memory with pending (ORIGINAL DESIGN)
// Restored: pending mechanism correctly handles 2-cycle BRAM read latency
// ============================================================================

module instruction_memory #(
    parameter MEM_DEPTH = 256,
    parameter ADDR_BITS = 8
) (
    input  wire        clk,
    input  wire        reset,
    input  wire [31:0] addr,
    output reg  [31:0] instr,
    output reg         imem_ready
);

    reg [31:0] mem [0:MEM_DEPTH-1];
    reg [ADDR_BITS-1:0] addr_reg;
    reg                 pending;
    reg [ADDR_BITS-1:0] pending_addr;

    wire [ADDR_BITS-1:0] word_addr = addr[ADDR_BITS+1:2];

    initial begin
        integer i;
        for (i = 0; i < MEM_DEPTH; i = i + 1)
            mem[i] = 32'h00000013;
        $readmemh("firmware.hex", mem);
        imem_ready = 1'b1;
        pending    = 1'b0;
    end

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            imem_ready   <= 1'b1;
            pending      <= 1'b0;
            addr_reg     <= 0;
            pending_addr <= 0;
            instr        <= 32'b0;
        end
        else if (imem_ready) begin
            addr_reg   <= word_addr;
            imem_ready <= 1'b0;
        end
        else begin
            instr      <= mem[addr_reg];
            imem_ready <= 1'b1;
            if (pending) begin
                addr_reg   <= pending_addr;
                imem_ready <= 1'b0;
                pending    <= 1'b0;
            end
        end
        if (!imem_ready && (word_addr != addr_reg)) begin
            pending      <= 1'b1;
            pending_addr <= word_addr;
        end
    end

endmodule
