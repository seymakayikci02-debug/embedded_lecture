`timescale 1ns/1ps

module top_two_picos_mem_comm(
    input  wire clk,
    input  wire reset
);

  // ============================================================
  // Pico1 signals (kcpsm6)
  // ============================================================
  wire [11:0] p1_address;
  wire [17:0] p1_instruction;
  wire        p1_bram_enable;

  wire [7:0]  p1_port_id;
  wire [7:0]  p1_out_port;
  reg  [7:0]  p1_in_port;
  wire        p1_write_strobe;
  wire        p1_read_strobe;
  wire        p1_k_write_strobe; // not used but must be wired

  // ============================================================
  // Pico2 signals (kcpsm6)
  // ============================================================
  wire [11:0] p2_address;
  wire [17:0] p2_instruction;
  wire        p2_bram_enable;

  wire [7:0]  p2_port_id;
  wire [7:0]  p2_out_port;
  reg  [7:0]  p2_in_port;
  wire        p2_write_strobe;
  wire        p2_read_strobe;
  wire        p2_k_write_strobe;

  // ============================================================
  // KCPSM6 instances
  // ============================================================
  kcpsm6 pico1 (
    .address        (p1_address),
    .instruction    (p1_instruction),
    .clk            (clk),
    .in_port        (p1_in_port),
    .interrupt      (1'b0),
    .reset          (reset),
    .sleep          (1'b0),
    .bram_enable    (p1_bram_enable),
    .k_write_strobe (p1_k_write_strobe),
    .out_port       (p1_out_port),
    .port_id        (p1_port_id),
    .read_strobe    (p1_read_strobe),
    .write_strobe   (p1_write_strobe)
  );

  kcpsm6 pico2 (
    .address        (p2_address),
    .instruction    (p2_instruction),
    .clk            (clk),
    .in_port        (p2_in_port),
    .interrupt      (1'b0),
    .reset          (reset),
    .sleep          (1'b0),
    .bram_enable    (p2_bram_enable),
    .k_write_strobe (p2_k_write_strobe),
    .out_port       (p2_out_port),
    .port_id        (p2_port_id),
    .read_strobe    (p2_read_strobe),
    .write_strobe   (p2_write_strobe)
  );

  // ============================================================
  // Instruction BRAMs (your wrapper style)
  // BRAM1 holds pico1 program, BRAM2 holds pico2 program
  // ============================================================
  BRAM1 inst_mem1 (
    .address     (p1_address),
    .instruction (p1_instruction),
    .clk         (clk),
    .enable      (p1_bram_enable)
  );

  BRAM2 inst_mem2 (
    .address     (p2_address),
    .instruction (p2_instruction),
    .clk         (clk),
    .enable      (p2_bram_enable)
  );

  // ============================================================
  // Data memories (NO block ram primitives)
  // ============================================================
  reg [7:0] mem1 [0:7];
  reg [7:0] mem2 [0:7];
  reg [7:0] mem3 [0:7];

  reg [7:0] mem1_addr;
  reg [7:0] mem2_addr;
  reg [7:0] mem3_addr;

  integer i;

always @(posedge clk) begin
  if (reset) begin
    for (i = 0; i < 8; i = i + 1) begin
      mem1[i] <= 8'h00;
      mem2[i] <= 8'h00;
      mem3[i] <= 8'h00;
    end

    // initial values after reset
    mem1[8'h00] <= 8'd7;    // X
    mem2[8'h00] <= 8'd10;   // K
  end
end

  // ============================================================
  // Comm buffer (1 byte)
  // bit0 FULL, bit1 EMPTY
  // ============================================================
  reg [7:0] comm_data;
  reg       comm_full;
  wire [7:0] comm_stat = {6'b0, (~comm_full), comm_full};

  // ============================================================
  // Pico2 -> Mem3 write latch
  // ============================================================
  reg [7:0] mem3_din_latched;
  reg [7:0] mem3_din_reg;
  // ============================================================
  // Sequential logic
  // ============================================================
  always @(posedge clk) begin
    if (reset) begin
      mem1_addr <= 8'h00;
      mem2_addr <= 8'h00;
      mem3_addr <= 8'h00;

      comm_data <= 8'h00;
      comm_full <= 1'b0;

      mem3_din_latched <= 8'h00;
    end else begin

      // -------- Pico1 writes --------
      if (p1_write_strobe) begin
        case (p1_port_id)
          8'd00: mem1_addr <= p1_out_port;  // MEM1_ADDR
          8'd10: begin                      // COMM_DIN
            if (!comm_full) begin
              comm_data <= p1_out_port;
              comm_full <= 1'b1;
            end
          end
          default: ;
        endcase
      end

      // -------- Pico2 writes --------
           
      if (p2_write_strobe) begin
        case (p2_port_id)
      
          8'd20: mem2_addr <= p2_out_port;   // MEM2_ADDR
      
          8'd30: mem3_addr <= p2_out_port;   // MEM3_ADDR
      
          8'd31: mem3_din_reg <= p2_out_port;
      
          8'd32: begin                       // MEM3_WRITE (DIRECT)
            mem3[mem3_addr] <= mem3_din_reg;
          end
      
          default: ;
        endcase
      end

      // -------- Comm pop on Pico2 read of COMM_DOUT (11) --------
      if (p2_read_strobe && (p2_port_id == 8'h11)) begin
        comm_full <= 1'b0;
      end

    end
  end

  // ============================================================
  // Read muxes (RDPRT)
  // ============================================================
  always @(*) begin
    p1_in_port = 8'h00;
    p2_in_port = 8'h00;

    // Pico1 reads
    case (p1_port_id)
      8'd01: p1_in_port = mem1[mem1_addr]; // MEM1_DOUT
      8'd12: p1_in_port = comm_stat;       // COMM_STAT
      default: p1_in_port = 8'h00;
    endcase

    // Pico2 reads
    case (p2_port_id)
      8'd21: p2_in_port = mem2[mem2_addr]; // MEM2_DOUT
      8'd11: p2_in_port = comm_data;       // COMM_DOUT
      8'd12: p2_in_port = comm_stat;       // COMM_STAT
      default: p2_in_port = 8'h00;
    endcase
  end

endmodule
