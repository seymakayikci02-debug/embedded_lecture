`timescale 1ns/1ps

module top_two_picos_mem_comm(
    input  wire clk,
    input  wire reset
);

  // ---------------- Pico1 ----------------
  wire [11:0] p1_address;
  wire [17:0] p1_instruction;
  wire        p1_bram_enable;
  wire [7:0]  p1_port_id;
  wire [7:0]  p1_out_port;
  reg  [7:0]  p1_in_port;
  wire        p1_write_strobe;
  wire        p1_read_strobe;
  wire        p1_k_write_strobe;

  // ---------------- Pico2 ----------------
  wire [11:0] p2_address;
  wire [17:0] p2_instruction;
  wire        p2_bram_enable;
  wire [7:0]  p2_port_id;
  wire [7:0]  p2_out_port;
  reg  [7:0]  p2_in_port;
  wire        p2_write_strobe;
  wire        p2_read_strobe;
  wire        p2_k_write_strobe;

  kcpsm6 pico1 (
    .address(p1_address), .instruction(p1_instruction), .clk(clk),
    .in_port(p1_in_port), .interrupt(1'b0), .reset(reset), .sleep(1'b0),
    .bram_enable(p1_bram_enable), .k_write_strobe(p1_k_write_strobe),
    .out_port(p1_out_port), .port_id(p1_port_id),
    .read_strobe(p1_read_strobe), .write_strobe(p1_write_strobe)
  );

  kcpsm6 pico2 (
    .address(p2_address), .instruction(p2_instruction), .clk(clk),
    .in_port(p2_in_port), .interrupt(1'b0), .reset(reset), .sleep(1'b0),
    .bram_enable(p2_bram_enable), .k_write_strobe(p2_k_write_strobe),
    .out_port(p2_out_port), .port_id(p2_port_id),
    .read_strobe(p2_read_strobe), .write_strobe(p2_write_strobe)
  );

  BRAM1 inst_mem1 (.address(p1_address), .instruction(p1_instruction), .clk(clk), .enable(p1_bram_enable));
  BRAM2 inst_mem2 (.address(p2_address), .instruction(p2_instruction), .clk(clk), .enable(p2_bram_enable));

  // ---------------- Data memories (demo arrays) ----------------
  reg [7:0] mem1 [0:255];
  reg [7:0] mem2 [0:255];
  reg [7:0] mem3 [0:255];

  reg [7:0] mem1_addr;
  reg [7:0] mem2_addr;
  reg [7:0] mem3_addr;

  integer i;

  // demo init
  always @(posedge clk) begin
    if (reset) begin
      for (i=0; i<256; i=i+1) begin
        mem1[i] <= 8'h00;
        mem2[i] <= 8'h00;
        mem3[i] <= 8'h00;
      end

      // example plaintext block at mem1[0..7]
      mem1[0] <= 8'h00; mem1[1] <= 8'h11; mem1[2] <= 8'h22; mem1[3] <= 8'h33;
      mem1[4] <= 8'h44; mem1[5] <= 8'h55; mem1[6] <= 8'h66; mem1[7] <= 8'h77;

      // example 80-bit key at mem2[0..9]
      mem2[0] <= 8'h00; mem2[1] <= 8'h01; mem2[2] <= 8'h02; mem2[3] <= 8'h03; mem2[4] <= 8'h04;
      mem2[5] <= 8'h05; mem2[6] <= 8'h06; mem2[7] <= 8'h07; mem2[8] <= 8'h08; mem2[9] <= 8'h09;
    end
  end

  // ---------------- Comm buffer 1 byte ----------------
  reg [7:0] comm_data;
  reg       comm_full;
  wire      comm_empty = ~comm_full;
  wire [7:0] comm_stat = {6'b0, comm_empty, comm_full}; // bit0 full, bit1 empty

  // write by Pico1 port 10
  wire p1_comm_wr = p1_write_strobe && (p1_port_id == 8'd10);

  // pop by Pico2 READ of port 11
  wire p2_comm_rd = p2_read_strobe && (p2_port_id == 8'd11);

  always @(posedge clk) begin
    if (reset) begin
      comm_data <= 8'h00;
      comm_full <= 1'b0;
    end else begin
      if (p1_comm_wr && !comm_full) begin
        comm_data <= p1_out_port;
        comm_full <= 1'b1;
      end
      if (p2_comm_rd && comm_full) begin
        comm_full <= 1'b0;
      end
    end
  end

  // ---------------- PRESENT core IO regs ----------------
  reg [7:0] crypto_idx;
  reg [79:0] crypto_key;
  reg [63:0] crypto_pt;
  wire [63:0] crypto_ct;
  reg  crypto_start;

  wire crypto_busy;
  wire crypto_done;

  // pack/unpack bytes (idx 0 = state[63:56] for convenience)
  function automatic [63:0] put_pt_byte(input [63:0] x, input [2:0] idx, input [7:0] b);
    reg [63:0] y;
    begin
      y = x;
      y[63-idx*8 -: 8] = b;
      put_pt_byte = y;
    end
  endfunction

  function automatic [79:0] put_key_byte(input [79:0] x, input [3:0] idx, input [7:0] b);
    reg [79:0] y;
    begin
      y = x;
      y[79-idx*8 -: 8] = b;
      put_key_byte = y;
    end
  endfunction

  function automatic [7:0] get_ct_byte(input [63:0] x, input [2:0] idx);
    begin
      get_ct_byte = x[63-idx*8 -: 8];
    end
  endfunction

  present80_core u_present (
    .clk(clk), .rst(reset),
    .start(crypto_start),
    .busy(crypto_busy),
    .done(crypto_done),
    .pt(crypto_pt),
    .key(crypto_key),
    .ct(crypto_ct)
  );

  // crypto status read
  wire [7:0] crypto_stat = {6'b0, crypto_done, crypto_busy};
  
    reg mem3_we;
    reg mem3_din_reg;
  // ---------------- Sequential decode ----------------
  always @(posedge clk) begin
    if (reset) begin
      mem1_addr <= 8'h00;
      mem2_addr <= 8'h00;
      mem3_addr <= 8'h00;

      crypto_idx   <= 8'h00;
      crypto_key   <= 80'h0;
      crypto_pt    <= 64'h0;
      crypto_start <= 1'b0;
    end else begin
      crypto_start <= 1'b0;

      // Pico1 writes
      if (p1_write_strobe) begin
        case (p1_port_id)
          8'd0: mem1_addr <= p1_out_port;   // MEM1_ADDR
          default: ;
        endcase
      end
   
      // Pico2 writes
      if (p2_write_strobe) begin
        case (p2_port_id)
          8'd20: mem2_addr <= p2_out_port;            // MEM2_ADDR

          8'd30: mem3_addr <= p2_out_port;            // MEM3_ADDR
          8'd31: begin  mem3_din_reg <= p2_out_port; end
          8'd32: begin                                // MEM3_WE (commit)
           
              if (p2_out_port != 8'h00) mem3_we <= 1'b1;       // not used by PRESENT flow
          end

          // crypto ports
          8'd41: crypto_idx <= p2_out_port;           // CRYPTO_IDX
          8'd42: crypto_key <= put_key_byte(crypto_key, crypto_idx[3:0], p2_out_port); // CRYPTO_KEY
          8'd43: crypto_pt  <= put_pt_byte(crypto_pt,  crypto_idx[2:0], p2_out_port);  // CRYPTO_PT
          8'd40: begin                                 // CRYPTO_CTRL
            if (p2_out_port[0]) crypto_start <= 1'b1;  // start pulse
          end

          default: ;
        endcase
      end
      if (mem3_we) begin
      mem3[mem3_addr] <= mem3_din_reg;
      end
    end
  end


     // ---------------- Read muxes ----------------
  always @(*) begin
    p1_in_port = 8'h00;
    case (p1_port_id)
      8'd1:  p1_in_port = mem1[mem1_addr];
      8'd12: p1_in_port = comm_stat;
      default: p1_in_port = 8'h00;
    endcase
  end

  always @(*) begin
    p2_in_port = 8'h00;
    case (p2_port_id)
      8'd21: p2_in_port = mem2[mem2_addr];
      8'd11: p2_in_port = comm_data;
      8'd12: p2_in_port = comm_stat;

      8'd45: p2_in_port = crypto_stat;
      8'd44: p2_in_port = get_ct_byte(crypto_ct, crypto_idx[2:0]);

      default: p2_in_port = 8'h00;
    endcase
  end

endmodule

