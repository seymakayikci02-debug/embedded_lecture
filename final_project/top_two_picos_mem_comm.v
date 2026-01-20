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

  // ---------------- MEM Size ----------------
  parameter MEM1_DEPTH = 8;   // 8 byte (64-bit)
  parameter MEM2_DEPTH = 10;  // 10 byte (80-bit)
  parameter MEM3_DEPTH = 8;   // 8 byte (64-bit)

  // PicoBlaze cores
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

  // Instruction memories 
  BRAM1 inst_mem1 (.address(p1_address), .instruction(p1_instruction), .clk(clk), .enable(p1_bram_enable));
  BRAM2 inst_mem2 (.address(p2_address), .instruction(p2_instruction), .clk(clk), .enable(p2_bram_enable));

  // ---------------- Data memories  ----------------
  reg [7:0] mem1 [0:MEM1_DEPTH-1];
  reg [7:0] mem2 [0:MEM2_DEPTH-1];
  reg [7:0] mem3 [0:MEM3_DEPTH-1];

  reg [7:0] mem1_addr;
  reg [7:0] mem2_addr;
  reg [7:0] mem3_addr;

  integer i;
  
  // ---------------- Comm buffer 1 byte ----------------
  reg [7:0] comm_data;
  reg       comm_full;
  wire      comm_empty = ~comm_full;
  wire [7:0] comm_stat = {6'b0, comm_empty, comm_full}; // bit0 full, bit1 empty,6 zeros concatenated

  wire p1_comm_wr = p1_write_strobe && (p1_port_id == 8'd10);  //instead of writing long condition it is combined in p1_comm_wr
  wire p2_comm_rd = p2_read_strobe  && (p2_port_id == 8'd11);  //instead of writing long condition it is combined in p2_comm_rd

  always @(posedge clk) begin
    if (reset) begin
      comm_data <= 8'h00;
      comm_full <= 1'b0;
    end else begin
      // push to communication buffer
      if (p1_comm_wr && !comm_full) begin
        comm_data <= p1_out_port;
        comm_full <= 1'b1;
      end
      // pop from communication buffer to Pico2
      if (p2_comm_rd && comm_full) begin
        comm_full <= 1'b0;      
      end
    end
  end

  // ---------------- PRESENT core IO regs ----------------
  reg [7:0]  crypto_idx;    //tracks the current byte
  reg [79:0] crypto_key;    //holds the 80 bit key data comming from memory2 
  reg [63:0] crypto_pt;     //holds the 64 bit plain text data comming from memory1 
  wire [63:0] crypto_ct;    //holds the 64 bit crypted data comming from algorithm 

//start signal for the Present-80 algorithm module
  reg  crypto_start;  
// Status signals coming from the crypto core     
  wire crypto_busy;         
  wire crypto_done;

  reg  crypto_done_sticky;

 // pack/unpack bytes: idx=0 -> MSB byte
  
 //put_pt_byte:  Takes a 64-bit plaintext word and replaces the byte at position idx with the new byte b
  function automatic [63:0] put_pt_byte(input [63:0] x, input [2:0] idx, input [7:0] b);
    reg [63:0] y;
    begin
      y = x;
      y[63-idx*8 -: 8] = b;
      put_pt_byte = y;
    end
  endfunction

//put_key_byte:  Takes an 80-bit key word and replaces the byte at position idx with the new key byte b
  function automatic [79:0] put_key_byte(input [79:0] x, input [3:0] idx, input [7:0] b);
    reg [79:0] y;
    begin
      y = x;
      y[79-idx*8 -: 8] = b;
      put_key_byte = y;
    end
  endfunction

//get_ct_byte:  Extracts and returns the byte at position idx from a 64-bit ciphertext word
  function automatic [7:0] get_ct_byte(input [63:0] x, input [2:0] idx);
    begin
      get_ct_byte = x[63-idx*8 -: 8];
    end
  endfunction

// PRESENT-80 crypto core instance

  present80_core u_present (
    .clk(clk), .rst(reset),
    .start(crypto_start),
    .busy(crypto_busy),
    .done(crypto_done),
    .pt(crypto_pt),
    .key(crypto_key),
    .ct(crypto_ct)
  );

//This block was added later to solve the problem about  Memory 3 write operation
  always @(posedge clk) begin
    if (reset) begin
      crypto_done_sticky <= 1'b0;
    end else begin
      if (crypto_start)
        crypto_done_sticky <= 1'b0;
      else if (crypto_done)
        crypto_done_sticky <= 1'b1;
    end
  end

// Crypto status register which is READ by PicoBlaze
// bit0 = busy, bit1 = done, upper bits unused

  wire [7:0] crypto_stat = {6'b0, crypto_done_sticky, crypto_busy}; // bit0=busy, bit1=done, 6 zeros is concatenated

  // ---------------- Mem3 Write  ----------------
  reg [7:0] mem3_din_reg; // temporary register to hold data for one clock cycle
  reg       mem3_we;      // write enable signal for third memory 

  always @(posedge clk) begin
    if (reset) begin
      // init memories
      for (i=0; i<MEM1_DEPTH; i=i+1) mem1[i] <= 8'h00;
      for (i=0; i<MEM2_DEPTH; i=i+1) mem2[i] <= 8'h00;
      for (i=0; i<MEM3_DEPTH; i=i+1) mem3[i] <= 8'h00;

      // Test memory initialization   
      
      //Mem1 isused for 64-bit plaintext input  
      mem1[0] <= 8'hD3;     mem1[1] <= 8'hAB;      mem1[2] <= 8'h4A;    mem1[3] <= 8'h74;
      mem1[4] <= 8'h56;     mem1[5] <= 8'hC0;      mem1[6] <= 8'hAC;    mem1[7] <= 8'hA6;
      
      // Mem2 used for 80-bit key input 
      mem2[0] <= 8'hFE;     mem2[1] <= 8'hDF;      mem2[2] <= 8'h1F;    mem2[3] <= 8'h72;       mem2[4] <= 8'h72;
      mem2[5] <= 8'hFD;     mem2[6] <= 8'h23;      mem2[7] <= 8'hC6;    mem2[8] <= 8'h67;       mem2[9] <= 8'h85; 

      mem1_addr    <= 8'h00;
      mem2_addr    <= 8'h00;
      mem3_addr    <= 8'h00;
      mem3_din_reg <= 8'h00;
      mem3_we      <= 1'b0;

      crypto_idx   <= 8'h00;
      crypto_key   <= 80'h0;
      crypto_pt    <= 64'h0;
      crypto_start <= 1'b0;

    end else begin
      
      crypto_start <= 1'b0;
      mem3_we      <= 1'b0;     //other than  (p2_write_strobe==1 & p2_out_port != 8'h00) condition write enable signal goes to LOW.

      // Pico1 writes
      // mem1_addr used to select which location of MEM1 will be read
      if (p1_write_strobe) begin
        if (p1_port_id == 8'd0) begin
          mem1_addr <= p1_out_port;   
        end
      end

      // Pico2 writes
      if (p2_write_strobe) begin
        case (p2_port_id)                           //It is shaped with the Picoblaze address assignment scheme
          8'd20: mem2_addr <= p2_out_port;          // MEM2_ADDR
          8'd30: mem3_addr <= p2_out_port;          // MEM3_ADDR
          8'd31: mem3_din_reg <= p2_out_port;       // MEM3_DIN 

          // Write enable signal goes HIGH at 32. address. 
          8'd32: begin
            if (p2_out_port != 8'h00) begin
              mem3_we <= 1'b1;
            end
          end
          

          // Crypto Algorithm signals and registers are assigned acorrding to port 2 addresses
          8'd41: crypto_idx <= p2_out_port; 
          8'd42: crypto_key <= put_key_byte(crypto_key, crypto_idx[3:0], p2_out_port);
          8'd43: crypto_pt  <= put_pt_byte (crypto_pt,  crypto_idx[2:0], p2_out_port); 
          8'd40: begin
            if (p2_out_port[0]) crypto_start <= 1'b1; // start input signal is given by p2_out_port[0]
          end

          default: ;
        endcase
      end
      
      //Memory 3 is filling only when write enable signal is HIGH. It is outside the case statemnet.
      //This if statement does not depend on p2_write_strobe. It belongs to the memory buffer.
      if (mem3_we) begin
      mem3[mem3_addr] <= mem3_din_reg;
      end
    end
  end

  // ---------------- Read muxes ----------------
  always @(*) begin
    p1_in_port = 8'h00;
    case (p1_port_id)
      8'd1:  p1_in_port = mem1[mem1_addr]; // MEM1_DOUT
      8'd12: p1_in_port = comm_stat;      // COMM_STAT
      default: p1_in_port = 8'h00;
    endcase
  end

  always @(*) begin
    p2_in_port = 8'h00;
    case (p2_port_id)
      8'd21: p2_in_port = mem2[mem2_addr]; // MEM2_DOUT (addr 0..9 bekleniyor)
      8'd11: p2_in_port = comm_data;       // COMM_DOUT
      8'd12: p2_in_port = comm_stat;       // COMM_STAT
      8'd45: p2_in_port = crypto_stat;     // CRYPTO_STAT
      8'd44: p2_in_port = get_ct_byte(crypto_ct, crypto_idx[2:0]); // CRYPTO_CT byte
      default: p2_in_port = 8'h00;
    endcase
  end

endmodule
