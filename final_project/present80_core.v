module present80_core (
    input  wire        clk,
    input  wire        rst,

    input  wire        start,
    output reg         busy,
    output reg         done,

    input  wire [63:0] pt,
    input  wire [79:0] key,
    output reg  [63:0] ct
);

  reg [63:0] state;
  reg [79:0] round_key;
  reg [5:0]  round; // 1..31

  function automatic [3:0] sbox4(input [3:0] x);
    case (x)
      4'h0: sbox4 = 4'hC; 4'h1: sbox4 = 4'h5; 4'h2: sbox4 = 4'h6; 4'h3: sbox4 = 4'hB;
      4'h4: sbox4 = 4'h9; 4'h5: sbox4 = 4'h0; 4'h6: sbox4 = 4'hA; 4'h7: sbox4 = 4'hD;
      4'h8: sbox4 = 4'h3; 4'h9: sbox4 = 4'hE; 4'hA: sbox4 = 4'hF; 4'hB: sbox4 = 4'h8;
      4'hC: sbox4 = 4'h4; 4'hD: sbox4 = 4'h7; 4'hE: sbox4 = 4'h1; 4'hF: sbox4 = 4'h2;
    endcase
  endfunction

  function automatic [63:0] sbox_layer(input [63:0] x);
    integer i;
    reg [63:0] y;
    begin
      y = 64'h0;
      for (i=0; i<16; i=i+1) begin
        y[i*4 +: 4] = sbox4(x[i*4 +: 4]);
      end
      sbox_layer = y;
    end
  endfunction

  function automatic [63:0] player(input [63:0] x);
    integer i;
    reg [63:0] y;
    integer pos;
    begin
      y = 64'h0;
      for (i=0; i<63; i=i+1) begin
        pos = (i*16) % 63;
        y[pos] = x[i];
      end
      y[63] = x[63];
      player = y;
    end
  endfunction

  function automatic [79:0] key_schedule_next(input [79:0] k, input [5:0] r);
    reg [79:0] kk;
    reg [3:0] ms_nib;
    begin
      // rotate left by 61
      kk = {k[18:0], k[79:19]};

      // S-box on MS nibble
      ms_nib = kk[79:76];
      kk[79:76] = sbox4(ms_nib);

      // XOR round counter r into bits [19:15]
      kk[19:15] = kk[19:15] ^ r[4:0];

      key_schedule_next = kk;
    end
  endfunction

  wire [63:0] add_round_key = state ^ round_key[79:16];
  wire [63:0] after_sbox    = sbox_layer(add_round_key);
  wire [63:0] after_player  = player(after_sbox);

  always @(posedge clk) begin
    if (rst) begin
      busy      <= 1'b0;
      done      <= 1'b0;
      ct        <= 64'h0;
      state     <= 64'h0;
      round_key <= 80'h0;
      round     <= 6'd0;
    end else begin
      done <= 1'b0;

      if (start && !busy) begin
        busy      <= 1'b1;
        round     <= 6'd1;
        state     <= pt;
        round_key <= key;
      end else if (busy) begin
        if (round <= 6'd31) begin
          // rounds 1..31
          state     <= after_player;
          round_key <= key_schedule_next(round_key, round);
          round     <= round + 6'd1;
        end else begin
          // final addRoundKey (round 32 key = current round_key[79:16])
          ct   <= state ^ round_key[79:16];
          busy <= 1'b0;
          done <= 1'b1;
          round <= 6'd0;
        end
      end
    end
  end

endmodule
