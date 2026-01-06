`timescale 1ns / 1ps

module sim_tb();

    reg clk;
    reg rst;

    top_two_picos_mem_comm Pico (
        .clk   (clk),
        .reset (rst)
    );

    always #5 clk = ~clk;   // <-- clock slowed down

    initial begin
        clk = 0;
        rst = 0;
        #20;
        rst = 1;
        #20;
        rst = 0;
        #200000;            // more simulation time
        $stop;
    end

endmodule
