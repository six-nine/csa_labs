module flopr(input  logic       clk,
             input  logic       reset,
             input  logic [4:0] d,
             output logic [4:0] q);

  always @(posedge clk)
    if (reset) 
      q <= 0;
    else
      q <= d;

endmodule
