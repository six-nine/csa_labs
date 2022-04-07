module sword_FSM(input logic clk,
                 input logic reset,
                 input logic[2:0] place,
                 output reg sword);
  reg state;

  assign sword = state;

  always @(posedge clk, posedge reset)
      if (reset)
        state <= 1'b0;
      else
        if (place == 3'b011)
          state <= 1'b1;

endmodule
