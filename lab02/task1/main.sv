`include "./flopr.sv"
`include "./output_decoder.sv"
`include "./state_changer.sv"

module lab2_01(input logic clk,
               input logic reset,
               input logic left, right,
               output logic la, lb, lc, ra, rb, rc);

  logic l0, r0, a0, b0, c0;
  logic l1, r1, a1, b1, c1;
  state_changer next_logic(l0, r0, a0, b0, c0, left, right, l1, r1, a1, b1, c1);
  flopr current_state(clk, reset, {l1, r1, a1, b1, c1}, {l0, r0, a0, b0, c0});
  output_decoder decoder(l0, r0, a0, b0, c0, la, lb, lc, ra, rb, rc);

endmodule
