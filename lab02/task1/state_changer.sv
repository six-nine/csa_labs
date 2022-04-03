module state_changer(input  logic l0, r0, a0, b0, c0,
                     input  logic in_l, in_r,
                     output logic l1, r1, a1, b1, c1);


  assign l1 = (l0 & ~c0) | (~l0 & ~r0 & in_l);
  assign r1 = (r0 & ~c0) | (~l0 & ~r0 & in_r);

  assign a1 = ~l0 & ~r0 & (in_l | in_r);
  assign b1 = (l0 | r0) & a0;
  assign c1 = (l0 | r0) & b0;

endmodule
