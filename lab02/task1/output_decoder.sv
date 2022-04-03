module output_decoder(input logic l, r, a, b, c,
                      output logic la, lb, lc, ra, rb, rc);

  assign la = l & (a | b | c);
  assign lb = l & (b | c);
  assign lc = l & c;

  assign ra = r & (a | b | c);
  assign rb = r & (b | c);
  assign rc = r & c;

endmodule
