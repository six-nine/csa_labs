module place_FSM(input logic clk,
                 input logic reset,
                 input logic[1:0] dir,
                 input logic sword,
                 output reg[2:0] place);

  reg[2:0] state;

  localparam[2:0]
    cave  = 3'b000,
    tunnel = 3'b001,
    river = 3'b010,
    cache = 3'b011,
    win = 3'b101,
    die = 3'b110;

  localparam[1:0]
    l = 2'b00,
    r = 2'b01,
    u = 2'b10,
    d = 2'b11;

  assign place = state;

  always @(posedge clk, posedge reset)
    begin
      if (reset)
        state <= cave;
      else
        begin
          case (state)
            cave:
              state <= tunnel;
            tunnel:
              if (dir == l)
                state <= cave;
              else
                state <= river;
            river:
              case (dir)
                l:
                  state <= cache;
                u:
                  state <= tunnel;
                r:
                  if (sword)
                    state <= win;
                  else 
                    state <= die;
              endcase
            cache:
              state <= river;
          endcase
        end
    end

endmodule
