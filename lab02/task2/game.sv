`include "place_fsm.sv"
`include "sword_fsm.sv"

module game(input logic clk,
            input logic reset,
            input logic[1:0] dir,
            output logic[1:0] result);
  localparam[1:0]
    in_process = 2'b00,
    win = 2'b01,
    die = 2'b10;

  localparam[2:0]
    cave  = 3'b000,
    tunnel = 3'b001,
    river = 3'b010,
    cache = 3'b011,
    dragon = 3'b100,
    victorious_vault = 3'b101,
    cemetery = 3'b110;

  logic sword;
  logic[2:0] place;

  place_FSM pl_fsm(clk, reset, dir, sword, place);
  sword_FSM sw_fsm(clk, reset, place, sword);

  always @(place)
    begin
      case (place)
        victorious_vault: 
          result <= win;
        cemetery:
          result <= die;
        default:
          result <= in_process;
      endcase
    end

endmodule
