module aludec(input  logic       opb5,
              input  logic [2:0] funct3,
              input  logic       funct7b5, 
              input  logic [1:0] ALUOp,
              output logic [2:0] ALUControl);

  logic  RtypeSub;
  assign RtypeSub = funct7b5 & opb5;  // TRUE for R-type subtract instruction

  always_comb
    case(ALUOp)
      2'b00:                ALUControl = 3'b000; // addition
      2'b01:                ALUControl = 3'b001; // subtraction
      default: case(funct3) // R-type or I-type ALU
                 3'b000:  if (RtypeSub) 
                            ALUControl = 3'b001; // sub
                          else          
                            ALUControl = 3'b000; // add, addi
                 3'b010:    ALUControl = 3'b101; // slt, slti
                 3'b110:    ALUControl = 3'b011; // or, ori
                 3'b111:    ALUControl = 3'b010; // and, andi
                 3'b100:    ALUControl = 3'b100; // xor, xori
                 default:   ALUControl = 3'bxxx; // ???
               endcase
    endcase
endmodule

module controller(input   logic       clk,
                  input   logic       reset,  
                  input   logic [6:0] op,
                  input   logic [2:0] funct3,
                  input   logic       funct7b5,
                  input   logic       zero,
                  output  logic [1:0] immsrc,
                  output  logic [1:0] alusrca, alusrcb,
                  output  logic [1:0] resultsrc, 
                  output  logic       adrsrc,
                  output  logic [2:0] alucontrol,
                  output  logic       irwrite, pcwrite, 
                  output  logic       regwrite, memwrite);

  parameter S0_FETCH     = 0;
  parameter S1_DECODE    = 1;
  parameter S2_MEM_ADR   = 2;
  parameter S3_MEM_READ  = 3;
  parameter S4_MEM_WB    = 4;
  parameter S5_MEM_WRITE = 5;
  parameter S6_EXEC_R    = 6;
  parameter S7_ALUWB     = 7;
  parameter S8_EXEC_L    = 8;
  parameter S9_JAL       = 9;
  parameter S10_BEQ      = 10;

  reg [3:0] state, next_state;

  always @(*) begin
    if (reset) next_state = S0_FETCH;
    else case (state)
      S0_FETCH: begin
        next_state = S1_DECODE;
      end
      S1_DECODE: begin
        case (op)
          7'b0000011: next_state = S2_MEM_ADR;
          7'b0100011: next_state = S2_MEM_ADR;
          7'b0110011: next_state = S6_EXEC_R;
          7'b0010011: next_state = S8_EXEC_L;
          7'b1101111: next_state = S9_JAL;
          7'b1100011: next_state = S10_BEQ;
        endcase
      end
      S2_MEM_ADR: begin
        case (op)
          7'b0000011: next_state = S3_MEM_READ;
          7'b0100011: next_state = S5_MEM_WRITE;
        endcase
      end
      S6_FETCH: begin
        next_state = S7_ALUWB;
      end
      S8_EXEC_L: begin
        next_state = S7_ALUWB;
      end
      S9_JAL: begin
        next_state = S7_ALUWB;
      end
      S10_BEQ: begin
        next_state = S0_FETCH;
      end
      S3_MEM_READ: begin
        next_state = S4_MEM_WB;
      end
      S5_MEM_WRITE: begin
        next_state = S0_FETCH;
      end
      S7_ALUWB: begin
        next_state = S0_FETCH;
      end
    endcase
  end

  always @(*) begin
    case (state)
      S0_FETCH: begin
        adrsrc = 0;
        irwrite = 1;
        alusrca = 2'b00;
        alusrcb = 2'b10;
        aluo = 2'b00;
      end
    endcase
  end

  always @(posedge clk)
    state <= next_state;

/*
*
  *
  *
                  output  logic [1:0] immsrc,
                  output  logic [1:0] alusrca, alusrcb,
                  output  logic [1:0] resultsrc, 
                  output  logic       adrsrc,
                  output  logic [2:0] alucontrol,
                  output  logic       irwrite, pcwrite, 
                  output  logic       regwrite, memwrite);
  *
  * */

endmodule
