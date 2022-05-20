// controller.sv
//
// This file is for HMC E85A Lab 5.
// Place controller.tv in same computer directory as this file to test your multicycle controller.
//
// Starter code last updated by Ben Bracker (bbracker@hmc.edu) 1/14/21
// - added opcodetype enum
// - updated testbench and hash generator to accomodate don't cares as expected outputs
// Solution code by ________ (________) ________

typedef enum logic[6:0] {r_type_op=7'b0110011, i_type_alu_op=7'b0010011, lw_op=7'b0000011, sw_op=7'b0100011, beq_op=7'b1100011, jal_op=7'b1101111} opcodetype;

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

module controller(input  logic       clk,
                  input  logic       reset,  
                  input  opcodetype  op,
                  input  logic [2:0] funct3,
                  input  logic       funct7b5,
                  input  logic       Zero,
                  output logic [1:0] ImmSrc,
                  output logic [1:0] ALUSrcA, ALUSrcB,
                  output logic [1:0] ResultSrc, 
                  output logic       AdrSrc,
                  output logic [2:0] ALUControl,
                  output logic       IRWrite, PCWrite, 
                  output logic       RegWrite, MemWrite);


  logic [1:0] ALUOp;
  logic       Branch;

  aludec alud(op[5], funct3, funct7b5, ALUOp, ALUControl);

  parameter S0_FETCH     = 4'b0000;
  parameter S1_DECODE    = 4'b0001;
  parameter S2_MEM_ADR   = 4'b0010;
  parameter S3_MEM_READ  = 4'b0011;
  parameter S4_MEM_WB    = 4'b0100;
  parameter S5_MEM_WRITE = 4'b0101;
  parameter S6_EXEC_R    = 4'b0110;
  parameter S7_ALUWB     = 4'b0111;
  parameter S8_EXEC_L    = 4'b1000;
  parameter S9_JAL       = 4'b1001;
  parameter S10_BEQ      = 4'b1010;

  reg [3:0] state, next_state;

  always_comb begin
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
      S6_EXEC_R: begin
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
      S4_MEM_WB: begin
        next_state = S0_FETCH;
      end
    endcase
  end

  logic PCUpdate;

  assign PCWrite = (Branch & Zero) | PCUpdate;

  always_comb
    case(op)
      7'b0000011: ImmSrc = 2'b00; // lw
      7'b0100011: ImmSrc = 2'b01; // sw
      7'b0110011: ImmSrc = 2'bxx; // R-type 
      7'b1100011: ImmSrc = 2'b10; // beq
      7'b0010011: ImmSrc = 2'b00; // I-type ALU
      7'b1101111: ImmSrc = 2'b11; // jal
      default:    ImmSrc = 2'bxx; // non-implemented instruction
    endcase

  always_comb begin
    IRWrite = 0;
    PCUpdate = 0;
    RegWrite = 0;
    MemWrite = 0;
    AdrSrc = 0;
    Branch = 0;
    if (!reset)
    case (state)
      S0_FETCH: begin
        AdrSrc = 0;
        IRWrite = 1;
        ALUSrcA = 2'b00;
        ALUSrcB = 2'b10;
        ALUOp = 2'b00;
        ResultSrc = 2'b10;
        PCUpdate = 1;
      end
      S1_DECODE: begin
        ALUSrcA = 2'b01;
        ALUSrcB = 2'b01;
        ALUOp = 2'b00;
      end
      S2_MEM_ADR: begin
        ALUSrcA = 2'b10;
        ALUSrcB = 2'b01;
        ALUOp = 2'b00;
      end
      S3_MEM_READ: begin
        ResultSrc = 2'b00;
        AdrSrc = 1;
      end
      S4_MEM_WB: begin
        ResultSrc = 2'b01;
        RegWrite = 1;
      end
      S5_MEM_WRITE: begin
        ResultSrc = 2'b00;
        AdrSrc = 1;
        MemWrite = 1;
      end
      S6_EXEC_R: begin
        ALUSrcA = 2'b10;
        ALUSrcB = 2'b00;
        ALUOp = 2'b10;
      end
      S7_ALUWB: begin
        ResultSrc = 2'b00;
        RegWrite = 1;
      end
      S8_EXEC_L: begin
        ALUSrcA = 2'b10;
        ALUSrcB = 2'b01;
        ALUOp = 2'b10;
      end
      S9_JAL: begin
        ALUSrcA = 2'b01;
        ALUSrcB = 2'b10;
        ALUOp = 2'b00;
        ResultSrc = 2'b00;  
        PCUpdate = 1;
      end
      S10_BEQ: begin
        ALUSrcA = 2'b10;
        ALUSrcB = 2'b00;
        ALUOp = 2'b01;
        ResultSrc = 2'b00;  
        Branch = 1;
      end
    endcase
  end

  always @(posedge clk)
    if (!reset)
      state <= next_state;

endmodule

module testbench();

  logic        clk;
  logic        reset;
  
  opcodetype  op;
  logic [2:0] funct3;
  logic       funct7b5;
  logic       Zero;
  logic [1:0] ImmSrc;
  logic [1:0] ALUSrcA, ALUSrcB;
  logic [1:0] ResultSrc;
  logic       AdrSrc;
  logic [2:0] ALUControl;
  logic       IRWrite, PCWrite;
  logic       RegWrite, MemWrite;
  
  logic [31:0] vectornum, errors;
  logic [39:0] testvectors[10000:0];
  
  logic        new_error;
  logic [15:0] expected;
  logic [6:0]  hash;


  // instantiate device to be tested
  controller dut(clk, reset, op, funct3, funct7b5, Zero,
                 ImmSrc, ALUSrcA, ALUSrcB, ResultSrc, AdrSrc, ALUControl, IRWrite, PCWrite, RegWrite, MemWrite);
  
  // generate clock
  always 
    begin
      clk = 1; #5; clk = 0; #5;
    end

  // at start of test, load vectors and pulse reset
  initial
    begin
      $dumpfile("riscvmulticontroller.vcd");
      $dumpvars(0, testbench);

      $readmemb("controller.tv", testvectors);
      
      vectornum = 0; errors = 0; hash = 0;
      reset = 1; #18; reset = 0;
    end
	 
  // apply test vectors on rising edge of clk
  always @(posedge clk)
    begin
      #1; {op, funct3, funct7b5, Zero, expected} = testvectors[vectornum];
    end

  // check results on falling edge of clk
  always @(negedge clk)
    if (~reset) begin // skip cycles during reset
      new_error=0; 

      if ((ImmSrc!==expected[15:14])&&(expected[15:14]!==2'bxx))  begin
        $display("   ImmSrc = %b      Expected %b", ImmSrc,     expected[15:14]);
        new_error=1;
      end
      if ((ALUSrcA!==expected[13:12])&&(expected[13:12]!==2'bxx)) begin
        $display("   ALUSrcA = %b     Expected %b", ALUSrcA,    expected[13:12]);
        new_error=1;
      end
      if ((ALUSrcB!==expected[11:10])&&(expected[11:10]!==2'bxx)) begin
        $display("   ALUSrcB = %b     Expected %b", ALUSrcB,    expected[11:10]);
        new_error=1;
      end
      if ((ResultSrc!==expected[9:8])&&(expected[9:8]!==2'bxx))   begin
        $display("   ResultSrc = %b   Expected %b", ResultSrc,  expected[9:8]);
        new_error=1;
      end
      if ((AdrSrc!==expected[7])&&(expected[7]!==1'bx))           begin
        $display("   AdrSrc = %b       Expected %b", AdrSrc,     expected[7]);
        new_error=1;
      end
      if ((ALUControl!==expected[6:4])&&(expected[6:4]!==3'bxxx)) begin
        $display("   ALUControl = %b Expected %b", ALUControl, expected[6:4]);
        new_error=1;
      end
      if ((IRWrite!==expected[3])&&(expected[3]!==1'bx))          begin
        $display("   IRWrite = %b      Expected %b", IRWrite,    expected[3]);
        new_error=1;
      end
      if ((PCWrite!==expected[2])&&(expected[2]!==1'bx))          begin
        $display("   PCWrite = %b      Expected %b", PCWrite,    expected[2]);
        new_error=1;
      end
      if ((RegWrite!==expected[1])&&(expected[1]!==1'bx))         begin
        $display("   RegWrite = %b     Expected %b", RegWrite,   expected[1]);
        new_error=1;
      end
      if ((MemWrite!==expected[0])&&(expected[0]!==1'bx))         begin
        $display("   MemWrite = %b     Expected %b", MemWrite,   expected[0]);
        new_error=1;
      end

      if (new_error) begin
        $display("Error on vector %d: inputs: op = %h funct3 = %h funct7b5 = %h", vectornum, op, funct3, funct7b5);
        errors = errors + 1;
      end
      vectornum = vectornum + 1;
      hash = hash ^ {ImmSrc&{2{expected[15:14]!==2'bxx}}, ALUSrcA&{2{expected[13:12]!==2'bxx}}} ^ {ALUSrcB&{2{expected[11:10]!==2'bxx}}, ResultSrc&{2{expected[9:8]!==2'bxx}}} ^ {AdrSrc&{expected[7]!==1'bx}, ALUControl&{3{expected[6:4]!==3'bxxx}}} ^ {IRWrite&{expected[3]!==1'bx}, PCWrite&{expected[2]!==1'bx}, RegWrite&{expected[1]!==1'bx}, MemWrite&{expected[0]!==1'bx}};
      hash = {hash[5:0], hash[6] ^ hash[5]};
      if (testvectors[vectornum] === 40'bx) begin 
        $display("%d tests completed with %d errors", vectornum, errors);
	      $display("hash = %h", hash);
        $finish;
      end
    end
endmodule

