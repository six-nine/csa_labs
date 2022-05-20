///////////////////////////////////////////////////////////////
// top
//
// Instantiates multicycle RISC-V processor and memory
///////////////////////////////////////////////////////////////

typedef enum logic[6:0] {r_type_op=7'b0110011, i_type_alu_op=7'b0010011, lw_op=7'b0000011, sw_op=7'b0100011, beq_op=7'b1100011, jal_op=7'b1101111} opcodetype;

module extend(input  logic [31:7] instr,
              input  logic [1:0]  immsrc,
              output logic [31:0] immext);
 
  logic instr31, instr7, instr20;
  logic [31:20] instr31_20;
  logic [31:25] instr31_25;
  logic [11:7] instr11_7;
  logic [30:25] instr30_25;
  logic [11:8] instr11_8;
  logic [19:12] instr19_12;
  logic [30:21] instr30_21;

  assign instr31 = instr[31];
  assign instr7 = instr[7];
  assign instr20 = instr[20];

  assign instr31_20 = instr[31:20];
  assign instr31_25 = instr[31:25];
  assign instr11_7  = instr[11:7];
  assign instr30_25 = instr[30:25];
  assign instr11_8  = instr[11:8];
  assign instr19_12 = instr[19:12];
  assign instr30_21 = instr[30:21];

  always_comb
    case(immsrc) 
               // I-type 
      2'b00:   immext = {{20{instr31}}, instr31_20};  
               // S-type (stores)
      2'b01:   immext = {{20{instr31}}, instr31_25, instr11_7}; 
               // B-type (branches)
      2'b10:   immext = {{20{instr31}}, instr7, instr30_25, instr11_8, 1'b0}; 
               // J-type (jal)
      2'b11:   immext = {{12{instr31}}, instr19_12, instr20, instr30_21, 1'b0}; 
      default: immext = 32'bx; // undefined
    endcase             

endmodule


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

module alu(input  logic [31:0] a, b,
           input  logic [2:0]  alucontrol,
           output logic [31:0] result,
           output logic        zero);

  logic [31:0] condinvb, sum;
  logic        v;              // overflow
  logic        isAddSub;       // true when is add or subtract operation

  assign condinvb = alucontrol[0] ? ~b : b;
  assign sum = a + condinvb + alucontrol[0];
  assign sum31 = sum[31];
  assign b40 = b[4:0];
  assign isAddSub = ~alucontrol[2] & ~alucontrol[1] |
                    ~alucontrol[1] & alucontrol[0];

  always_comb
    case (alucontrol)
      3'b000:  result = sum;         // add
      3'b001:  result = sum;         // subtract
      3'b010:  result = a & b;       // and
      3'b011:  result = a | b;       // or
      3'b100:  result = a ^ b;       // xor
      3'b101:  result = sum31 ^ v; // slt
      3'b110:  result = a << b40; // sll
      3'b111:  result = a >> b40; // srl
     default: result = 32'bx;
    endcase 

  assign zero = (result == 32'b0);
  assign v = ~(alucontrol[0] ^ a[31] ^ b[31]) & (a[31] ^ sum[31]) & isAddSub;
  
endmodule

module mux2 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, 
              input  logic             s, 
              output logic [WIDTH-1:0] y);

  assign y = s ? d1 : d0; 
endmodule

module mux3 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, d2,
              input  logic [1:0]       s, 
              output logic [WIDTH-1:0] y);

  assign y = s[1] ? d2 : (s[0] ? d1 : d0); 
endmodule

module regfile(input  logic        clk, 
               input  logic        we3, 
               input  logic [ 4:0] a1, a2, a3, 
               input  logic [31:0] wd3, 
               output logic [31:0] rd1, rd2);

  logic [31:0] rf[31:0];

  // three ported register file
  // read two ports combinationally (A1/RD1, A2/RD2)
  // write third port on rising edge of clock (A3/WD3/WE3)
  // register 0 hardwired to 0

  always_ff @(posedge clk)
    if (we3) rf[a3] <= wd3;	

  assign rd1 = (a1 != 0) ? rf[a1] : 0;
  assign rd2 = (a2 != 0) ? rf[a2] : 0;
endmodule

module flopr_en #(parameter WIDTH = 8)
              (input  logic             clk, reset, enabled,
               input  logic [WIDTH-1:0] d,
               output reg [WIDTH-1:0] q);

  always_ff @(posedge clk, posedge reset)
    if (reset)        q <= 0;
    else if (enabled) q <= d;
endmodule

module flopr #(parameter WIDTH = 8)
              (input  logic             clk, reset,
               input  logic [WIDTH-1:0] d,
               output logic [WIDTH-1:0] q);

  always_ff @(posedge clk, posedge reset)
    if (reset) q <= 0;
    else       q <= d;
endmodule

module datapath(input  logic        clk, reset,
                input  logic [1:0]  ResultSrc, 
                input  logic        PCWrite, AdrSrc,
                input  logic        MemWrite, RegWrite, IRWrite,
                input  logic [1:0]  ImmSrc,
                input  logic [1:0]  ALUSrcA, ALUSrcB,
                input  logic [2:0]  ALUControl,
                input  logic [31:0] ReadData,
                output logic        Zero,
                output logic [31:0] Adr,
                output logic [31:0] Instr,
                output logic [31:0] WriteData);

  logic [31:0] PC;

  logic [31:0] SrcA, SrcB;
  logic [31:0] Result;
  logic [31:0] ALUResult, ALUOut;

  logic [31:0] ImmExt;

  logic [31:0] OldPC, A;

  logic [31:0] rd1, rd2;

  logic [31:0] Data;

  flopr #(32) rd1reg(clk, reset, rd1, A);
  flopr #(32) rd2reg(clk, reset, rd2, WriteData);

  flopr_en #(32) oldpcreg(clk, reset, IRWrite, PC, OldPC);
  flopr_en #(32) instrreg(clk, reset, IRWrite, ReadData, Instr);

  flopr #(32) readdatareg(clk, reset, ReadData, Data);

  flopr #(32) alureg(clk, reset, ALUResult, ALUOut);

  flopr_en #(32) pcreg(clk, reset, PCWrite, Result, PC);

  mux2 #(32) adrmux(PC, Result, AdrSrc, Adr);

  mux3 #(32) resultmux(ALUOut, Data, ALUResult, ResultSrc, Result);

  // register file logic
  regfile     rf(clk, RegWrite, Instr[19:15], Instr[24:20], 
                 Instr[11:7], Result, rd1, rd2);
  extend      ext(Instr[31:7], ImmSrc, ImmExt);

  // ALU logic
  mux3 #(32)  srcamux(PC, OldPC, A, ALUSrcA, SrcA);
  mux3 #(32)  srcbmux(WriteData, ImmExt, 4, ALUSrcB, SrcB);
  alu         alu(SrcA, SrcB, ALUControl, ALUResult, Zero);
endmodule

module top(input  logic        clk, reset, 
           output logic [31:0] WriteData, DataAdr, 
           output logic        MemWrite);

  logic [31:0] ReadData;
  
  // instantiate processor and memories
  riscvmulti rvmulti(clk, reset, MemWrite, DataAdr, 
                     WriteData, ReadData);
  mem mem(clk, MemWrite, DataAdr, WriteData, ReadData);
endmodule

///////////////////////////////////////////////////////////////
// mem
//
// Single-ported RAM with read and write ports
// Initialized with machine language program
///////////////////////////////////////////////////////////////

module mem(input  logic        clk, we,
           input  logic [31:0] a, wd,
           output logic [31:0] rd);

  logic [31:0] RAM[63:0];
  
  initial
      $readmemh("riscvtest.txt",RAM);

  assign rd = RAM[a[31:2]]; // выравнивание на слово

  always_ff @(posedge clk)
    if (we) RAM[a[31:2]] <= wd;
endmodule

///////////////////////////////////////////////////////////////
// riscvmulti
//
// Multicycle RISC-V microprocessor
///////////////////////////////////////////////////////////////

module riscvmulti(input  logic        clk, reset,
                  output logic        MemWrite,
                  output logic [31:0] Adr, WriteData,
                  input  logic [31:0] ReadData);


  logic [31:0] Instr;
  logic [1:0]  ImmSrc, ALUSrcA, ALUSrcB, ResultSrc;
  logic        AdrSrc, IRWrite, PCWrite, RegWrite;
  logic [2:0]  ALUControl;
  logic Zero;

  controller c(clk, reset, Instr[6:0], Instr[14:12], Instr[30], Zero, 
               ImmSrc, ALUSrcA, ALUSrcB, ResultSrc, AdrSrc, ALUControl, 
               IRWrite, PCWrite, RegWrite, MemWrite);

  datapath dp(clk, reset, ResultSrc, PCWrite, AdrSrc, MemWrite, RegWrite,
              IRWrite, ImmSrc, ALUSrcA, ALUSrcB, ALUControl, ReadData, Zero, 
              Adr, Instr, WriteData);
endmodule
