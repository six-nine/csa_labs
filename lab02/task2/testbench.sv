`include "game.sv"

module testbench(); 
  logic        clk, reset;
  logic [1:0]  dir, result;
  logic [1:0]  expected;
  logic [31:0] vectornum, errors;
  logic [3:0]  testvectors[10000:0];

// инстанцировать тестируемое устройство 
game dut(clk, reset, dir, result); 

// generate clock 
always 
  begin
    clk=1; #5; clk=0; #5; 
  end 

// на старте теста, загрузите вектора и запустите сброс
initial 
  begin
    $dumpfile("testbench.vcd");
    $dumpvars(0, testbench);

    $readmemb("testvectors.tv", testvectors); 
    vectornum = 0; errors = 0; reset = 1; #22; reset = 0; 
  end 

// применение тестовых векторов по нарастающему фронту тактового сигнала
always @(posedge clk) 
  begin
    #1; {dir, expected} = testvectors[vectornum]; 
  end 

// проверка результатов по спадающему фронту сигнала clk
always @(negedge clk) 
  if (~reset) begin    // skip during reset
    if (result !== expected) begin // check result 
      $display("Error: inputs = %b", dir);
      $display(" outputs = %b (%b expected)", 
        result, expected); 
      errors = errors + 1; 
    end
    if (result !== 2'b00)
    begin
      reset = 1; #22; reset = 0;
    end
    vectornum = vectornum + 1;
    if (testvectors[vectornum] === 4'bx) begin 
      $display("%d tests completed with %d errors", vectornum, errors); 
      $finish;
    end 
  end 
endmodule 
