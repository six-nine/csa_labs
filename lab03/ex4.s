.globl iterative
.globl recursive

.data
n: .word 3
m: .word 3

.text
main:
    la t0, n
    la t1, m
    lw a2, 0(t0)
    lw a3, 0(t1)
    jal ra, recursive

    addi a1, a0, 0
    addi a0, x0, 1
    ecall # Print Result

    addi a1, x0, '\n'
    addi a0, x0, 11
    ecall # Print newline

    addi a0, x0, 10
    ecall # Exit

tester:
    # YOUR CODE HERE
    # вызвать итеративную и рекурсивные функции, сравнить ответ и вернуть результат, если совпал. иначе вернуть -1.
    
recursive:

    # BEGIN PROLOGUE
    addi sp, sp, -20
    sw a2, 0(sp)
    sw a3, 4(sp)
    sw ra, 8(sp)
    sw s2, 12(sp)
    sw s3, 16(sp)
    # END PROLOGUE

    # a2 - n, a3 - m   
    beq a3, x0, l1

    beq a2, x0, l2

    j l3

    l1:
        addi a0, a2, 1
        j exit
    l2:
        add s2, x0, a2
        add s3, x0, a3
        addi a3, a3, -1
        addi a2, x0, 1
        jal recursive
        add a2, x0, s2
        add a3, x0, s3
        j exit
   l3:
	    add s2, x0, a2
        add s3, x0, a3
        
        addi a2, a2, -1
        jal recursive
        add t0, x0, a0
        
        addi a3, a3, -1
        add a2, x0, t0
        jal recursive
        add a2, x0, s2
        add a3, x0, s3
        j exit
  
    exit:
    # BEGIN EPILOGUE
    lw a2, 0(sp)
    lw a3, 4(sp)
    lw ra, 8(sp)
    lw s2, 12(sp)
    lw s3, 16(sp)
    addi sp, sp, 20
    # END EPILOGUE
    jr ra
