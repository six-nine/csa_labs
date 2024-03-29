# Дирректива .globl определяет функции, которые мы хотим экспортировать
# похоже на размещение объявления функции в заголовке в Cи
.globl f

.data
# asciiz директива используется дял размещения строк
# asciiz автоматически добавит нулевой символ в конец строки
# FIXME исправьте строки, чтобы они выдавали корректные отладочные значения 
case1:   .asciiz "f(-5) should be 25, and it is: "
case2:   .asciiz "f(-3) should be 8, and it is: "
case3:   .asciiz "f(-1) should be 6, and it is: "
case4:   .asciiz "f(0) should be 3, and it is: "
case5:   .asciiz "f(1) should be -12, and it is: "
case6:   .asciiz "f(3) should be -34, and it is: "
case7:   .asciiz "f(5) should be 5, and it is: "

# FIXME Разместите значения из вашего варианта в этом массиве 
output: .word   25, 8, 6, 3, -12, -34, 5

.text
main:
	######### перебор случаев, случай 1 (case1) #########
    # загружаем адрес строки case1 в a0
    # это послужит аргументом функции print_str
    la a0, case1 
    # Выводим строку по адресу case1
    jal print_str 
    # Загружаем первый аргумент функции f в a0
    # FIXME Подставьте первый аргумент case1
    li a0, -5 
    # загружаем второй аргумент функции f в a1
    # `output` -- это указатель на массив, который содержит возможные выходные значения f
    la a1, output
    # выполняем f(case1)
    jal f     
    # f вернет результат f(-3) в регистре a0
    # чтобы отобразить это значение мы вызовем print_int
    # print_int ожидает значение аргумента в регистре a0
    # значение уже находится в a0, не требуется перемещений
    jal print_int
    # print a new line
    jal print_newline

	######### перебор случаев, случай 2 (case2) #########
    la a0, case2
    jal print_str
    # FIXME Подставьте первый аргумент case1
    li a0, -3
    la a1, output
    jal f                
    jal print_int
    jal print_newline

	######### перебор случаев, случай 3 (case3) #########
    la a0, case3
    jal print_str
    # FIXME Подставьте первый аргумент case1
    li a0, -1
    la a1, output
    jal f               
    jal print_int
    jal print_newline

	######### перебор случаев, случай 4 (case4) #########
    la a0, case4
    jal print_str
    # FIXME Подставьте первый аргумент case1
    li a0, 0
    la a1, output
    jal f               
    jal print_int
    jal print_newline

	######### перебор случаев, случай 5 (case5) #########
    la a0, case5
    jal print_str
    # FIXME Подставьте первый аргумент case1
    li a0, 1
    la a1, output
    jal f                
    jal print_int
    jal print_newline

	######### перебор случаев, случай 6 (case6) #########
    la a0, case6
    jal print_str
    # FIXME Подставьте первый аргумент case1
    li a0, 3
    la a1, output
    jal f               
    jal print_int
    jal print_newline

	######### перебор случаев, случай 7 (case7) #########
    la a0, case7
    jal print_str
    # FIXME Подставьте первый аргумент case1
    li a0, 5
    la a1, output
    jal f                
    jal print_int
    jal print_newline

	# передаем 10 в ecall чтобы завершить программу
    li a0, 10
    ecall

# f принимает два аргумента:
# a0 значение для которого мы хотим вычислить функцию f
# a1 адрес выходного ("output") массива, содержащего все допустимые варианты.
f:
    addi t1, x0, 1
    slli t1, t1, 31
    and  t1, t1, a0 # t1 is 1000..00 if a0 is negative, and 0 otherwise
    srai t1, t1, 30 # t1 is 1111..10 if a0 is negative, and 0 otherwise
    ori  t1, t1, 1  # t1 is sgn(a0) 1 (1 or -1)

    add  a0, a0, t1 # [-5, -3, -1, 0, 1, 3, 5] ->[-6, -4, -2, 1, 2, 4, 6]
    srai a0, a0, 1  # [-3, -2, -1, 0, 1, 2, 3]
    addi a0, a0, 3  # [0, 1, 2, 3, 4, 5, 6]

    slli a0, a0, 2
    add t0, a1, a0

    lw a2, 0(t0)
    
    # FIXME
    # YOUR CODE GOES HERE!
    add a0, x0, a2
    jr ra               # Всегда вызывайте jr ra для выхода из функции!

# печатает одно целое число
# вход: a0: число на печать
# ничего не возвращает
print_int:
	# to print an integer, we need to make an ecall with a0 set to 1
    # the thing that will be printed is stored in register a1
    # this line copies the integer to be printed into a1
    mv a1, a0
    # set register a0 to 1 so that the ecall will print
    li a0, 1
    # print the integer
    ecall
    # return to the calling function
    jr    ra

# печатает строку
print_str:
    mv a1, a0
    li a0, 4 # tells ecall to print the string that a1 points to
    ecall
    jr    ra

print_newline:
    li a1, '\n'
    li a0, 11 # tells ecall to print the character in a1
    ecall
    jr    ra
