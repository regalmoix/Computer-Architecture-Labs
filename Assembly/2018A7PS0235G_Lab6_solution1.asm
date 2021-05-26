.data
userin: .asciiz "Enter n : \n"


.text
    # Ask input
    li $v0, 4
    la $a0, userin
    syscall

    # Read input
    li $v0, 5
    syscall

    # Call fibonacci
    move $a0, $v0
    jal fibonacci

    jal endmain

    ## Function Definition
    fibonacci:
        beq $a0, 0, edgecasehandler         # For input = 0
        bge $a0, 46, edgecasehandler        # If input >= 46

        addi $sp, $sp, -12                  # Stack frame size 12
        sw $ra, 8($sp)
        sw $s0, 4($sp)                      # Input paramter            
        sw $s1, 0($sp)
        
        move $s0, $a0
        li $v0, 1                           # return value for terminal condition
        ble $s0, 0x2, fibonacciwrapup       # check termination condition
         
        
        #Fib(n-1) function call
        
            addi $a0, $s0, -1               # For calling fib(n-1) we subtract
            jal fibonacci                   # Call made to function with above set paramter
            move $s1, $v0                   # store calculated result of f(n-1) [$v0] to $s1
        
        #Fib(n-2) function call
        
            addi $a0, $s0, -2               # For calling fib(n-2) we subtract
            jal fibonacci                   # Call made to function with above set paramter
            add $v0, $s1, $v0               # store calculated result of f(n-2) [$v0] + $s1 in $v0 : thus storing fib(n). 
    
    fibonacciwrapup:
        
        lw $ra, 8($sp)
        lw $s0, 4($sp)
        lw $s1, 0($sp)
        addi $sp, $sp, 12
        jr $ra

    edgecasehandler:
        li $v0, 0
        jr $ra
    
    endmain: 
    	li $v0, 10
    	syscall
