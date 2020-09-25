import time
import math

def fibonacci1(n):
    '''This uses the naive fibonacci algorithm. An example of a binary tree'''
    if n < 2:
        return n
    else:
        return fibonacci1(n-1) + fibonacci1(n-2)

memo = {}
def fibonacci2(n):
    ''' This function uses a memo of computed values to speed up the process'''
    if n in memo:
        return memo[n]
    if n < 2:
        return n
    f = fibonacci2(n-1) + fibonacci2(n-2)
    memo[n] = f
    return f

def fibonacci3(N):
    '''This function uses the bottom up dynamic algorithm.'''
    fib = {}
    for k in range(N+1):
        if k < 2:
            fib[k] = k
        else:
            fib[k] = fib[k-1] + fib[k-2]       
    return fib[N]

def fibonacci4(N):
    """This uses the golden ratio to reduce the time and space complexity to O(1).
    This was run and tested. Accurate only upto 70"""
    goldenRatio = (1 + math.sqrt(5))/2.0
    #This formula is known as the Binet's formula
    return round((pow(goldenRatio,N) - pow((1-goldenRatio),N))/math.sqrt(5))




def screen(i,N):
    if i==1:
        return fibonacci1(N)
    if i == 2:
        return fibonacci2(N)
    if i==3:
        return fibonacci3(N)
    if i==4:
        return fibonacci4(N)

N = int(input("Enter a number: "))

for i in range(1,4):
    t0 = time.time()
    fib = screen(i+1,N)
    print("The "+ str(N) +"th fibonacci number is {}".format(fib))
    t1 = time.time()
    print("Time taken is : {}s".format(t1-t0))

#import sys
#print(sys.getrecursionlimit())
