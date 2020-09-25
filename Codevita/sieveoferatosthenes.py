#The content has been copied from https://stackoverflow.com/questions/7370801/how-to-measure-elapsed-time-in-python
import time

#start = time.time()
#print("hello")
#end = time.time()
#print(end - start)

#copied content


def sieveOfEratosthenes(n):
    primes = [True for i in range(n+1)]
    primelist = []
    p = 2
    while(p<=n):
        if primes[p] == True:
            primelist.append(p)
            for i in range(p*p,n+1,p):
                primes[i] = False
        p += 1

    return primelist

start = time.time()
N = int(input("Enter a number: "))
primeNumbers = sieveOfEratosthenes(N)
values = ", ".join([str(i) for i in primeNumbers])
print("The primes are: "+ values)
end = time.time()
print("Time taken for program : {}s".format(end - start))

