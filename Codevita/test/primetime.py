def getPrimes(N):
    notprimes = []
    primes = []
    for i in range(2,N):
        if i in notprimes:
            continue
        else:
            if i not in primes:
                primes.append(i)
                j = i
                for j in range(2*i,N,i):
                    if j not in notprimes:
                        notprimes.append(j)
    return primes

D,P = [int(i) for i in input().split()]
count = 0
if D % P == 0:
    prime1 = getPrimes(D)
    prime2 = getPrimes(D//P)
    test = prime1[len(prime2):]
    second = [i for i in test if i ]
    temp = {}
    for j in prime2:
        temp[j] = 0
        for i in test:
            for k in range(1,P):
                if i == j + k * (D//P):
                    temp[j] += 1
                    if temp[j] == P-1:
                        count += 1
           
       
    print(count)

else:
    print("Invalid Input")