def numberTypes(N):
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

def counting(primes):
    sum = 0
    count = 0
    requiredprimes = []
    for i in primes:
        sum = sum + i
        if sum in primes and sum > 2:
            count += 1
            requiredprimes.append(sum)
    print("Required Primes: {}".format(requiredprimes))
    return count

def main():
    print("Enter a number")
    N = int(input())
    if(N>2 and N<=12000000000):
        primes = numberTypes(N)
        count = counting(primes)
        print("The number of such prime numbers are: {}".format(count))
    else:
        print("Invalid Input")
main()



