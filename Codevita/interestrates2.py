bank = []
principal = int(input())
years = int(input())
for i in range(2):
    sum = 0
    slab = int(input())
    for j in range(slab):
        period, rate = [float(j) for j in input().split()]
        term = pow(1+rate, years * 12) 
        emi = principal * rate / (1 - term)
        sum += emi
    bank.append(sum)
if bank[0] < bank[1]:
    print("Bank A")
else:
    print("Bank B")