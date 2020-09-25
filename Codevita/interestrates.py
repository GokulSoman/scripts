P = int(input("Enter principal amount:" ))
T = int(input("Enter total tenure in years: "))
N1 = int(input("Enter No. of Slabs for Bank A: "))
emiA,emiB = 0,0
rateA,rateB, periodA,periodB = {},{},{},{}
print("Enter rate and period for Bank A")
for i in range(N1):
    rateA[i], periodA[i] = input().split()
    rateA[i], periodA[i] = float(rateA[i]), int(periodA[i])
    emiA += P * rateA[i] / (1 - 1/(1 + rateA[i])**(periodA[i]*12))
N2 = int(input("Enter No. of Slabs for Bank B: "))
print("Enter rate and period for Bank B")
for i in range(N2):
    rateB[i], periodB[i] = input().split()
    rateB[i], periodB[i] = float(rateB[i]), int(periodB[i])
    emiB += P * rateB[i] / (1 - 1/(1 + rateB[i])**(periodB[i]*12))


if(emiA > emiB):
    print("Bank B")
elif emiB > emiA:
    print("Bank A")
else:
    print("No difference")