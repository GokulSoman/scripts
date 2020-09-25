houses = [6,7,1,3,8,2,5]
total = []
test = []
''''
for i in range(2):
    for j in range(1, len(houses)):
        if i == 0:
            test[0] = 1
    total[i] = max(houses[j], houses[j+1])

def count(n):
    for i in range(3):
        if n[i]
'''
for i in range(1, len(houses)-1):
    test = houses[:3]
    if i==1:
        print(test)
    if houses[i + 1] >= houses[i] + houses[i + 2]:
        test.extend([0,1,0]) 
    
