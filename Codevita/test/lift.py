N, M = [int(i) for i in input().split()]
values = []
for line in range(N):
    values.append([int(i) for i in input().split()])
print(values)
values = sorted(values, key = lambda x:x[0])
lift = {}
pos = {}
'''for i in range(M):
    lift[i] = {}
    pos[]
'''
times = [item[0] for item in values]
source = [item[1] for item in values]
dest = [item[2] for item in values]
t= 0
#print(values[N-1][0])
i = 0

print(times)
'''
while t <= values[N-1][0]:
    if t==0:
        for k in range(M):
        lift[k] = 0
        pos[k] = values[k][1]


    t += 1
    sum += sum(lift)
'''