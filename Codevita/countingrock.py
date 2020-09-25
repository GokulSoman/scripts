samples, nofranges = [int(i) for i in input().split()]
sampSize = [int(i) for i in input().split()]
print(sampSize)
ranges = {}
sorted = {}
temp = sampSize
for i in range(nofranges):
    ranges[i] = [int(i) for i in input().split()]
    sorted[i] = [j for j in temp if j >= ranges[i][0] and j <=ranges[i][1] ]
    print(len(sorted[i]))
    

