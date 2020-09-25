N = int(input())
a,b = [],[]
values = []
for i in range(N):
    values.append([int(i) for i in input().split()])
def flatten(values):
    a,b = [], []
    flat_list = [item for sublist in values for item in sublist] #https://stackoverflow.com/questions/952914/how-to-make-a-flat-list-out-of-list-of-lists
    for i in range(len(flat_list)):
        if i % 2 == 0:
            a.append(flat_list[i])
        else:
            b.append(flat_list[i])
    return a,b


def sorting(mat):
    return sorted(mat) #, key=lambda x: x[0]) 





def minPlatform(values):
    
    values = sorting(values)
    #print(values)
    a,b = flatten(values)
    d= [a[i] + b[i] for i in range(len(a))]
    d = sorting(d)
    maximum = 0
    p = 0
    count = 0

    i,j = 0,0

    while i < len(a):
        if a[i] <= d[j]:
            count = count + 1
            maximum = max(p,count)
            i = i + 1
        else:
            count = count - 1
            j = j+1

    return maximum

print(minPlatform(values))
'''
    for i in range(len(values)):
    
        if a[i+1] <= d[i]
            count += 1
            if max < count:
                max = count
        elif d[i] < a[i + 1]:
            count -= 1
        if a[]
        
        for j in range(i+1 ,len(values)):
            if a[j] < d[i]:
                count+ = 1
            elif d[i] < a[j]:
                count -= 1
                break







                p+= 1
                if 
                return 1 + 

                if p > max:
                    max = p
                
        if a[i] < min:
            min = a[i]
            time = a[i] + b[i]
            num = i
            p += 1
    for i in range(len(a)):
        if a[i] <= time and i!=num:
            p +=1
'''