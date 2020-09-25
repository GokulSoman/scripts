N, K = [int(i) for i in input().split()]
arr = [int(i) for i in input().split()]
happy = 0
for i in range(len(arr)):
    for j in range(len(arr)):
        if i == j:
            continue
        else:
            if abs(arr[i] - arr[j]) <= K:
                happy += 1
                break
print(happy)


