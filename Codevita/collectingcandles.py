tests = int(input())
for i in range(tests):
    boxes = int(input())
    candies = [int(i) for i in input().split()]
    print("Candies: {}".format(candies))
    time = 0
    temp = 0
    for i in range(boxes - 1):
        if i == 0:
            temp = candies[i] + candies[i+1]
        else:
            temp += candies[i+1] 
        time += temp
    print("Time: {}".format(time))