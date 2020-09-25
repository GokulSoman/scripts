T = int(input("Enter no: of test: "))
n,m = map(int, input().split())
def travel(a,b):
    for i in range(a):
            for j in range(b):
                if i + 1 == a:
                    return 1
                elif j + 1 == b:
                    return 1
                elif a < 0 or b < 0:
                    return 0
                else:
                    return travel(a-1,b) + travel(a,b-1)

count = travel(n,m)
print(count)