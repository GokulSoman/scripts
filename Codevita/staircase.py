n= int(input("Enter Number of stairs: "))
def jump(steps):
    dist = steps
    if dist == 0:
        return 1
    elif steps > 0:
        return jump(steps-1) + jump(steps-2)
    else:
        return 0

print("Ways: {}".format(jump(n)))