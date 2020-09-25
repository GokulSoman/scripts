def maxProfit1(prices) -> int: 
    """Say you have an array prices for which the ith element is the price of a given stock on day i. 
        Design an algorithm to find the maximum profit. You may complete as many transactions as you like (i.e., buy one and sell one share of the stock multiple times).
        Note: You may not engage in multiple transactions at the same time (i.e., you must sell the stock before you buy again).
        REFER: https://leetcode.com/problems/best-time-to-buy-and-sell-stock-ii/solution/"""
    #bought = False
    #if len(prices) > 0:
    # min = prices[0]
    i = 0
    prof = 0
    max, min = 0, 0
    while i in range(len(prices)):
        while i in range(len(prices) - 1) and prices[i] >= prices[i+1]:
            i += 1
        min = prices[i]
        while i in range(len(prices) - 1) and prices[i] <= prices[i+1]:
            i += 1
        max = prices[i]
        prof += max - min
        if i == len(prices) - 1:
            break
    return prof

stockPrice = [int(i) for i in input("Enter the stock prices for each day: ").split()]
profit = maxProfit1(stockPrice)
print("The maximum possible profit is: {}".format(profit))