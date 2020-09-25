def minCoins(denominations, nOfDenominations,target):
    """ This function is used to determine the minimum number of coins of given denominations to
    make the target amount. It is assumed that the targetAmount can always be created with the given denominations."""
    count = target//denominations[-1]
    if len(denominations)==0:
        return 0
    if type(count) is int:
        return 0 for i in range()]