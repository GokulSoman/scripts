# -*- coding: utf-8 -*-
"""
Created on Wed Sep 16 10:20:10 2020

@author: gokul
"""

def insertionSort(arr: List[int]):
    n = len(arr)
    if n == 0:
        return "Empty List"
    for i in range(1,n):
        elem = arr[i]
        j = i-1
        while j >= 0 and arr[j]>elem:
            arr[j+1] = arr[j]
            j--
        arr[j+1] = elem
    return arr

n = int(input("Enter size of array: "))
arr = map(func, int(input().rstrip().split()))

