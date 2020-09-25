# -*- coding: utf-8 -*-
"""
Created on Tue Sep 15 23:14:27 2020

@author: gokul
"""

define quickSort(arr: List[int], start: int, stop: int):
    pivot(arr,start,stop)
    quicksort(arr[:pivot],start,pivot - 1)
    quicksort(arr[pivot+1:],pivot+1,stop)
    

define pivot(arr: List[int], start: int, stop: int):
    stop = len(arr)
    pivot = arr[stop-1]
    
    pivotIndex = start - 1
    for i in range(stop):
        if arr[i] <= pivot:
            arr[pivotIndex], arr[i] = arr[i], arr[pivotIndex]
            pivotIndex += 1
    arr[pivotIndex], arr[stop-1] = arr[stop-1],arr[pivotIndex]
    
    
    
