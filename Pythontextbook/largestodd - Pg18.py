# -*- coding: utf-8 -*-
"""
Created on Tue Aug 18 14:49:58 2020

@author: gokul

Print largest odd number between 3 numbers. If no odd numbers, output
an error message. Only use conditionals.
"""

def largestOdd(numbers):
    if numbers[0]%2 + numbers[1]%2 + numbers[2] % 2 == 0:
        print("Enter atleast one odd number")
    else:
        if numbers[0] % 2 !=0:
            if numbers[1] % 2 !=0:
                if numbers[0] > numbers[1]:
                    largest = numbers[0]
                else:
                    largest= numbers[1]
            else:
                largest = numbers[0]
            if numbers[2] % 2 != 0 and numbers[2]> largest:
                largest = numbers[2]
        elif numbers[1] %2 !=0:
            largest = numbers[1]
            if numbers[2] % 2 != 0 and numbers[2] > largest:
                largest = numbers[2]
        else:
            largest = numbers[2]
        print("Largest Odd number is : {}".format(largest))

largestOdd([20,30,0])