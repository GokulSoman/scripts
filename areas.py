# -*- coding: utf-8 -*-
"""
Created on Sat May 16 10:18:23 2020

@author: gokul
"""
import math

def triangle(base, height):
    area = .5 * base * height
    return area
def square(side):
    area = side**2
    return area
def rectangle(length,breadth):
    area = length * breadth
    return area
def circle(radius):
    area = math.pi * radius**2
    return area

def donut(outside_radius, inside_radius):
    area = circle(outside_radius) - circle(inside_radius)
    return area