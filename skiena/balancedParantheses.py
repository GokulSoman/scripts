# -*- coding: utf-8 -*-
"""
Created on Mon Sep 21 09:48:50 2020

@author: gokul
"""


def balanced1(s):
    """ Implementation using naive searching. I coudnt provide 
    index of unbalanced '(' """ 
    count = 0
    i = 0
    left = None
    while count >= 0 and i<len(s):
        if s[i]==")" and count == 0:
            print("String contains unbalanced parantheses at {}".format(i))
            return False
        if s[i] == "(":
            if left is None:
                left = i
            count += 1
        elif s[i] == ")":
            count -= 1
            if count == 0:
                left = None
        i += 1
    if count == 0:
        print("All parentheses in string are balanced")
        return True
    else:
        
        print("String contains unbalanced parantheses at {}".format(left))
        return False
class Stack():
    def __init__(self):
        self.parens = []
    def add(self,i):
        self.parens.append(i)
    def peek(self):
        return self.parens[-1]
    def remove(self):
        self.parens.pop()
        

def balanced2(s):
    """ Implementation using stack. Works perfectly."""
    aStack = Stack()
    for i in range(len(s)):
        if s[i] == ')':
            if len(aStack.parens)==0:
                print("String contains unbalanced parantheses at {}".format(i))
                return False
            aStack.remove()
        elif s[i] == "(":
            aStack.add(i)            
    if len(aStack.parens) == 0:
        print("All parentheses in string are balanced")
        return True
    else:
        print("String contains unbalanced parantheses at {}".format(aStack.parens[0]))
        return False
    
    
if __name__ == "__main__":
    sentence = input("Enter a string: ")
    balanced1(sentence)
        
        