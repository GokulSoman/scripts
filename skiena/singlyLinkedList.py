# -*- coding: utf-8 -*-
"""
Created on Mon Sep 21 16:02:34 2020

@author: gokul
"""


class Node:
    def __init__(self, val = None, nextNode = None):
        self.value = val
        self.next = None
    def __str__(self):
        return self.value
    def get(self):
        return self.value
    def getNext(self):
        return self.next
    def setNext(self, new):
        self.next = new

class SLinkedList: 
    def __init__(self, head = None):
        self.head = Node(head)
        self.size = 0
        if self.head != None:
            self.size = 1
    def insert(self, val):
        newHead = Node(val)
        newHead.setNext(self.head)
        self.head = newHead
        self.size += 1
    def traverse(self, val):
        temp = self.head
        while temp.get() != None:
            print(temp.value, end= " ")
            temp = temp.next
            if temp == None:
                break
        print("")
    def getSize(self):
        print("List is of size: {}".format(self.size))
    """
    def delete(self, val = self.head.value):
        if self.head == None:
            return "Cannot delete from empty list"
        else:
            while self.head!= None:
                if self.head == val:
                    temp = self.head
                    self.head    """       
if __name__ == "__main__":
    aList = SLinkedList(5)
    aList.insert(3)
    aList.insert(4)
    aList.traverse()
    aList.getSize()
    print(type(aList.head))
        
            
    