# -*- coding: utf-8 -*-
"""
Created on Wed Sep 16 10:34:28 2020

@author: gokul
"""
import unittest

class sortTest(unittest.TestCase):
    def test_emptyList(self):
        testcase = []
        expected = "Empty List"
        self.assertEqual(testcase, expected, "Empty list not detected")
        
unittest.main()