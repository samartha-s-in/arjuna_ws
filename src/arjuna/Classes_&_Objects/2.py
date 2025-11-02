#! /usr/bin/env python

import rospy

class Newrro:
    def __init__(self , name , age):
        self.NAME = name
        self.AGE = age

    def getdata(self):
        return self.NAME ,self.AGE 
    
    def setdata(self ,name , age):
        self.NAME = name
        self.AGE = age
        return self.NAME , self.AGE


NR_1 = Newrro("Nik" , 25)
print(NR_1.NAME)

NR_2 = Newrro("Bin" , 26)
print(NR_2.NAME)

print("-----------------")

print(NR_1.getdata())
print(NR_2.getdata())

print("----------------")

print(NR_1.setdata("Raja" , 24))