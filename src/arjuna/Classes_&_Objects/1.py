#! /usr/bin/env python

import rospy

class Newrro:
    def Nik(self):
        print("Iam a Designer")
    def Bin(self):
        print("Iam a Electronics Engineer")
    def Add(self , x , y):
        return x+y

NR = Newrro()

NR.Nik()
NR.Bin()
print(NR.Add(5,4))
