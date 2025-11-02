#! /usr/bin/env python3

import rospy

class Math:
    @staticmethod
    def add(var1 , var2):
        Add = var1 + var2
        return Add

    @staticmethod
    def sub(var1 , var2):
        Sub = var1 - var2
        return Sub

object_1 = Math.add(10,20)
print(object_1)

object_2 = Math.sub(30, 20)
print(object_2)