#! /usr/bin/env python3

#Class Attributes
import rospy

# class Person:
#     number_of_people = 9 # class Attribute
#     def __init__(self , name ):
#         self.Name = name

# p1 = Person("Nik")
# p2 = Person("Bin")


# print(p1.number_of_people)
# print(p2.number_of_people)
# print(Person.number_of_people)


print("---------------------------------------------------------------------")

#Class Methods
class Person:
    number_of_people = 9 # class Attribute
    def __init__(self , name ):
        self.Name = name

    @classmethod   # Class method
    def update_people_1(cls):
        cls.number_of_people = cls.number_of_people + 1
        return cls.number_of_people

    @classmethod   # Class method
    def update_people_2(cls , var):
        cls.number_of_people = var + 1
        return cls.number_of_people

p1 = Person("Nik")
p2 = Person("Bin")


print(p1.number_of_people)
print(p2.number_of_people)
print(Person.number_of_people)

print(Person.update_people_1())
print(Person.update_people_2(5))