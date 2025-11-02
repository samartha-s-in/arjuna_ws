#! /usr/bin/env python

import rospy

class Student:
    def __init__(self , student_name, student_age , grade):
        self.S_Name = student_name
        self.S_Age = student_age
        self.S_Grade = grade

    def getgrade(self):
        return self.S_Grade

class Course:
    def __init__(self , course_name , max_sudents):
        self.C_Name = course_name
        self.Max_Students = max_sudents
        self.Students_list = []

    def add_student(self , student): #student is the object that we create for the class Student
        if len(self.Students_list) < self.Max_Students:
            self.Students_list.append(student)

S1= Student("Nik" , 26 , 90)
S2 = Student("bin" , 25 , 80)

print(S1.getgrade())

course = Course("Science" , 2)
course.add_student(S1)
course.add_student(S2)
print(course.Students_list[0].S_Name)
print(course.Students_list[1].S_Name)

