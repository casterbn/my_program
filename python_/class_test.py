class Employee:
    empcount = 0
    
    def __init__(self,name,salary):
        self.name = name
        self.salary = salary
        Employee.empcount += 1

    def __del__(self):
        class_name = self.__class__.__name__
        print class_name,"distroy"

    def displayCount(self):
        print "total %d" % Employee.empcount

    def displayEmployee(self):
        print "name:",self.name,"salary:",self.salary

test = Employee("daichenghe",10000)
test.displayCount()
test.displayEmployee()
del test

"""
print Employee.__doc__
print Employee.__name__
print Employee.__module__
print Employee.__bases__

print Employee.__dict__
"""
