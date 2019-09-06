#!/usr/bin/python

class Parent:
    parent_attr = 100

    def __init__(self):
        print "parent init function"
    def parent_method(self):
        print "parent function"
    def set_attr(self,attr):
        Parent.parent_attr = attr
        print "set attr"
    def get_attr(self):
#        return self.parent_attr
        return Parent.parent_attr
class child(Parent):
    def __init__(self):
        print "child init function"
    def child_method(self):
        print "child function"

child_test = child()
child_1 = child()
child_1.set_attr(250)
child_test.child_method()
child_test.parent_method()
child_test.set_attr(200)
value = child_test.get_attr()
print "value = %d" % value
print issubclass(child,Parent)

print isinstance(child(),child)


