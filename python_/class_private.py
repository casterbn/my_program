#!/usr/bin/python

class class_test:
    __private_count = 0
    public_count = 0;

    def count(self):
        self.__private_count += 1
        self.public_count += 1
        print self.__private_count
count_test = class_test()
count_test.count()
count_test.count()
print count_test.public_count
print count_test._class_test__private_count
