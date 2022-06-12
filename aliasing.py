# -*- coding: utf-8 -*-
"""
Created on Wed Aug  4 09:57:07 2021

@author: james
"""
class robot:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        
    def move(self, dx, dy):
        self.x = self.x + dx
        self.y += dy
        
    
    def copy(self):
        return robot(self.x, self.y)
        
    def __repr__(self):
         return '[%f, %f]' % (self.x, self.y)
        
''' Aliasing does occur with integer, float, boolean, and string values '''
''' Using an assignment statement creates a copy '''
x = 1
y = x
x = 2
print('x = %d  y = %d  Aliased? %s' % (x, y, x is y))

x = 1.0
y = x
x = 2.0
print('x = %f  y = %f  Aliased? %s' % (x, y, x is y))

x = True
y = x
x = False
print('x = %s  y = %s  Aliased? %s' % (str(x), str(y), x is y))

x = 'a'
y = x
x = 'b'
print('x = %s  y = %s  Aliased? %s' % (x, y, x is y))

''' Aliasing occurs with other data types '''
x = [0, 1, 2]
y = x
x[0] = 99
print('\n\nx = %s\ny = %s  \nAliased? %s' % (str(x), str(y), x is y))

myrobot = robot(1.0, 2.0)
print('\n\nOriginal robot location [%f, %f]' % (myrobot.x, myrobot.y))
rob_list = []
for i in range(3):
    rob_list.append(myrobot)
print('Move original robot')
myrobot.move(1.0, -1.0)
for i in range(len(rob_list)):
    print('Robot ' + str(i) + ': ', end = '')
    print(rob_list[i])
print('Move robot in list')
rob_list[0].move(1.0, -1.0)
for i in range(len(rob_list)):
    print('Robot ' + str(i) + ': ', end = '')
    print(rob_list[i])
    
print('\n\nCurrent robot location [%f, %f]' % (myrobot.x, myrobot.y))
rob_list = []
for i in range(3):
    rob_list.append(myrobot.copy())
print('Move original robot')
myrobot.move(1.0, -1.0)
for i in range(len(rob_list)):
    print('Robot ' + str(i) + ': ', end = '')
    print(rob_list[i])
print('Move robot in list')
rob_list[0].move(1.0, -1.0)
for i in range(len(rob_list)):
    print('Robot ' + str(i) + ': ', end = '')
    print(rob_list[i])
