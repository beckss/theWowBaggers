# -*- coding: utf-8 -*-
"""
Spyder Editor

This temporary script file is located here:
/home/mosaic/.spyder2/.temp.py
"""
import math    

def add(a, b):
    return [a[0]+b[0], a[1]+b[1]]
    
def neg(a):
    return [-a[0], -a[1]]
    
def normalize(a):
    length = math.sqrt(a[0]*a[0]+a[1]*a[1])
    return [a[0]/length, a[1]/length]
    
def scale(a, s):
    return[a[0]*s, a[1]*s]
    
def getProjection(point, linea, lineb):
    dirVect = normalize(add(lineb, neg(linea)))
    if dirVect[0] == 0:
        distance = point[0] - linea[0]
    elif dirVect[1] == 0:
        distance = -point[1] + linea[1]
    else:
        distance = ((point[0] - linea[0])/dirVect[0] - (point[1] - linea[1])/dirVect[1])/2
    return add(point, scale([-dirVect[1], dirVect[0]], distance))
    
print getProjection([0.1, 0.1], [0, 0], [0, 0.2])