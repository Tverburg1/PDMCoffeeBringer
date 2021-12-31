# -*- coding: utf-8 -*-
"""
Created on Sun Dec 26 21:12:05 2021

@author: jevon
"""
import numpy as np

with open("path.txt", "r") as file:   
    lines = [line.rstrip() for line in file]
    
    path = []
    for line in lines:
        config = [float(i) for i in line.split()]
        
        path.append(config[:2])
    path = np.array(path, np.float32)

print(path.shape)