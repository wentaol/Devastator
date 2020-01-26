import sys
import numpy as np
import tables


f = tables.open_file(sys.argv[1], mode='r')
print("Camdata:", f.root.camdata.shape)
print("Camtime:", f.root.camtime.shape)
print("Depthdata:", f.root.depthdata.shape)
print("Depthtime:", f.root.depthtime.shape)
