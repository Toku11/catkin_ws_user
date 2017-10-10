#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Oct  2 17:24:06 2017

@author: fatoks
"""
import numpy as np
import sys
import time
from matplotlib import pyplot as plt
c=[]
for line in file('ruta.txt','r'):
    c.append([float(line.split(',')[0]),
              float(line.split(',')[1]),float(line.split(',')[2])])
c=np.asarray(c)
latitud=c[:,1]
longitud=c[:,0]
altura=c[:,2]

#%%%%%%%%%%CONSTANTS
llh0=c[0]
llh=c
#%llh=[latitud(:,1),longitud(:,1),altitud(:,1)];
a = 6378137
b = 6356752.3142
e2 = 1 - (b/a)**2
#%%%%%%%%%%Location of reference point in radians
phi = llh0[1]*np.pi/180;
lam = llh0[0]*np.pi/180;
h = llh0[2]

#%%%%%%%%%%Location of data points in radians
dphi= llh[:,1]*np.pi/180 - phi;
dlam= llh[:,0]*np.pi/180 - lam;
dh = llh[:,2] - h;
#%%%%%%%%%%Some useful definitions
tmp1 = np.sqrt(1-e2*np.sin(phi)**2);
cl = np.cos(lam);
sl = np.sin(lam);
cp = np.cos(phi);
sp = np.sin(phi);
#%%%%%%%%%%Transformations
de = (a/tmp1+h)*cp*dlam - (a*(1-e2)/(tmp1**3)+h)*sp*dphi*dlam +cp*dlam*dh
dn = (a*(1-e2)/tmp1**3 + h)*dphi + 1.5*cp*sp*a*e2*dphi**2 + sp**2*dh*dphi + 0.5*sp*cp*(a/tmp1 +h)*dlam**2
du = dh - 0.5*(a-1.5*a*e2*cp**2+0.5*a*e2+h)*dphi**2 - 0.5*cp**2*(a/tmp1 -h)*dlam**2
denu = np.matrix(np.column_stack((de, dn, du)))
with open('outfile.txt','wb') as f:
    for line in denu:
        np.savetxt(f, line, fmt='%f')
plt.plot(de,dn)
plt.show()
plt.close(1)
sys.exit(0)
'''hold on
plot(denu(:,1),denu(:,2))'''