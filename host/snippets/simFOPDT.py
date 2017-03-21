
import numpy as np, sys
import matplotlib.pyplot as plt
from scipy.integrate import odeint
from scipy.optimize import minimize
from scipy.interpolate import interp1d

import ddprintutil as util

# Import CSV data file
# Column 1 = time (t)
# Column 2 = input (u)
# Column 3 = output (yp)
# data = np.loadtxt('data.txt',delimiter=',')
# u0 = data[0,1]
# yp0 = data[0,2]
# t = data[:,0].T
# u = data[:,1].T
# yp = data[:,2].T

#######################################################################################

f=open(sys.argv[1])
raw = util.jsonLoad(f)
# data = raw["data"]
data = raw["data"]

Xo = raw["Xo"]
# data = []
# for tim, temp in d:
    # data.append((tim, temp/Xo))
# endTemp = raw["tEnd"]/Xo

dataArray = np.array(data)
# u0 = dataArray[0,1]
u0 = 0.0
yp0 = dataArray[0,1]
t = dataArray[:,0].T
# u = dataArray[:,1].T
yp = dataArray[:,1].T

#######################################################################################

# specify number of steps
ns = len(t)
delta_t = t[1]-t[0]
# create linear interpolation of the u data versus time
u = np.zeros(ns)
u[10:] = 100.0
uf = interp1d(t,u)

# define first-order plus dead-time approximation    
def fopdt(y,t,uf,Km,taum,thetam):
    
    # print "start fopdt"
    # arguments
    #  y      = output
    #  t      = time
    #  uf     = input linear function (for time shift)
    #  Km     = model gain
    #  taum   = model time constant
    #  thetam = model time constant
    # time-shift u
    try:
        if (t-thetam) <= 0:
            um = uf(0.0)
        else:
            um = uf(t-thetam)
    except:
        #print('Error with time extrapolation: ' + str(t))
        um = u0
    # calculate derivative
    dydt = (-(y-yp0) + Km * (um-u0))/taum
    
    # print "end fopdt"
    return dydt

# simulate FOPDT model with x=[Km,taum,thetam]
def sim_model(x):
    print "start sim_model"
    # input arguments
    Km = x[0]
    taum = x[1]
    thetam = x[2]
    # storage for model values
    ym = np.zeros(ns)  # model
    # initial condition
    ym[0] = yp0
    # loop through time steps    
    for i in range(0,ns-1):
        ts = [delta_t*i,delta_t*(i+1)]
        y1 = odeint(fopdt,ym[i],ts,args=(uf,Km,taum,thetam))
        ym[i+1] = y1[-1]
    print "end sim_model"
    return ym

# define objective
def objective(x):
    print "start objective"
    # simulate model
    ym = sim_model(x)
    # calculate objective
    obj = 0.0
    for i in range(len(ym)):
        obj = obj + (ym[i]-yp[i])**2    
    # return result
    print "objective: ", x, obj
    print "end objective"
    return obj

# initial guesses
x0 = np.zeros(3)
# x0[0] = 3 # Km
x0[0] = 2.11281958 # Km
# x0[1] = 5 # taum
# x0[1] = 90 # taum
x0[1] = 105 # taum
# x0[2] = 1 # thetam
# x0[2] = 21 # thetam
# x0[2] = 7 # thetam
# x0[2] = 4.2 # thetam
x0[2] = 7.1 # thetam

# show initial objective
print('Initial SSE Objective: ' + str(objective(x0)))

"""
# optimize Km, taum, thetam
# bounds on variables
bnds = ((-1.0e10, 1.0e10), (0.01, 1.0e10), (0.0, 1.0e10))
# bnds = ((0, 4), (1, 1.0e10), (1, 10))
solution = minimize(objective,x0,method='SLSQP',bounds=bnds)
x = solution.x

# show final objective
print('Final SSE Objective: ' + str(objective(x)))

print('Kp: ' + str(x[0]))
print('taup: ' + str(x[1]))
print('thetap: ' + str(x[2]))
"""

# calculate model with updated parameters
ym1 = sim_model(x0)
"""
ym2 = sim_model(x)
"""

x0 = np.zeros(3)
x0[0] = 2.11281958 # Km
x0[1] = 112 # taum
x0[2] = 4.2 # thetam
ym1b = sim_model(x0)

# plot results
plt.figure()

plt.subplot(2,1,1)
plt.plot(t,yp,'kx-',linewidth=2,label='Process Data')
plt.plot(t,ym1,'b-',linewidth=2,label='Initial Guess')
plt.plot(t,ym1b,'g-',linewidth=2,label='Initial Guess2')
# plt.plot(t,ym2,'r--',linewidth=3,label='Optimized FOPDT')
plt.ylabel('Output')
plt.legend(loc='best')

plt.subplot(2,1,2)
plt.plot(t,u,'bx-',linewidth=2)
plt.plot(t,uf(t),'r--',linewidth=3)
plt.legend(['Measured','Interpolated'],loc='best')
plt.ylabel('Input Data')
plt.show()





