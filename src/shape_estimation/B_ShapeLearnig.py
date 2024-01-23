# -*- coding: utf-8 -*-
"""
Created on Sun Jan 31 15:20:08 2021

@author: fkhad
"""
import pylab as pb
pb.ion()
import scipy.io
import GPy
import numpy as np

mat = scipy.io.loadmat('toGpy_BottleNeck.mat')

X = mat["X"].transpose()
Vn = mat["Xn"].transpose()
Xin = mat["Xin"].transpose()
Yin = mat["Yin"].transpose()
Xout = mat["Xout"].transpose()
Yout = mat["Yout"].transpose()

# data selection and procsess
no_tr_set = 1500
no_tst_set = X.shape[0]- no_tr_set

choice = np.random.choice(range(no_tst_set+no_tr_set), size=(no_tr_set,), replace=False)    
ind = np.zeros(no_tst_set+no_tr_set, dtype=bool)
ind[choice] = True
rest = ~ind

trn_X = X[ind,:]
trn_Vn = Vn[ind,:]
trn_Xin = Xin[ind,:]
trn_Xout = Xout[ind,:]
trn_Yin = Yin[ind,:]
trn_Yout = Yout[ind,:]

tst_X = X[rest,:]
tst_Vn = Vn[rest,:]
tst_Xin = Xin[rest,:]
tst_Xout = Xout[rest,:]
tst_Yin = Yin[rest,:]
tst_Yout = Yout[rest,:]

trn_input = np.vstack((trn_X,trn_Xin,trn_Xout))
trn_output = np.vstack((np.zeros((trn_X.shape[0],1)),trn_Yin,trn_Yout))

#setting up gp
print("setting the gp model :")
# kernel selection
ker1 = GPy.kern.RBF(input_dim= trn_input.shape[1])
#ker1 = GPy.kern.RBF(input_dim= X.shape[1],lengthscale=np.ones(X.shape[1]),ARD=True)
ker2 = GPy.kern.White(input_dim=trn_input.shape[1])
ker3 = GPy.kern.Bias(trn_input.shape[1])

# model creation
kernel = ker1 + ker2 + ker3
m = GPy.models.GPRegression(trn_input, trn_output,kernel=kernel)

print("optimizing the gp model :")
m.optimize(optimizer='bfgs',max_iters=1000)
print (m)
y_mean, std_pred = m.predict(tst_X)
er_y = - y_mean
print ("test prediction MSE is:  " , np.square(er_y).mean())
print ("test prediction Mean Std is: " , std_pred.mean())

header = "x0,x1,x2,y0,std"
np.savetxt('test_of_gpy.csv',np.hstack((tst_X,y_mean,std_pred)), delimiter=',',header=header,comments='')
