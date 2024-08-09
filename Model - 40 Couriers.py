import numpy as np
import pyomo.environ as Pyo
from Functions import *
import pandas as pd
import datetime as dt

import networkx as nx
import matplotlib.pyplot as plt
#%%

if __name__ == '__main__':
    # Build the delivery fleet
    nCouriers = [[1,1],[1,1], [1,1],[1,1],[1,1],[1,1],[1,1],[1,1],[1,1],[1,1]]
    
    # Input the restaurants
    temp = pd.read_csv('R.csv')
    R = {'r'+str(i+1):(temp.loc[i,'Res_x'], temp.loc[i,'Res_y']) for i in temp.index}       
    # Input the fuel stations
    temp = pd.read_csv('F.csv')
    F = {'f'+str(i+1)+str(j):(temp.loc[i,'Fuel_x'], temp.loc[i,'Fuel_y']) for i in temp.index for j in range(2)}  
    
    
    # Form the vehicle list
    ModeParam = {1:{'Cap':6, 'Min_Fuel': 2, 'Max_Fuel': 14, 'Speed': 25/60} ,
                 2:{'Cap':4, 'Min_Fuel': 527472, 'Max_Fuel': 3516480, 'Speed': 45/60} }
    VehicleList = []
    vID = 0
    for i in range(len(R)):
        for j in range(len(nCouriers[i])):
            VehicleList.append( Vehicle(vID, j+1, ModeParam[j+1]['Cap'], ModeParam[j+1]['Min_Fuel'], ModeParam[j+1]['Max_Fuel'], 
                                        ModeParam[j+1]['Speed'], R['r'+str(i+1)], 0, ModeParam[j+1]['Max_Fuel'], [], R['r'+str(i+1)]) )
            vID += 1
    
    
    # Form the event and customer list
    
    temp = pd.read_csv('EventList.csv')
    EventList ={temp.loc[i,'CustomerID']:[temp.loc[i,'RestauranID'], temp.loc[i,'Weight'], temp.loc[i,'Arrival']] for i in temp.index}
    EventList = {'c' + str(i): EventList['c' + str(i)] for i in range(1250)}
    
    temp = pd.read_csv('CustomerList.csv')
    CustomerList = {temp.loc[i,'CustomerID']:[temp.loc[i,'Order_x'], temp.loc[i,'Order_y']] for i in temp.index}
    
    # Simulate the system  
    file = open('Unassigned.txt','w+')
    file.close()
    res = Simulate(EventList, CustomerList, VehicleList, R, F, nWorker = 10, ObjTyoe = 2)    
    
    ###--------------------------------------------------------------------------
    res = pd.DataFrame(res, columns = ['Vehicle ID', 'Visited Point', 'x', 'y', 'Visiting time', 'Load', 'Fuel'])
    now = dt.datetime.now().strftime(('%Y-%m-%d %H-%M-%S'))
    res.to_csv('Results'+now+'.csv')

