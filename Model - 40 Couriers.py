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




















# Gs = {}
# node_pos = {}
# edges = {}

# # formatting 
# StyleList = ['dashed', 'solid']
# ColorList = ['blue', 'red']
# for v in range(len(VehicleList)):
#     temp = res[res['Vehicle ID'] == v]
#     node_pos[v] = [(temp.loc[i,'Visited Point'], {'loc': (temp.loc[i,'x'], temp.loc[i,'y']), 'time': temp.loc[i,'Visiting time']}) for i in temp.index ]
#     edges[v] = [(temp.iloc[i,1],temp.iloc[i+1,1]) for i in range(temp.shape[0]-1)]
#     Gs[v] = nx.Graph()
#     Gs[v].add_nodes_from(node_pos[v])
#     Gs[v].add_edges_from(edges[v])
#     node_positions = nx.get_node_attributes(Gs[v], 'loc')
#     nx.draw(Gs[v], node_positions, with_labels=True, node_color='lightblue') #, font_weight='bold'
#     nx.draw_networkx_edges(Gs[v], node_positions, edgelist=edges[v], edge_color=ColorList[v], width=.5, style = StyleList[v], arrows=True)
#     # labels = {node: f"\n{Gs.[node]}" for node in Gs[v].nodes}


# plt.title("Network Graph with Route and Arrows")
# plt.show()





    