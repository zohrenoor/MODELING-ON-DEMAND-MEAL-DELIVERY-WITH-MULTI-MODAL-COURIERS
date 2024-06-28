import numpy as np
import pyomo.environ as Pyo
import math
import pandas as pd

import multiprocessing as mp
import queue 
import time


class Vehicle:
    def __init__(self, ID, Mode, MaxL, MinF, MaxF, V, Loc, L, F, AssignedOrders,EndLoc):
        self.ID = ID
        self.Mode = Mode
        self.MaxL = MaxL
        self.MaxF = MaxF
        self.MinF = MinF
        self.Loc = Loc
        self.L = L
        self.F = F
        self.V = V
        self.Route = []#(x,y, node, L, F)
        self.AssignedOrders = AssignedOrders
        self.e = EndLoc
    def __repr__(self):
        return f"<ID: {self.ID}, Mode: {self.Mode}, Loc: {self.Loc}, AssignedO:{self.AssignedOrders}>"
    
    def Update(self, time, Clock = 0, Recorder = [], Arrival = ''):
        if Clock == 0:
            Recorder.append([self.ID, 's', self.Loc[0], self.Loc[1],  Clock, self.L, self.F ])
            return
        
        n = len(self.Route)
        if n>0:
            while True:
                if len(self.Route) < 2:
                    del(self.Route[0])
                    break 
                i=0
                dist = EuclideanDist(self.Route[i][0:2], self.Route[i+1][0:2])
                t = dist/self.V

                if t<time:
                    time -= t
                    ind = self.Route[i+1][2].find('c')
                    if ind == 0:
                        for ii in range(len(self.AssignedOrders)):
                            if self.AssignedOrders[ii][1] == self.Route[i+1][2] :
                                del(self.AssignedOrders[ii]) 
                                break
                    elif ind>0:
                        for ii in range(len(self.AssignedOrders)):
                            if self.AssignedOrders[ii][0] == self.Route[i+1][2][:ind] and self.AssignedOrders[ii][1] == self.Route[i+1][2][ind:]:
                                temp = list(self.AssignedOrders[ii])
                                temp[0] = ''
                                self.AssignedOrders[ii] = tuple(temp)
                                break
                    self.L = self.Route[i+1][3]
                    self.F = self.Route[i+1][4]
                    
                    Clock += t
                    Recorder.append([self.ID, self.Route[i+1][2], self.Route[i+1][0], self.Route[i+1][1],  Clock, self.L, self.F ])
                    
                    del(self.Route[0])
                else:
                    
                        
                    d = time * self.V 
                    
                    x = self.Route[i][0] + d/dist*(self.Route[i+1][0]-self.Route[i][0])
                    y = self.Route[i][1] + d/dist*(self.Route[i+1][1]-self.Route[i][1])
                    self.Loc = (x,y)
                    ALPHA,BETA = EnergyConsum(self.Mode)
                    self.F -= ALPHA*d+BETA*d*self.L
                    

                    self.Route[0] = (x,y,'s',self.L, self.F)
                    
                    
                    Clock += time
                    Recorder.append([self.ID, 's-'+Arrival, x, y,  Clock, self.L, self.F ])
                    break
                
def Eval(Input_q, Output_q, Lock, Model = 1):
    while True:
        Lock.acquire()
        if not Input_q.empty():
            Args = Input_q.get()
            Lock.release()
            if len(Args) == 0:
                break
            
            vID = Args[0]
            Args = Args[1]
            if Model == 1:
                # Find a dual value
                dual, _ = SolveModel1Dual(Args[0], Args[1], Args[3], Args[4], Args[5], Args[7], Args[8], Args[9], Args[10],
                                      mode = Args[12])
                if dual < 0:
                   Output_q.put((vID, dual, [])) 
                   continue
                # Solve the actual problem
                                         #   R,        C,       F,       O,       s,      e,      F0,     Fmin,   Fmax,       L0,    Lmax, 
                obj, route = SolveModel1(Args[0], Args[1], Args[2], Args[3], Args[4], Args[5], Args[6], Args[7], Args[8], Args[9], Args[10],
                                      mode = Args[12], Dual = dual) 
                                    # mode = 1,           Dual = 0
                Output_q.put((vID, obj, route))
            elif Model == 2:
                # Find a dual value
                dual, _ = SolveModel2Dual(Args[0], Args[1], Args[3], Args[4], Args[5], Args[7], Args[8], Args[9], Args[10],
                                      Args[11], mode = Args[12], timer = Args[13])
                if dual < 0:
                   Output_q.put((vID, dual, [])) 
                   continue
                # Solve the actual problem
                                         #R,       C,      F,         O,      s,        e,      F0,     Fmin,    Fmax,    L0,      Lmax,
                obj, route = SolveModel2(Args[0], Args[1], Args[2], Args[3], Args[4], Args[5], Args[6], Args[7], Args[8], Args[9], Args[10],
                                      Args[11], Args[12], timer = Args[13], Dual = dual) 
                                        # v,    mode = 1, timer = 0, Dual = 0
                Output_q.put((vID, obj, route))
        else:
            Lock.release()
            
        time.sleep(1)
    
            
            
            
def Simulate(EventList, C, VehicleList, R, F, nWorker = 3, ObjTyoe = 1):
    
    X_q = mp.Manager().Queue()
    Y_q = mp.Manager().Queue()
    lock = mp.Lock()
    
    WorkerPool = []
    for i in range(nWorker):
        WorkerPool.append(mp.Process(target = Eval, args = (X_q, Y_q, lock, ObjTyoe )) )
        WorkerPool[i].start()
        
                          
    
    CustomerCounter = 0
    Timer = 0
    res = [] # [VehicleID, Node, Coords, Timer, Load, Fuel]
    EvenIDs = list(EventList.keys())
        
    for it in range(len(EvenIDs)):
        print( 'Customer {} is being planned.'.format(str(it)) )
        
        c = EvenIDs[it]
        t = EventList[c][2] 
        r = EventList[c][0] 
        l = EventList[c][1] 
               
        CustomerCounter += 1
        
        # Update the vehicles status
        for v in VehicleList:
            v.Update(t, Clock = Timer, Recorder = res, Arrival = c)
        
        Timer += t
        
        # Pick the arrived order
        NewO = (r, c, l, Timer )
    
        
        for v in range(len(VehicleList)):
            O = VehicleList[v].AssignedOrders + [NewO]
            CC = {o[1]:C[o[1]] for o in O}
            vv = VehicleList[v]
            # Find a dual value
            args = (VehicleList[v].ID,[R, CC, F, O, vv.Loc, vv.e, vv.F, vv.MinF, vv.MaxF, vv.L, vv.MaxL, vv.V, vv.Mode, Timer])
            X_q.put(args) 
            
        while Y_q.qsize() != len(VehicleList):
            time.sleep(1)
        
        # Find the best vehicle:
        temp = {}
        MinObj = np.inf
        MinID = -1
        MinRoute = []
        while not Y_q.empty():
            vID, obj, route = Y_q.get()
            # print(vID, obj)
            if obj > 0 and obj < MinObj:
                MinID = vID
                MinObj = obj
                MinRoute = route
        if MinID == -1:
            print('No vehicle had a feasible route')
            file = open('Unassigned.txt','a')
            file.write(str(it)+', '+str(c)+', '+ str(r)+'\n')
            file.close()
            continue
        # Update the route of selected vehicle
        VehicleList[MinID].Route = MinRoute
        VehicleList[MinID].AssignedOrders.append(NewO)
        # print(c,'->', MinID)
        # print(MinRoute)
        # print(MinObj)
        # print(VehicleList[MinID].Route)
   
    # End the processes
    for i in range(nWorker):
        X_q.put([])
    for i in range(nWorker):
        WorkerPool[i].join()
        
    # Kill the queues
    # print(X_q.get())
    # X_q.join()
    # Y_q.join()      
    
    for v in VehicleList:
        v.Update(np.Inf, Clock = Timer, Recorder = res)    

    return res

def SolveModel1(R, C, F, O, s, e, F0, Fmin, Fmax, L0, Lmax, mode = 1, Dual = 0):
    RR = [o[0]+o[1] for o in O if not o[0] == '']
    FF = list(F.keys())
    N = RR + list(C.keys()) + FF + ['s', 'e']
    NN = MergeDic([{rr: R[rr[0:rr.find('c')]] for rr in RR} , C , F , {'s': s, 'e': e} ])
    Links = [(i,j) for i in N for j in N if i!=j]
    # Decistion Variables
    MODEL = Pyo.ConcreteModel()
    
    MODEL.Xij = Pyo.Var(Links, domain= Pyo.Binary, name = 'X')
    MODEL.Fi = Pyo.Var(N, domain = Pyo.NonNegativeReals, name = 'F')
    MODEL.Li = Pyo.Var(N, domain = Pyo.NonNegativeReals, name = 'L')
    MODEL.n = Pyo.Var(N, domain = Pyo.NonNegativeReals, name = 'n')
    ## Linearization variables
    MODEL.FXij = Pyo.Var(Links, domain = Pyo.NonNegativeReals, name = 'FX')
    MODEL.LXij = Pyo.Var(Links, domain = Pyo.NonNegativeReals, name = 'LX')


    # Objective: Minimizing the distance
    MODEL.Obj = Pyo.Objective(expr = Pyo.quicksum( EuclideanDist(NN[i], NN[j])*MODEL.Xij[i,j] for i in N for j in N if i!=j), name='Obj')
    MODEL.Cons0 = Pyo.ConstraintList(name = 'Cons'+str(0))
    MODEL.Cons0.add(Pyo.quicksum( EuclideanDist(NN[i], NN[j])*MODEL.Xij[i,j] for i in N for j in N if i!=j) >= Dual)
    
    
    # S.T.
    ConsCounter = 0
    ConsCounter += 1
    ## Cons1: Enter only once
    MODEL.Cons1 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))
    for j in np.setdiff1d(N, FF+['s'] ):
        MODEL.Cons1.add(expr = Pyo.quicksum(MODEL.Xij[i,j] for i in N if i!=j) == 1)

    ConsCounter += 1
    ## Cons2: Exit only once
    MODEL.Cons2 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))
    for i in np.setdiff1d(N, FF+['e']):
        MODEL.Cons1.add(expr = Pyo.quicksum(MODEL.Xij[i,j] for j in N if i!=j) == 1)
        
    
    ConsCounter += 1
    ## Cons3: Stay at end
    MODEL.Cons3 = Pyo.Constraint(expr = Pyo.quicksum(MODEL.Xij['e',j] for j in N if j !='e') == 0, name = 'Cons'+str(ConsCounter))
    
    ConsCounter += 1
    ## Cons4: Exit from the node that is entered
    MODEL.Cons4 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))
    for j in F:
        MODEL.Cons4.add(expr = Pyo.quicksum(MODEL.Xij[i,j] for i in N if i!=j) ==
                        Pyo.quicksum(MODEL.Xij[j,i] for i in N if i!=j))
        
   
    ConsCounter += 1
    ## Cons5: Fuel calculation from i to j
    MODEL.Cons5 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))
    for j in np.setdiff1d(N, list(F.keys()) + ['s']):
        ALPHA, BETA = EnergyConsum(mode)
        MODEL.Cons5.add(expr = MODEL.Fi[j] == 
                        Pyo.quicksum( MODEL.FXij[i,j] - (ALPHA*EuclideanDist(NN[i], NN[j])*MODEL.Xij[i,j] + BETA* EuclideanDist(NN[i], NN[j])*MODEL.LXij[i,j]) for i in N if i!=j))
    
    ConsCounter += 1
    ## Cons6: Fuel at certain points (F, s and e)
    MODEL.Cons6 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))
    for i in F:
        MODEL.Cons6.add(expr = MODEL.Fi[i] == Fmax)
    MODEL.Cons6.add(expr = MODEL.Fi['s'] == F0)  
    MODEL.Cons6.add(expr = MODEL.Fi['e'] >= Fmin) 
    
    
    ConsCounter += 1
    ## Cons7: fuel must be enough to go from i to F
    MODEL.Cons7 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))
    for i in N:
        ALPHA, BETA = EnergyConsum(mode)
        MODEL.Cons7.add(expr = MODEL.Fi[i] >= Pyo.quicksum(ALPHA*EuclideanDist(NN[i], NN[j])*MODEL.Xij[i,j] + BETA*EuclideanDist(NN[i], NN[j])*MODEL.LXij[i,j]  for j in F if i!=j))

    ConsCounter += 1
    ## Cons8: Load calculation from i to j
    MODEL.Cons8 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))
    for o in O:
        if not o[0] == '':
            j = o[0]+o[1]
            MODEL.Cons8.add(expr = MODEL.Li[j] == 
                            Pyo.quicksum( ( MODEL.LXij[i,j] + o[2]*MODEL.Xij[i,j] ) for i in N if i!=j))
        j = o[1]
        MODEL.Cons8.add(expr = MODEL.Li[j] == 
                        Pyo.quicksum( ( MODEL.LXij[i,j] - o[2]*MODEL.Xij[i,j] ) for i in N if i!=j))
    
    for j in F:
        MODEL.Cons8.add(expr = MODEL.Li[j] == Pyo.quicksum( MODEL.LXij[i,j]  for i in N if i!=j) )
    MODEL.Cons8.add(expr = MODEL.Li['s'] == L0)
    
    ConsCounter += 1
    ## Cons9: Maximum Load 
    MODEL.Cons9 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))
    for i in N:
        MODEL.Cons9.add(expr = MODEL.Li[i] <= Lmax)
        
    ConsCounter += 1
    ## Cons12: Subtour elimination 
    MODEL.Cons12 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))
    
    MODEL.Cons12.add(expr = MODEL.n['s'] == 0)   
    for i in N:
        for j in N:
            if i != j:
                MODEL.Cons12.add(expr = MODEL.n[j] >= MODEL.n[i] + 1 - len(N) *(1-MODEL.Xij[i,j]) ) 
    
    ConsCounter += 1
    ## Cons13: Visit Restaurants before customers
    ###### This constriant and subtour elimination can be removed when we have Ti in the model
    MODEL.Cons13 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))  
    for o in O:
        if not o[0] == '':
            i = o[0]+o[1]
            j = o[1]
            MODEL.Cons13.add(expr = MODEL.n[i] <= MODEL.n[j]) 
                          
        
    # Linearization Constraints
    ConsCounter += 1
    ## Cons10: FX Linearization 
    MODEL.Cons10 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))
    for i in N:
        for j in N:
            if i==j:
                continue
            MODEL.Cons10.add(expr = MODEL.FXij[i,j] <= Fmax * MODEL.Xij[i,j])
            MODEL.Cons10.add(expr = MODEL.FXij[i,j] <= MODEL.Fi[i])
            MODEL.Cons10.add(expr = MODEL.FXij[i,j] >= MODEL.Fi[i] - Fmax*(1 - MODEL.Xij[i,j]))

    ConsCounter += 1
    ## Cons11: LX Linearization 
    MODEL.Cons11 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))
    for i in N:
        for j in N:
            if i==j:
                continue
            MODEL.Cons11.add(expr = MODEL.LXij[i,j] <= Lmax * MODEL.Xij[i,j])
            MODEL.Cons11.add(expr = MODEL.LXij[i,j] <= MODEL.Li[i])
            MODEL.Cons11.add(expr = MODEL.LXij[i,j] >= MODEL.Li[i] - Lmax*(1 - MODEL.Xij[i,j]))
            
    solver = Pyo.SolverFactory('cplex', executable = '/opt/ibm/ILOG/CPLEX_Studio2211/cplex/bin/x86-64_linux/cplex')
    # solver = Pyo.SolverFactory('cplex',executable = '/home/m/mahmoodian/Softwares/ibm/ILOG/CPLEX_Studio1210/cplex/bin/x86-64_linux/cplex')
    # solver = Pyo.SolverFactory('gurobi')
    
    solver.options['timelimit'] = 120  # Set time limit in seconds
    # solver.options['absmipgap'] = 1e-4  # Set absolute gap
    solver.options['mipgap'] = 0.1 
    
    
    
    
    Res = solver.solve(MODEL,tee=False)
    
    if Res.solver.termination_condition in  ['infeasible', 'unknown']:
        return -1,[]
    
    Orig = 's'
    Route = [(s[0],s[1], 's', L0, F0)]
    
    
    
    # print('---------------------------------------------------------')
    # for i in MODEL.Xij.keys():
    #     if MODEL.Xij[i]()>0.5:
    #         print(i, MODEL.Xij[i](), MODEL.Fi[i[0]](), MODEL.Li[i[0]]())

    # print('--------------------------------------------',Res.solver.termination_condition)
    
    while True:
        for k in N:
            try:
                if k!=Orig and MODEL.Xij[Orig, k]()>0.5:
                    Orig = k
                    # print(Orig)
                    Route.append((NN[k][0], NN[k][1], k, MODEL.Li[k](), MODEL.Fi[k]()))
                    break
            except:
                print(Res.solver.termination_condition)
                raise Exception("The Model was not solved and the condition is not listed.")
                    
        if Orig == 'e':
            break
        
    return MODEL.Obj(), Route 

def SolveModel2(R, C, F, O, s, e, F0, Fmin, Fmax, L0, Lmax, v, mode = 1, timer = 0, Dual = 0):
    # This model needs the speed and time's zero point
    RR = [o[0]+o[1] for o in O if not o[0] == '']
    FF = list(F.keys())
    N = RR + list(C.keys()) + FF + ['s', 'e']
    NN = MergeDic([{rr: R[rr[0:rr.find('c')]] for rr in RR} , C , F , {'s': s, 'e': e}])
    Links = [(i,j) for i in N for j in N if i!=j]
    # Decistion Variables
    MODEL = Pyo.ConcreteModel()
    
    MODEL.Z = Pyo.Var(domain = Pyo.NonNegativeReals, name = 'Z')
    MODEL.Xij = Pyo.Var(Links, domain= Pyo.Binary, name = 'X')
    MODEL.Fi = Pyo.Var(N, domain = Pyo.NonNegativeReals, name = 'F')
    MODEL.Li = Pyo.Var(N, domain = Pyo.NonNegativeReals, name = 'L')
    MODEL.n = Pyo.Var(N, domain = Pyo.NonNegativeReals, name = 'n')
    MODEL.Ti = Pyo.Var(N, domain = Pyo.NonNegativeReals, name = 'T')
    ## Linearization variables
    MODEL.FXij = Pyo.Var(Links, domain = Pyo.NonNegativeReals, name = 'FX')
    MODEL.LXij = Pyo.Var(Links, domain = Pyo.NonNegativeReals, name = 'LX')
    MODEL.TXij = Pyo.Var(Links, domain = Pyo.NonNegativeReals, name = 'TX')


    # Objective: Minimizing the distance
    # MODEL.Obj = Pyo.Objective(expr = Pyo.quicksum( EuclideanDist(NN[i], NN[j])*MODEL.Xij[i,j] for i in N for j in N if i!=j), name='Obj')
    
    MODEL.Obj = Pyo.Objective(expr =  MODEL.Z + .00001 * MODEL.Ti['e'], name='Obj') #MODEL.Z + 1/(100+timer) *
    MODEL.Cons0 = Pyo.ConstraintList(name = 'Cons'+str(0))
    MODEL.Cons0.add( MODEL.Z >= Dual)
    
    # S.T.
    ConsCounter = 0
    ConsCounter += 1
    ## Cons1: Enter only once
    MODEL.Cons1 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))
    for j in np.setdiff1d(N, FF+['s'] ):
        MODEL.Cons1.add(expr = Pyo.quicksum(MODEL.Xij[i,j] for i in N if i!=j) == 1)

    ConsCounter += 1
    ## Cons2: Exit only once
    MODEL.Cons2 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))
    for i in np.setdiff1d(N, FF+['e']):
        MODEL.Cons1.add(expr = Pyo.quicksum(MODEL.Xij[i,j] for j in N if i!=j) == 1)
        
    
    ConsCounter += 1
    ## Cons3: Stay at end
    MODEL.Cons3 = Pyo.Constraint(expr = Pyo.quicksum(MODEL.Xij['e',j] for j in N if j !='e') == 0, name = 'Cons'+str(ConsCounter))
    
    ConsCounter += 1
    ## Cons4: Exit from the node that is entered
    MODEL.Cons4 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))
    for j in F:
        MODEL.Cons4.add(expr = Pyo.quicksum(MODEL.Xij[i,j] for i in N if i!=j) ==
                        Pyo.quicksum(MODEL.Xij[j,i] for i in N if i!=j))
        
   
    ConsCounter += 1
    ## Cons5: Fuel calculation from i to j
    MODEL.Cons5 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))
    for j in np.setdiff1d(N, list(F.keys()) + ['s']):
        ALPHA, BETA = EnergyConsum(mode)
        MODEL.Cons5.add(expr = MODEL.Fi[j] == 
                        Pyo.quicksum( MODEL.FXij[i,j] - (ALPHA*EuclideanDist(NN[i], NN[j])*MODEL.Xij[i,j] + BETA* EuclideanDist(NN[i], NN[j])*MODEL.LXij[i,j]) for i in N if i!=j))
    
    ConsCounter += 1
    ## Cons6: Fuel at certain points (F, s and e)
    MODEL.Cons6 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))
    for i in F:
        MODEL.Cons6.add(expr = MODEL.Fi[i] == Fmax)
    MODEL.Cons6.add(expr = MODEL.Fi['s'] == F0)  
    MODEL.Cons6.add(expr = MODEL.Fi['e'] >= Fmin) 
    
    
    ConsCounter += 1
    ## Cons7: fuel must be enough to go from i to F
    MODEL.Cons7 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))
    for i in N:
        ALPHA, BETA = EnergyConsum(mode)
        MODEL.Cons7.add(expr = MODEL.Fi[i] >= Pyo.quicksum(ALPHA*EuclideanDist(NN[i], NN[j])*MODEL.Xij[i,j] + BETA*EuclideanDist(NN[i], NN[j])*MODEL.LXij[i,j]  for j in F if i!=j))

    ConsCounter += 1
    ## Cons8: Load calculation from i to j
    MODEL.Cons8 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))
    for o in O:
        if not o[0] == '':
            j = o[0]+o[1]
            MODEL.Cons8.add(expr = MODEL.Li[j] == 
                            Pyo.quicksum( ( MODEL.LXij[i,j] + o[2]*MODEL.Xij[i,j] ) for i in N if i!=j))
        j = o[1]
        MODEL.Cons8.add(expr = MODEL.Li[j] == 
                        Pyo.quicksum( ( MODEL.LXij[i,j] - o[2]*MODEL.Xij[i,j] ) for i in N if i!=j))
    
    for j in F:
        MODEL.Cons8.add(expr = MODEL.Li[j] == Pyo.quicksum( MODEL.LXij[i,j]  for i in N if i!=j) )
    MODEL.Cons8.add(expr = MODEL.Li['s'] == L0)
    
    ConsCounter += 1
    ## Cons9: Maximum Load 
    MODEL.Cons9 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))
    for i in N:
        MODEL.Cons9.add(expr = MODEL.Li[i] <= Lmax)
    
    # Linearization Constraints
    ConsCounter += 1
    ## Cons10: FX Linearization 
    MODEL.Cons10 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))
    for i in N:
        for j in N:
            if i==j:
                continue
            MODEL.Cons10.add(expr = MODEL.FXij[i,j] <= Fmax * MODEL.Xij[i,j])
            MODEL.Cons10.add(expr = MODEL.FXij[i,j] <= MODEL.Fi[i])
            MODEL.Cons10.add(expr = MODEL.FXij[i,j] >= MODEL.Fi[i] - Fmax*(1 - MODEL.Xij[i,j]))

    ConsCounter += 1
    ## Cons11: LX Linearization 
    MODEL.Cons11 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))
    for i in N:
        for j in N:
            if i==j:
                continue
            MODEL.Cons11.add(expr = MODEL.LXij[i,j] <= Lmax * MODEL.Xij[i,j])
            MODEL.Cons11.add(expr = MODEL.LXij[i,j] <= MODEL.Li[i])
            MODEL.Cons11.add(expr = MODEL.LXij[i,j] >= MODEL.Li[i] - Lmax*(1 - MODEL.Xij[i,j]))
    
    
    # ConsCounter += 1
    ## Cons12: Subtour elimination 
    MODEL.Cons12 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))
    
    MODEL.Cons12.add(expr = MODEL.n['s'] == 0)   
    for i in N:
        for j in N:
            if i != j:
                MODEL.Cons12.add(expr = MODEL.n[j] >= MODEL.n[i] + 1 - len(N) *(1-MODEL.Xij[i,j]) ) 
    
    ConsCounter += 1
    ## Cons13: Visit Restaurants before customers
    ###### This constriant and subtour elimination can be removed when we have Ti in the model
    MODEL.Cons13 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))  
    for o in O:
        if not o[0] == '':
            i = o[0]+o[1]
            j = o[1]
            MODEL.Cons13.add(expr = MODEL.Ti[i] <= MODEL.Ti[j]) 
                          
    ConsCounter += 1
    ## Cons14: Time calculation
    MODEL.Cons14 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))
    for j in N:
        MODEL.Cons14.add(expr = MODEL.Ti[j] == Pyo.quicksum(MODEL.TXij[i,j] + EuclideanDist(NN[i], NN[j])/v*MODEL.Xij[i,j]  for i in N if i!=j))
    MODEL.Cons14.add(expr = MODEL.Ti['s'] == timer)

    # ConsCounter += 1
    ## Cons15: TX Linearization 
    MODEL.Cons15 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))
    Tmax = timer
    for i in N:
        for j in N:
            Tmax += EuclideanDist(NN[i], NN[j])

    for i in N:
        for j in N:
            if j=='s' or i==j:
                continue
            MODEL.Cons15.add(expr = MODEL.TXij[i,j] <= Tmax * MODEL.Xij[i,j])
            MODEL.Cons15.add(expr = MODEL.TXij[i,j] <= MODEL.Ti[i])
            MODEL.Cons15.add(expr = MODEL.TXij[i,j] >= MODEL.Ti[i] - Tmax*(1 - MODEL.Xij[i,j]))
    
    ConsCounter += 1
    ## Cons16: MinMax Linearization Constraints
    MODEL.Cons16 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))
    for o in O:
        MODEL.Cons16.add(expr = MODEL.Z >= MODEL.Ti[o[1]] - o[3])
            
    # solver = Pyo.SolverFactory('gurobi')
    solver = Pyo.SolverFactory('cplex',executable = '/opt/ibm/ILOG/CPLEX_Studio2211/cplex/bin/x86-64_linux/cplex')
    # solver = Pyo.SolverFactory('cplex',executable = '/home/m/mahmoodian/Softwares/ibm/ILOG/CPLEX_Studio1210/cplex/bin/x86-64_linux/cplex')
    
    
    solver.options['timelimit'] = 120  # Set time limit in seconds
    # solver.options['absmipgap'] = 1e-4  # Set absolute gap
    solver.options['mipgap'] = 0.1 
    
    Res = solver.solve(MODEL,tee=False)
    
    
    
    if Res.solver.termination_condition in  ['infeasible', 'unknown']:
        return -1,[]
    
    
    Orig = 's'
    Route = [(s[0],s[1], 's', L0, F0)]
    
    
    
    # print('---------------------------------------------------------')
    # print('Timer = ', timer)
    # for i in MODEL.Xij.keys():
    #     if MODEL.Xij[i]()>0.5:
    #         print(i, MODEL.Xij[i](), MODEL.Fi[i[0]](), MODEL.Li[i[0]](), MODEL.Ti[i[0]]())
    # print(Res.solver.termination_condition)
    
    
    while True:
        for k in N:
            try:
                if k!=Orig and MODEL.Xij[Orig, k]()>0.5:
                    Orig = k
                    # print(Orig)
                    Route.append((NN[k][0], NN[k][1], k, MODEL.Li[k](), MODEL.Fi[k]()))
                    break
            except:
                print(Res.solver.termination_condition)
                raise Exception("The Model was not solved and the condition is not listed.")
        if Orig == 'e':
            break
        
    return MODEL.Obj(), Route        
                
def SolveModel1Dual(R, C, O, s, e, Fmin, Fmax, L0, Lmax, mode = 1):
    RR = [o[0]+o[1] for o in O if not o[0] == '']
    N = RR + list(C.keys()) + ['s', 'e']
    NN = MergeDic([{rr: R[rr[0:rr.find('c')]] for rr in RR} , C , {'s': s, 'e': e}])
    Links = [(i,j) for i in N for j in N if i!=j]
    # Decistion Variables
    MODEL = Pyo.ConcreteModel()
    
    MODEL.Xij = Pyo.Var(Links, domain= Pyo.Binary, name = 'X')
    MODEL.Li = Pyo.Var(N, domain = Pyo.NonNegativeReals, name = 'L')
    MODEL.n = Pyo.Var(N, domain = Pyo.NonNegativeReals, name = 'n')
    ## Linearization variables
    MODEL.LXij = Pyo.Var(Links, domain = Pyo.NonNegativeReals, name = 'LX')


    # Objective: Minimizing the distance
    MODEL.Obj = Pyo.Objective(expr = Pyo.quicksum( EuclideanDist(NN[i], NN[j])*MODEL.Xij[i,j] for i in N for j in N if i!=j), name='Obj')
    
    
    # S.T.
    ConsCounter = 0
    ConsCounter += 1
    ## Cons1: Enter only once
    MODEL.Cons1 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))
    for j in np.setdiff1d(N, ['s'] ):
        MODEL.Cons1.add(expr = Pyo.quicksum(MODEL.Xij[i,j] for i in N if i!=j) == 1)

    ConsCounter += 1
    ## Cons2: Exit only once
    MODEL.Cons2 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))
    for i in np.setdiff1d(N, ['e']):
        MODEL.Cons1.add(expr = Pyo.quicksum(MODEL.Xij[i,j] for j in N if i!=j) == 1)
        
    
    ConsCounter += 1
    ## Cons3: Stay at end
    MODEL.Cons3 = Pyo.Constraint(expr = Pyo.quicksum(MODEL.Xij['e',j] for j in N if j !='e') == 0, name = 'Cons'+str(ConsCounter))
    

    ConsCounter += 1
    ## Cons8: Load calculation from i to j
    MODEL.Cons8 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))
    for o in O:
        if not o[0] == '':
            j = o[0]+o[1]
            MODEL.Cons8.add(expr = MODEL.Li[j] == 
                            Pyo.quicksum( ( MODEL.LXij[i,j] + o[2]*MODEL.Xij[i,j] ) for i in N if i!=j))
        j = o[1]
        MODEL.Cons8.add(expr = MODEL.Li[j] == 
                        Pyo.quicksum( ( MODEL.LXij[i,j] - o[2]*MODEL.Xij[i,j] ) for i in N if i!=j))
    
    MODEL.Cons8.add(expr = MODEL.Li['s'] == L0)
    
    ConsCounter += 1
    ## Cons9: Maximum Load 
    MODEL.Cons9 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))
    for i in N:
        MODEL.Cons9.add(expr = MODEL.Li[i] <= Lmax)
        
    ConsCounter += 1
    ## Cons12: Subtour elimination 
    MODEL.Cons12 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))
    
    MODEL.Cons12.add(expr = MODEL.n['s'] == 0)   
    for i in N:
        for j in N:
            if i != j:
                MODEL.Cons12.add(expr = MODEL.n[j] >= MODEL.n[i] + 1 - len(N) *(1-MODEL.Xij[i,j]) ) 
    
    ConsCounter += 1
    ## Cons13: Visit Restaurants before customers
    ###### This constriant and subtour elimination can be removed when we have Ti in the model
    MODEL.Cons13 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))  
    for o in O:
        if not o[0] == '':
            i = o[0]+o[1]
            j = o[1]
            MODEL.Cons13.add(expr = MODEL.n[i] <= MODEL.n[j]) 
                          

    ConsCounter += 1
    ## Cons11: LX Linearization 
    MODEL.Cons11 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))
    for i in N:
        for j in N:
            if i==j:
                continue
            MODEL.Cons11.add(expr = MODEL.LXij[i,j] <= Lmax * MODEL.Xij[i,j])
            MODEL.Cons11.add(expr = MODEL.LXij[i,j] <= MODEL.Li[i])
            MODEL.Cons11.add(expr = MODEL.LXij[i,j] >= MODEL.Li[i] - Lmax*(1 - MODEL.Xij[i,j]))
            
    solver = Pyo.SolverFactory('cplex', executable = '/opt/ibm/ILOG/CPLEX_Studio2211/cplex/bin/x86-64_linux/cplex')
    # solver = Pyo.SolverFactory('cplex', executable = '/home/m/mahmoodian/Softwares/ibm/ILOG/CPLEX_Studio1210/cplex/bin/x86-64_linux/cplex')
    # solver = Pyo.SolverFactory('gurobi')
    
    solver.options['timelimit'] = 120  # Set time limit in seconds
    # solver.options['absmipgap'] = 1e-4  # Set absolute gap
    solver.options['mipgap'] = 0.1 
    
    
    
    
    Res = solver.solve(MODEL,tee=False)
    
    if Res.solver.termination_condition in  ['infeasible', 'unknown']:
        return -1,[]
    
    Orig = 's'
    Route = [(s[0],s[1], 's', L0, 0)]
    
    
    
    # print('---------------------------------------------------------')
    # for i in MODEL.Xij.keys():
    #     if MODEL.Xij[i]()>0.5:
    #         print(i, MODEL.Xij[i](), MODEL.Fi[i[0]](), MODEL.Li[i[0]]())

    
    
    while True:
        for k in N:
            try:
                if k!=Orig and MODEL.Xij[Orig, k]()>0.5:
                    Orig = k
                    # print(Orig)
                    Route.append((NN[k][0], NN[k][1], k, MODEL.Li[k](), 0))
                    break
            except:
                print(Res.solver.termination_condition)
                raise Exception("The Model was not solved and the condition is not listed.")
        if Orig == 'e':
            break
        
    return MODEL.Obj(), Route 

def SolveModel2Dual(R, C, O, s, e, Fmin, Fmax, L0, Lmax, v, mode = 1, timer = 0):
    # This model needs the speed and time's zero point
    RR = [o[0]+o[1] for o in O if not o[0] == '']

    N = RR + list(C.keys()) + ['s', 'e']
    NN = MergeDic([{rr: R[rr[0:rr.find('c')]] for rr in RR} , C , {'s': s, 'e': e}])
    Links = [(i,j) for i in N for j in N if i!=j]
    # Decistion Variables
    MODEL = Pyo.ConcreteModel()
    
    MODEL.Z = Pyo.Var(domain = Pyo.NonNegativeReals, name = 'Z')
    MODEL.Xij = Pyo.Var(Links, domain= Pyo.Binary, name = 'X')
    MODEL.Li = Pyo.Var(N, domain = Pyo.NonNegativeReals, name = 'L')
    MODEL.n = Pyo.Var(N, domain = Pyo.NonNegativeReals, name = 'n')
    MODEL.Ti = Pyo.Var(N, domain = Pyo.NonNegativeReals, name = 'T')
    ## Linearization variables
    MODEL.LXij = Pyo.Var(Links, domain = Pyo.NonNegativeReals, name = 'LX')
    MODEL.TXij = Pyo.Var(Links, domain = Pyo.NonNegativeReals, name = 'TX')


    # Objective: Minimizing the distance
    # MODEL.Obj = Pyo.Objective(expr = Pyo.quicksum( EuclideanDist(NN[i], NN[j])*MODEL.Xij[i,j] for i in N for j in N if i!=j), name='Obj')
    
    MODEL.Obj = Pyo.Objective(expr = MODEL.Z + .00001 * MODEL.Ti['e'], name='Obj')

    
    # S.T.
    ConsCounter = 0
    ConsCounter += 1
    ## Cons1: Enter only once
    MODEL.Cons1 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))
    for j in np.setdiff1d(N, ['s'] ):
        MODEL.Cons1.add(expr = Pyo.quicksum(MODEL.Xij[i,j] for i in N if i!=j) == 1)

    ConsCounter += 1
    ## Cons2: Exit only once
    MODEL.Cons2 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))
    for i in np.setdiff1d(N, ['e']):
        MODEL.Cons1.add(expr = Pyo.quicksum(MODEL.Xij[i,j] for j in N if i!=j) == 1)
        
    
    ConsCounter += 1
    ## Cons3: Stay at end
    MODEL.Cons3 = Pyo.Constraint(expr = Pyo.quicksum(MODEL.Xij['e',j] for j in N if j !='e') == 0, name = 'Cons'+str(ConsCounter))
    
   

    ConsCounter += 1
    ## Cons8: Load calculation from i to j
    MODEL.Cons8 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))
    for o in O:
        if not o[0] == '':
            j = o[0]+o[1]
            MODEL.Cons8.add(expr = MODEL.Li[j] == 
                            Pyo.quicksum( ( MODEL.LXij[i,j] + o[2]*MODEL.Xij[i,j] ) for i in N if i!=j))
        j = o[1]
        MODEL.Cons8.add(expr = MODEL.Li[j] == 
                        Pyo.quicksum( ( MODEL.LXij[i,j] - o[2]*MODEL.Xij[i,j] ) for i in N if i!=j))
    

    MODEL.Cons8.add(expr = MODEL.Li['s'] == L0)
    
    ConsCounter += 1
    ## Cons9: Maximum Load 
    MODEL.Cons9 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))
    for i in N:
        MODEL.Cons9.add(expr = MODEL.Li[i] <= Lmax)
    
    # Linearization Constraints

    ConsCounter += 1
    ## Cons11: LX Linearization 
    MODEL.Cons11 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))
    for i in N:
        for j in N:
            if i==j:
                continue
            MODEL.Cons11.add(expr = MODEL.LXij[i,j] <= Lmax * MODEL.Xij[i,j])
            MODEL.Cons11.add(expr = MODEL.LXij[i,j] <= MODEL.Li[i])
            MODEL.Cons11.add(expr = MODEL.LXij[i,j] >= MODEL.Li[i] - Lmax*(1 - MODEL.Xij[i,j]))
    
    
    # ConsCounter += 1
    ## Cons12: Subtour elimination 
    MODEL.Cons12 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))
    
    MODEL.Cons12.add(expr = MODEL.n['s'] == 0)   
    for i in N:
        for j in N:
            if i != j:
                MODEL.Cons12.add(expr = MODEL.n[j] >= MODEL.n[i] + 1 - len(N) *(1-MODEL.Xij[i,j]) ) 
    
    ConsCounter += 1
    ## Cons13: Visit Restaurants before customers
    ###### This constriant and subtour elimination can be removed when we have Ti in the model
    MODEL.Cons13 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))  
    for o in O:
        if not o[0] == '':
            i = o[0]+o[1]
            j = o[1]
            MODEL.Cons13.add(expr = MODEL.Ti[i] <= MODEL.Ti[j]) 
                          
    ConsCounter += 1
    ## Cons14: Time calculation
    MODEL.Cons14 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))
    for j in N:
        MODEL.Cons14.add(expr = MODEL.Ti[j] == Pyo.quicksum(MODEL.TXij[i,j] + EuclideanDist(NN[i], NN[j])/v*MODEL.Xij[i,j]  for i in N if i!=j))
    MODEL.Cons14.add(expr = MODEL.Ti['s'] == timer)

    # ConsCounter += 1
    ## Cons15: TX Linearization 
    MODEL.Cons15 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))
    Tmax = timer
    for i in N:
        for j in N:
            Tmax += EuclideanDist(NN[i], NN[j])

    for i in N:
        for j in N:
            if j=='s' or i==j:
                continue
            MODEL.Cons15.add(expr = MODEL.TXij[i,j] <= Tmax * MODEL.Xij[i,j])
            MODEL.Cons15.add(expr = MODEL.TXij[i,j] <= MODEL.Ti[i])
            MODEL.Cons15.add(expr = MODEL.TXij[i,j] >= MODEL.Ti[i] - Tmax*(1 - MODEL.Xij[i,j]))
    
    ConsCounter += 1
    ## Cons16: MinMax Linearization Constraints
    MODEL.Cons16 = Pyo.ConstraintList(name = 'Cons'+str(ConsCounter))
    for o in O:
        MODEL.Cons16.add(expr = MODEL.Z >= MODEL.Ti[o[1]] - o[3] )
            
    # solver = Pyo.SolverFactory('cplex')
    solver = Pyo.SolverFactory('cplex',executable = '/opt/ibm/ILOG/CPLEX_Studio2211/cplex/bin/x86-64_linux/cplex')
    # solver = Pyo.SolverFactory('cplex', executable = '/home/m/mahmoodian/Softwares/ibm/ILOG/CPLEX_Studio1210/cplex/bin/x86-64_linux/cplex')
    # solver = Pyo.SolverFactory('gurobi')
    
    
    solver.options['timelimit'] = 120  # Set time limit in seconds
    # solver.options['absmipgap'] = 1e-4  # Set absolute gap
    solver.options['mipgap'] = 0.1 
    
    Res = solver.solve(MODEL,tee=False)
    
    
    
    if Res.solver.termination_condition in  ['infeasible', 'unknown']:
        return -1,[]
    
    
    Orig = 's'
    Route = [(s[0],s[1], 's', L0, 0)]
    
    
    
    # print('---------------------------------------------------------')
    # print('Timer = ', timer)
    # for i in MODEL.Xij.keys():
    #     if MODEL.Xij[i]()>0.5:
    #         print(i, MODEL.Xij[i](), MODEL.Fi[i[0]](), MODEL.Li[i[0]](), MODEL.Ti[i[0]]())

    
    
    while True:
        for k in N:
            try:
                if k!=Orig and MODEL.Xij[Orig, k]()>0.5:
                    Orig = k
                    # print(Orig)
                    Route.append((NN[k][0], NN[k][1], k, MODEL.Li[k](),0))
                    break
            except:
                print(Res.solver.termination_condition)
                raise Exception("The Model was not solved and the condition is not listed.")
        if Orig == 'e':
            break
        
    return MODEL.Obj(), Route        
                
                        
        
def EuclideanDist(p1, p2, scale = 1/35):
    return np.sqrt(((p2[0]-p1[0])*scale)**2+((p2[1]-p1[1])*scale)**2)


def EnergyConsum(mode):
    if mode==1:
        return .0341,1.47*10**-5
    elif mode == 2:
        return 98733.33, 20166.7
    else: 
        raise Exception(ValueError, "Invalid Value for type:"+str(mode))
    
def MergeDic(X):
    return {j:i[j]   for i in X for j in i.keys()}
    
    
    


