import numpy as np
from .get_target_point import get_target_point
from cvxopt import matrix, solvers
# import cvxpy as cp
import math
# TODO: Tune parameters of PID with these global variables
param_Kp = 3
param_Ki = 0.0057
param_Kd = 0
# TODO: Tune parameters of Pure Pursuit with these global variables
param_K_dd = 0.3
# The above parameters will be used in the Carla simulation
# The simple simulation in tests/control/control.ipynb does not use these parameters


class MPC:
    def __init__(self, K_dd=param_K_dd, wheel_base=2.65, waypoint_shift=1.4):
        self.K_dd = K_dd
        self.wheel_base = wheel_base          # called L in the book
        self.waypoint_shift = waypoint_shift
        self.record = []
        self.U = np.array([[0],[0]])  # 用于记录控制量，即公式中的U, [v - v_r, \delta - \delta_r] 
        

        
        self.Nc = 30    # 控制区间
        self.Np = 100    # 预测区间
        self.Row = 10   # 松弛因子

        self.u1_bound = 0.2           # 控制变量（速度）的范围
        self.u2_bound = 0.436         # 控制变量（车轮转向角）的范围
        self.delta_u1_bound = 0.05    # 输入，\delta U 的范围
        self.delta_u2_bound = 0.0082
        
        self.Qq = 10                    # 矩阵Q前的系数 
        self.Rr = 200                  # 矩阵R前的系数
        self.Use_relaxation_factor = 1 # 是否使用松弛因子，1使用，0不使用

        self.old_sol_x = matrix(np.zeros( (self.Nc*2+self.Use_relaxation_factor,1) ),tc='d') # 用于储存上一时间步的解

        self.umin = [[-self.u1_bound], [-self.u2_bound]]  # 控制输入的大小空间
        self.umax = [[self.u1_bound] , [self.u2_bound]]

        self.Umin = np.kron(np.ones((self.Nc,1)), self.umin)
        self.Umax = np.kron(np.ones((self.Nc,1)), self.umax)

        self.delta_umin = [[-self.delta_u1_bound], [-self.delta_u2_bound]]
        self.delta_umax = [[ self.delta_u1_bound ], [self.delta_u2_bound]]

        self.delta_Umin = np.kron(np.ones((self.Nc,1)),self.delta_umin)
        self.delta_Umax = np.kron(np.ones((self.Nc,1)),self.delta_umax)

    def get_control(self,waypoints_world,waypoints_object,speed,state,v_r,dt):
        """获得控制，输入一些必要信息后，返回控制变量，这里返回期望速度和期望前轮转角

        首先根据输入的导航点，和当前速度，确定要跟踪的目标点。即pure pursuit中，获得目标点的方法。
        同样通过pure pursuit的方法，获得参考前轮偏角

        然后进行MPC计算

        Parameters
        ----------
        waypoints_world : list
            世界坐标系下的导航点，shape(n,3) 
            
            n表示一次性输入多少个导航点, 第二个维度对应[x, y, yaw] 即x，y坐标和偏航角
        waypoints_object : list
            体坐标系下的导航点，shape(n,3)
        speed : double
            当前速度(m/s)
        state : list
            表示当前车辆的状态，包括 [x, y, \phi, speed, \delta]，
            
            即当前车辆的x,y坐标(world)(m)，偏航角(deg)，速度(m/s)，前轮偏角(deg)
        v_r : double
            期望速度 (m/s)
        dt : double
            采样时间 (s)

        Returns
        -------
        output_speed : double
            输出的速度 (m/s)
        output_steer : double
            输出的前轮转向角 (rad)
        """
        # transform x coordinates of waypoints such that coordinate origin is in rear wheel
        waypoints_object[:,0] += self.waypoint_shift
        
        lookahead = np.clip(self.K_dd * speed, 3,20)

        coordinate_object,index = get_target_point(lookahead,waypoints_object,return_index=1)
        if(coordinate_object == None): # 如果没有目标点，则直接跳出
            return 0

        delta_r = self.get_delta_r(coordinate_object,lookahead)
        coordinate_world = waypoints_world[index]

        # if coordinate_world[2] < 0:
        #     coordinate_world[2] = coordinate_world[2] + 360
        [x_r, y_r, phi_r] = coordinate_world

        # 把所有的角度全部转换为弧度
        phi_r   = math.radians(phi_r)
        delta_r = math.radians(delta_r)
        reference = [x_r, y_r, phi_r, v_r, delta_r]
        
        state[2] = math.radians(state[2])
        state[4] = math.radians(state[4])
        # 求解MPC
        sol = self.Kinematic_Bicycle_MPC(state,reference,dt)
        
        # 更新控制量
        # self.U[0][0] = self.U[0][0] + sol[0]
        # self.U[1][0] = self.U[1][0] + sol[1]
        self.U[0][0] = self.U[0][0] + sol['x'][0]
        self.U[1][0] = self.U[1][0] + sol['x'][1]

        # 计算输出
        output_speed = self.U[0][0] + v_r
        output_steer = math.degrees (self.U[1][0] + delta_r)

        
        # 记录数据
        Delta_steer = sol['x'][1]
        # Delta_steer = sol[1]
        record_data = [x_r, y_r, phi_r, v_r, delta_r,state[0],state[1],state[2],state[3],state[4],Delta_steer]
        
        self.record.append(record_data)

        # undo transform to waypoints 
        waypoints_object[:,0] -= self.waypoint_shift
        
        return output_speed,output_steer

    def Kinematic_Bicycle_MPC(self,state,reference,T):
        """根据所给状态和参考值计算控制量

        Parameters
        ----------
        state : list
            表示当前车辆的状态，包括 [x, y, \phi, speed, \delta]，
            
            即当前车辆的x,y坐标(world)(m)，偏航角(rad)，速度(m/s)，前轮偏角(rad)
        reference : list
            表示路径所给出的参考值，包括 [x_r, y_r, \phi_r, v_r, \delta_r]
        T : double
            采样时间(s)
        
        Returns
        -------
        sol : double
            返回MPC的输出,sol[0]为速度 (m/s); sol[1]为前轮转角增量(rad)

            对应为\delta U
        """        

        [x_r, y_r, phi_r, v_r, delta_r] = reference
        
        kesi = np.array( np.array(state) - np.array(reference) ) # \kesi MPC中的状态量 kesi = [x - x_r, y - y_r, phi - phi_r,speed - v_r,delta - delta_r]
        kesi[-2] = self.U[0][0]
        kesi[-1] = self.U[1][0]                                  # U 控制量

        
        kesi = np.array(kesi).reshape(-1,1)                      # 变为列向量
        
        # 为直观，以下变量重新赋值了一遍
        l = self.wheel_base                                      # 车的前后轴长
        Nc  =self.Nc 
        Np  =self.Np 
        Row =self.Row
        Qq  = self.Qq        
        Rr  = self.Rr        
        

        A = np.array([[1, 0, -v_r * math.sin(phi_r) * T],
                      [0, 1,  v_r * math.cos(phi_r) * T],
                      [0, 0, 1]])

        B = np.array([[math.cos(phi_r) * T,     0],
                      [math.sin(phi_r) * T,     0],
                      [math.tan(delta_r) * T/l, v_r * T / (math.cos(delta_r)**2)]])
        C = np.eye(3)

        Nx = A.shape[0] # 状态量维度
        Nu = B.shape[1] # 控制量维度
        
        tilde_A = np.block([[A,               B],
                            [np.zeros((Nu,Nx)), np.eye(Nu)]])
        tilde_B = np.block([[B],
                            [np.eye(Nu)]])

        tilde_C = np.block([C, np.zeros((Nx,Nu))])

        Theta = []
        Phi   = []
        for j in range(Np):
            Theta_row = []
            Phi.append(tilde_C @ np.linalg.matrix_power(tilde_A, j+1))
            # Phi.append(np.dot(tilde_C, np.linalg.matrix_power(tilde_A, j)))
            for k in range(Nc):
                if k <= j:
                    Theta_row.append(tilde_C @ np.linalg.matrix_power(tilde_A, j - k) @ tilde_B)
                    # Theta_row.append(np.dot(np.dot(tilde_C, np.linalg.matrix_power(tilde_A, j - k)), tilde_B))
                else:
                    Theta_row.append(np.zeros((Nx, Nu), dtype=np.float))
            Theta.append(Theta_row)

        Theta = np.reshape(np.array(Theta),(Np * Nx,Nc * Nu))
        Phi   = np.reshape(np.array(Phi),(Np * Nx,tilde_A.shape[1]))

        e = Phi @ kesi
        # Qq = np.eye(Nx)
        # Rr = np.eye(Nu)
        # Q = np.kron( np.eye(Np), Qq)
        # R = np.kron( np.eye(Nc), Rr)
        Q = Qq * np.eye(Nx*Np)
        R = Rr * np.eye(Nu*Nc)

        A_constrain = np.kron( np.tri(Nc) , np.eye(Nu))
        U_t = np.kron(np.ones((Nc,1)),self.U)

        if self.Use_relaxation_factor:
            H = np.block([[Theta.T @ Q @ Theta + R,  np.zeros((R.shape[0],1))],
                        [np.zeros((1,R.shape[1])), Row]])

            G = np.block([2 * e.T @ Q @ Theta, 0])

            A_constrain = np.block([[ A_constrain, np.zeros((A_constrain.shape[0],1))],
                                    [-A_constrain, np.zeros((A_constrain.shape[0],1))]])

            b_constrain = np.block([[self.Umax - U_t],[U_t - self.Umin]])

            lb = np.block([[self.delta_Umin],[0]])
            ub = np.block([[self.delta_Umax],[10]])

            A_constrain = np.concatenate( (A_constrain, np.eye((Nc*Nu+1)), -np.eye((Nc*Nu+1))) ) 
            b_constrain = np.concatenate( (b_constrain,ub,-lb) )

        else:
            H = Theta.T @ Q @ Theta + R

            G = 2 * e.T @ Q @ Theta     

            A_constrain = np.block([[A_constrain], [-A_constrain]])

            b_constrain = np.block([[self.Umax - U_t],[U_t - self.Umin]])

            lb = self.delta_Umin
            ub = self.delta_Umax
            A_constrain = np.concatenate((A_constrain,np.eye(Np*Nu),-np.eye(Np*Nu)))
            b_constrain = np.concatenate((b_constrain,ub,-lb))

        H = matrix(H, tc='d')
        G = matrix(G, tc='d')
        A_constrain = matrix(A_constrain, tc='d')
        b_constrain = matrix(b_constrain, tc='d')
        solvers.options["show_progress"] = False
        sol = solvers.qp(P = H,q = G.T,G = A_constrain,h = b_constrain,initvals={'x':self.old_sol_x})

        if sol['status'] == 'unknow':
            print("Status of qp is Unknow,meas that outputs are the last iterates before termination,These satisfy s > 0 and z > 0,but are not necessarily feasible")
        self.old_sol_x = sol['x']
        # x = cp.Variable(Nu * Nc + Use_relaxation_factor)
        # prob = cp.Problem(cp.Minimize((1/2)*cp.quad_form(x, H) + G.reshape(-1) @ x),
        #          [A_constrain @ x <= b_constrain.reshape(-1)])
        # # prob.solve(solver=cp.ECOS, verbose=False)
        # prob.solve(solver = cp.OSQP,verbose = False)
        # if prob.status != cp.OPTIMAL and prob.status != cp.OPTIMAL_INACCURATE:
        #     print("Error: Cannot solve mpc..")
        
        # sol = x.value

        return sol
    
    def get_delta_r(self,coordinate,lookahead):
        """通过体坐标系的导航点和lookahead获得目标点的期望前轮偏角值

        Parameters
        ----------
        coordinate : list
            体坐标系下，目标点的坐标
        lookahead : double
            

        Returns
        -------
        delta_r : double
            期望前轮偏角值
        """
        x,y,_ = coordinate
        alpha = math.atan2(y, x)
        delta_r = math.atan(2 * self.wheel_base * math.sin(alpha)/lookahead) # called delta in the book

        return delta_r


class PIDController:
    """ PID Controller copied from book """
    def __init__(self, Kp, Ki, Kd, set_point):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.set_point = set_point
        self.int_term = 0
        self.derivative_term = 0
        self.last_error = None
    
    def get_control(self, measurement, dt):
        error = self.set_point - measurement
        self.int_term += error*self.Ki*dt
        if self.last_error is not None:
            self.derivative_term = (error-self.last_error)/dt*self.Kd
        self.last_error = error
        return self.Kp * error + self.int_term + self.derivative_term


class MPCPlusPID:
    def __init__(self, MPC=MPC(), pid=PIDController(param_Kp, param_Ki, param_Kd, 0)):
        self.MPC = MPC
        self.pid = pid

    def get_control(self,waypoints_world,waypoints_object,speed, desired_speed, dt,state):
        mpc_desire_speed,steer = self.MPC.get_control(waypoints_world,waypoints_object,speed,state,desired_speed,dt)
        self.pid.set_point = mpc_desire_speed
        a = self.pid.get_control(speed,dt)
        return a, steer

