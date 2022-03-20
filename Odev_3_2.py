import matplotlib.pyplot as plt
import numpy as np

animation=True
def movement(x_start,y_start,theta_start,dt,x_rex,y_rec,x1,y1,v_error_r_sum ,v_error_l_sum ):
    while True:
        """
        Movement func for follow-trajectory algorithm
        """  

        x_goal, y_goal, x1, y1 = cos_draw(dt)
    
        v_error_r,v_error_l,theta_error = calculate_error(x_start,y_start,theta_start,x_goal,y_goal,d)

        a,v_error_r_sum  = PI_V_R(v_error_r,v_error_r_sum,dt)
        c,v_error_l_sum  = PI_V_L(v_error_l,v_error_l_sum,dt)
        x_n,y_n,theta_n,dt = diff_drive(x_start,y_start,theta_start,theta_error,a,c,dt)

        x_rec,y_rec=route_draw(x_n,y_n)    
      
        x_start, y_start, theta_start = recursive_position(x_n,y_n,theta_n)

        if animation:
            plt.cla()
            vehicle(x_n,y_n,theta_n, x_rec, y_rec,x1,y1) 
       
def calculate_error(x_start,y_start,theta_start,x_goal,y_goal,d):
    """
    This func calculate error.
    """
    u1=x_goal-x_start
    u2=y_goal-y_start
    
    v_error_r = np.sqrt((u1)**2+(u2)**2)-d
    v_error_l = np.sqrt((u1)**2+(u2)**2)-d

    theta_goal = np.arctan2(u2,u1)
    u3=theta_goal-theta_start
    theta_error = np.arctan2(np.sin(u3),np.cos(u3))
    
    return v_error_r,v_error_l,theta_error

def PI_V_R(v_error_r,v_error_r_sum,dt):
    """ 
    PI controller for right wheel
    """
    kd=0.05
    kı=0.00001
    p=kd*v_error_r
    I=v_error_r_sum*kı
    PID=p+I
    v_error_r_sum+=v_error_r

    
    return PID, v_error_r_sum 

def PI_V_L(v_error_l,v_error_l_sum,dt):
    """
    PI controller for left wheel
    """
    kd=0.05
    kı=0.00001
    p=kd*v_error_l
    I=v_error_l_sum*kı
    PID=p+I
    v_error_l_sum+=v_error_l

    
    return PID, v_error_l_sum 

def P_H(theta_error):
    """
    P controller for heading angle
    """
    Kh=0.1
    return theta_error*Kh

def vehicle(x_n, y_n, theta_n, x_rec, y_rec,x1,y1):  
    """
    The function simulate vehicle.
    """
    start_1,start_2,start_3 = vehicle_initial_position()

    T = transformation_matrix(x_n, y_n, theta_n) 

    p1,p2,p3 = matrix_mul(T,start_1,start_2,start_3)

    plot_vehicle(p1,p2,p3)
   

    plt.plot(x1,y1, ',-r') # plotting cos wave
    plt.plot(x_rec, y_rec, '.c')# plotting robots route
    
    #If ESC is pressed, the plotting screen is closed.
    plt.gcf().canvas.mpl_connect('key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None]) 
    
    plt.xlim(-20, 30)
    plt.ylim(-3, 3)
    
    plt.pause(dt)

def transformation_matrix(x_n, y_n, theta_n):
    """ 
    Transformation matrix of robot
    """
    return np.array([
        [np.cos(theta_n), -np.sin(theta_n), x_n],
        [np.sin(theta_n), np.cos(theta_n), y_n],
        [0, 0, 1]
    ])

def cos_draw(dt):
    """
    the function that draws the cosine wave against time
    """
    x_goal = dt
    y_goal = np.cos(dt)
        
    x1.append(x_goal)
    y1.append(y_goal)

    return x_goal, y_goal, x1, y1

def diff_drive(x_start,y_start,theta_start,theta_error,a,c,dt):
    """
    New pose of diff drive robot 
    """
    x_n = x_start + (1/2)*(a+c)*np.cos(theta_start)*dt
    y_n = y_start + (1/2)*(a+c)*np.sin(theta_start)*dt
    theta_n = theta_start+P_H(theta_error)*dt
    dt+=0.1
    return x_n,y_n,theta_n,dt

def vehicle_initial_position ():
    """
    vehicle initial position function
    """

    start_1 = np.array([0.25, 0, 1])
    start_2 = np.array([-0.25, 0.1, 1])
    start_3 = np.array([-0.25, -0.1, 1])

    return start_1,start_2,start_3

def recursive_position(x_n,y_n,theta_n):
    """
    Recursive position of robot.
    We assign the newly pose as the old pose.
    """
    x_start = x_n
    y_start = y_n
    theta_start = theta_n
    return x_start, y_start, theta_start

def route_draw(x_n,y_n):
    """
    The function keeps the locations of the robot in the list.
    """
    x_rec.append(x_n)
    y_rec.append(y_n)
    return x_rec,y_rec

def matrix_mul(T,start_1,start_2,start_3):
    """
    Matrix multiplication function of the initial position of the vehicle and the updated position.
    """
    p1 = np.matmul(T, start_1)
    p2 = np.matmul(T, start_2)
    p3 = np.matmul(T, start_3)

    return p1,p2,p3

def plot_vehicle(p1,p2,p3):
    """
    The function plot vehicle.
    """
    plt.plot([p1[0], p2[0]], [p1[1], p2[1]], color='green', marker='.', linestyle='-')
    plt.plot([p2[0], p3[0]], [p2[1], p3[1]], color='green', marker='.', linestyle='-')
    plt.plot([p3[0], p1[0]], [p3[1], p1[1]], color='green', marker='.', linestyle='-')

if __name__=='__main__':
    x_start=-15
    y_start=0
    theta_start=0
    dt=0.1
    x_rec, y_rec = [], []
    x1, y1 = [],[]
    d=1
    v_error_r_sum , v_error_l_sum =0,0

    movement(x_start,y_start,theta_start,dt,x_rec,y_rec,x1,y1,v_error_r_sum ,v_error_l_sum )
