import numpy
from PIL import Image,ImageDraw
import matplotlib.pyplot as plt
import numpy as np

theta_initial = (np.pi/3)
theta_deriv_initial = 0
dt = 0.05
m = 3
l = 1
g = 9.8
width = 1280
height = 720
frames_per_second= 60
take_frame_every=int(1/(dt*frames_per_second))     
save_path=''
number_Of_Iterations = 100

def calcuateTrajectoryEulerCromerMethod(theta_initial,theta_deriv_initial,m,l,g,dt,number_Of_Iterations):
    trajectory = np.ones([number_Of_Iterations,2])
    # Set the initial conditions
    
    trajectory[0,0] = theta_initial
    trajectory[0,1] = theta_deriv_initial  
    
    for i in range(0,number_Of_Iterations-1):
        # Calculation of the angluar velocity
        trajectory[i+1,1] = trajectory[i,1] - (g/l)*np.sin(trajectory[i,0])*dt                      
        # Calculation of the theta points
        trajectory[i+1,0] = trajectory[i,0] + trajectory[i+1,1]*dt
    return trajectory

def getEnergy(theta,angular_velocity):
    y = l * np.cos(theta)
    potential_Energy = (-1)*m*g*y  
    kinetic_Energy = 1/2*m*(l**2)*(angular_velocity**2)
    return potential_Energy , kinetic_Energy

def draw_pendulum(theta,w,h,m,l):
    #create image with width=w, and height = h
    img = Image.new('RGB',(w,h),"white")
    # Convert length of pendulum to the same image units\
    L=int(0.4*h*l)
    # Define a diameter for the pendulums mass
    d = int(0.02*h)*m**(1/4)
    # Create the draw objects of the image
    draw = ImageDraw.Draw(img)
    #calculate cartesian coordinates
    x0 = int(w/2)
    y0 = int(w/4)
    x = x0 + L*np.sin(theta)
    y = y0 + L*np.cos(theta)
    # draw the pendulum
    draw.line([(x0,y0),(x,y)],fill=(0,0,0),width=3)
    draw.ellipse([(x-d,y-d),(x+d,y+d)],fill=(0,0,255),outline=None)
    return img
    
def render_pendulum(m,l,g,save_path,take_frame_every,trajectile):
    frames = [] # We will collect the frames here",
    for i in range(trajectile.shape[0]):
        #get the i-th angle and i'th angular velocity",
        theta = trajectile[i,0]
        angular_vel = trajectile[i:1]
        # draw the corresponding image and add to our frame list
        img=draw_pendulum(theta,width,height,m,l)
        frames.append(img)
    frames[0].save(save_path+'pendulum_simulation.gif', save_all=True,append_images=frames[1:],duration=40,loop=0),
 
 
trajectory_EulerCromer = calcuateTrajectoryEulerCromerMethod(theta_initial,theta_deriv_initial,m,l,g,dt,number_Of_Iterations)
render_pendulum(m,l,g,save_path,take_frame_every,trajectory_EulerCromer)
