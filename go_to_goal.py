from math import sqrt,atan2,sin,cos
import pylab as plt

R = 0.05 #wheel radius
L = 0.3 #base length
[x0,y0,q0]=[0,0,0] #initial position
[xg,yg]=[10,20] #target pose
dt=0.1
Kv=0.5 #velocity gain
Kh=2 #heading gain

print("inital position:",x0,y0,q0)



def velocity_controller(x0,y0,xg,yg):
     ev =sqrt((xg-x0)**2 + (yg-y0)**2)
     v=Kv*ev
     return v
 


def heading_controller(x0,y0,q0,xg,yg):
    qg= atan2(yg-y0,xg-x0)
    eq=atan2(sin(qg-q0),cos(qg-q0))
    w=Kh*eq
    return w

def wr_wl_controller(v,w,R,L): #this function used because of problem
    wr = ((2 * v) + (w * L)) / (2 * R)
    wl = ((2 * v) - (w * L)) / (2 * R)
    return wr,wl
    
    
v_c=velocity_controller(x0,y0,xg,yg) #velocity controller 
w_c=heading_controller(x0,y0,q0,xg,yg) #heading controller 
wr,wl=wr_wl_controller(v_c,w_c,R,L)

def next_step(x0,y0,q0,wr,wl,R,L):
    x0 = x0 + (R*dt/2.0)*(wr+wl)*cos(q0)
    y0 = y0 + (R*dt/2.0)*(wr+wl)*sin(q0)
    q0 = q0 + (R*dt/(L))*(wr-wl)
    return (x0,y0,q0)

x=list()
y=list()
q=list()

while( v_c >=0.001): # if i set velocity error to zero this program doesn't work
    
    v_c=velocity_controller(x0,y0,xg,yg) #velocity controller updates according to error
    w_c=heading_controller(x0,y0,q0,xg,yg) #heading controller updates according to error
    wr,wl=wr_wl_controller(v_c,w_c,R,L)

    x0,y0,q0 = next_step(x0,y0,q0,wr,wl,R,L) 
    x.append(x0)
    y.append(y0)
    q.append(q0)
    
   
print("final position:", round(x0,2),round(y0,2),round(q0,2))
plt.plot(x,y,'rv')
plt.show()
