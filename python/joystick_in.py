import sys;
import pygame;
import time;

stop = False
sweep = True
sweep_pos = 1;

x = 0.0
y = 0.0
z = 0.0
t = 0.0

def handle_event(event):
    global x
    global y
    global z
    global t
    if e.type == pygame.JOYAXISMOTION:  
        axis = "unknown"  
        if (e.dict['axis'] == 0):  
            axis = "X"  
  
        if (e.dict['axis'] == 1):  
            axis = "Y"  
  
        if (e.dict['axis'] == 2):  
            axis = "Z"
  
        if (e.dict['axis'] == 3):  
            axis = "Throttle"  
  
        if (axis != "unknown"):  
            #str = "Axis: %s; Value: %f" % (axis, e.dict['value'])

            if (axis == "X"):
                x = e.dict['value']
            if (axis == "Y"):
                y = e.dict['value']
            if (axis == "Z"):
                z = e.dict['value']
            if (axis == "Throttle"):
                t = e.dict['value']
                
  
    elif e.type == pygame.JOYBUTTONDOWN:  
        if (e.dict['button'] == 5):  
            global stop
            stop = True
        if (e.dict['button'] == 6):  
            sys.stdout.write("connect\n")
            time.sleep(2)
            pygame.time.set_timer(pygame.USEREVENT, 500)
            
    elif e.type == pygame.USEREVENT:
        if (sweep):
            sys.stdout.write("set %d %d %d\n" % ((350*y-100*x), (350*y+100*x), sweep_pos*450.0))
        else:
            sys.stdout.write("set %d %d %d\n" % ((750*y-150*x), (750*y+150*x), -t*1822/2))
    else:  
        pass  

pygame.init()

# Create joystick object
joy = pygame.joystick.Joystick(0)

joy.init()
count = 0;

while (not stop):
    e = pygame.event.wait()
    handle_event(e)
    count = count + 1
    if (count % 25 == 0):
        sweep_pos = -sweep_pos
    
sys.stdout.write("quit\n")
