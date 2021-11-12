import mtl 
from mtl import utils
from discrete_signals import signal
from utils import signals
import math
import datetime
import time


# AUX
def verify(result):
    time = [] 

    for t, b in result:
        if not b:
            time.append(t)

    return time

def distanceToLine(a,b,c):
    return ((b[0] - a[0])*(c[1] - a[1]) - (b[1] - a[1])*(c[0] - a[0]))

def compute_slope(x0,y0,x1,y1):
            try:
                return (y1-y0)/(x1-x0)
            except:
                return float('inf')

def compute_left_point(x,y,distance, slope):
            if slope == 0:
                return x, y + distance
            elif slope == float('inf'):
                return x - distance, y
            else:
                dx = distance*(1/(math.sqrt(1+slope**2)))
                dy = slope * dx
                return x+ dx, y-dy 
        
def compute_right_point(x,y,distance, slope):
    if slope == 0:
        return x, y - distance
    elif slope == float('inf'):
        return x + distance, y
    else:
        dx = distance*(1/(math.sqrt(1+slope**2)))
        dy = slope * dx
        return x - dx, y + dy 

    

def sign(number):
    if number<0:
        return -1
    else:
        return 1

def compute_distance(x0,y0,x1,y1):
            return math.sqrt(math.pow(x1-x0,2)+ math.pow(y1-y0,2))


# works only for runs less than an hour for now
def convert_seconds_to_timpstamp(time0,time):
    time_stamp = f"{math.floor(float(time)/60)}:{round(float(time)%60,3)}"
    time_p =  time0+datetime.datetime.strptime(time_stamp,"%M:%S.%f")
    return datetime.datetime.strftime(time_p,"%H:%M:%S,%f")



# ====================================================================================================
# ========================================= Geo-Fencing ==============================================
# Monitor for checking whether robot fell off the ramp. Monitor issues warning when the robot
# is to close the the edges of the ramp. Closeness is determined using the parameter "margin". Monitor
# issues a violation when robot falls off the ramp. 
def out_of_bounds(ramp_coordinates, margin, time0):

    
    # preprocess ramp coordinates 
    bottom_left = ramp_coordinates[0]
    top_left = ramp_coordinates[1]
    top_right = ramp_coordinates[2]
    bottom_right = ramp_coordinates[3]
    cliff_left = ramp_coordinates[4]
    cliff_right = ramp_coordinates[5]

    
    print("=======================================================")
    print("                     Geo-Fencing                       ")
    print(".......................................................")

    violation = False

    # position of robot
    position = signals.x_pos | signals.y_pos | signals.z_pos

    # robot is left the ramp at its base: <bottom_left> -- <bottom_right>
    off_ramp = position.map(lambda val: distanceToLine(bottom_left,bottom_right,(val['x_pos'],val['y_pos']))<=0, tag="off_ramp")

    # robot to close to left edge: <bottom_left> -- <top_left>
    too_left = position.map(lambda val: distanceToLine(bottom_left,top_left,(val['x_pos'],val['y_pos']))+margin>0 and distanceToLine(bottom_left,top_left,(val['x_pos'],val['y_pos']))<0, tag="too_left")
    
    # specification for checking robot too close to left edge. Checked only while robot on ramp
    data = too_left | off_ramp
    phi = mtl.parse("(~too_left)")
    result = phi(data,time=None,quantitative=False)
    phi_p = mtl.parse("~off_ramp")
    result_p = phi_p(data,time=None,quantitative=False)
    verdict = verify(result) # times ~too_left was violated
    verdict_p = verify(result_p) # times ~off_ramp was violated, i.e., robot left ramp. Needed to obtain time verdict_p[0] when robot left ramp 
    if verdict!=[]:
        #filter
        filtered_verdict = []
        if verdict_p!=[]:
            for t in verdict:
                if t<verdict_p[0]:
                    filtered_verdict.append(convert_seconds_to_timpstamp(time0,t)) # times where robot was on ramp and close to left edge 
            if filtered_verdict!=[]:
                print(f"WARNING: too close to left edge at times {filtered_verdict}. Check orientation of robot")
                print("-------------------------------------------------------------------------")
                violation = True
        else:
            print(f"WARNING: too close to left edge at times {list(map(lambda val:convert_seconds_to_timpstamp(time0,val),verdict))}. Check orientation of robot")
            print("-------------------------------------------------------------------------")
            violation = True
        
    # robot out of bounds w.r.t. left edge of ramp
    oob_left = position.map(lambda val: distanceToLine(bottom_left,top_left,(val['x_pos'],val['y_pos']))-margin>0, tag="oob_left")

    # specification for checking robot fell off left edge. Checked only while robot on ramp
    data = oob_left | off_ramp
    phi = mtl.parse("(((~oob_left) U  off_ramp) | (G(~oob_left)))") # specification of weak until
    result = phi(data,time=0,quantitative=False)
    phi_p = mtl.parse("(~oob_left)")
    result_p = phi_p(data,time=None,quantitative=False)
    verdict = verify(result_p) # verdict[0] is first time was out of bounds
    if not result:
        print(f"VIOLATION: Robot fell off left edge at time {convert_seconds_to_timpstamp(time0,verdict[0])}. Check usage of cliff sensor.")
        print("-------------------------------------------------------------------------")
        violation = True
        
    # robot to close to left edge: <bottom_right> -- <top_right>
    too_right = position.map(lambda val: distanceToLine(bottom_right,top_right,(val['x_pos'],val['y_pos']))-margin<0 and distanceToLine(bottom_right,top_right,(val['x_pos'],val['y_pos']))>0, tag="too_right")

    # specification for checking robot too close to right edge. Checked only while robot on ramp
    data = too_right | off_ramp
    phi = mtl.parse("(~too_right)")
    result = phi(data,time=None,quantitative=False)
    phi_p = mtl.parse("~off_ramp")
    result_p = phi_p(data,time=None,quantitative=False)
    verdict = verify(result)
    verdict_p = verify(result_p) # times ~off_ramp was violated, i.e., robot left ramp. Needed to obtain time verdict_p[0] when robot left ramp 
    if verdict!=[]:
        #filter
        filtered_verdict = []
        if verdict_p!=[]:
            for t in verdict:
                if t<verdict_p[0]:
                    filtered_verdict.append(convert_seconds_to_timpstamp(time0,t)) # times where robot was on ramp and close to right edge 
            if filtered_verdict!=[]:
                print(f"WARNING: too close to right edge at times {filtered_verdict}. Check orientation of robot")
                print("-------------------------------------------------------------------------")
                violation = True
        else:
            print(f"WARNING: too close to right edge at times {list(map(lambda val:convert_seconds_to_timpstamp(time0,val),verdict))}. Check orientation of robot")
            print("-------------------------------------------------------------------------")
            violation = True
        

    # robot out of bounds w.r.t. right edge of ramp
    oob_right = position.map(lambda val: distanceToLine(bottom_right,top_right,(val['x_pos'],val['y_pos']))+margin<0, tag="oob_right")

    # specification for checking robot fell off right edge. Checked only while robot on ramp
    data = oob_right | off_ramp
    phi = mtl.parse("(((~oob_right) U  off_ramp) | (G(~oob_right)))")
    result = phi(data,time=0,quantitative=False)
    phi_p = mtl.parse("(~oob_right)")
    result_p = phi_p(data,time=None,quantitative=False)
    verdict = verify(result_p) # verdict[0] is first time was out of bounds
    if not result:
        print(f"VIOLATION: Robot fell off right edge at time {convert_seconds_to_timpstamp(time0,verdict[0])}. Check usage of cliff sensor.")
        print("-------------------------------------------------------------------------")
        violation = True
    
    # robot out of bounds w.r.t. top cliff of ramp
    oob_top_c = position.map(lambda val: distanceToLine(cliff_left,cliff_right,(val['x_pos'],val['y_pos']))-margin>0, tag="oob_top_c")
    # oob_top_l = position.map(lambda val: distanceToLine(top_left,cliff_left,(val['x_pos'],val['y_pos']))-margin>0, tag="oob_top_l")
    # oob_top_r = position.map(lambda val: distanceToLine(top_right,cliff_right,(val['x_pos'],val['y_pos']))+margin<0, tag="oob_top_r")
    # off_top = position.map(lambda val: distanceToLine(top_left,top_right,(val['x_pos'],val['y_pos']))<0, tag="off_top" )

    # specification for checking robot fell off top cliff. Checked only while robot on ramp
    data = oob_top_c | off_ramp 
    phi = mtl.parse("(((~oob_top_c) U  off_ramp)| G(~oob_top_c))")
    result = phi(data,time=0,quantitative=False)
    phi_p = mtl.parse("(~oob_top_c)")
    result_p = phi_p(data,time=None,quantitative=False)
    verdict = verify(result_p)
    if not result:
        print(f"VIOLATION: Robot fell off top of the ramp at time {convert_seconds_to_timpstamp(time0,verdict[0])}. Check usage of cliff sensor.")
        violation = True

    
    if not violation:
        print("No violation.")

    print("***********************************************************\n")

# ====================================================================================================
# ====================================================================================================

# ====================================================================================================
# ==================================== Circle in place ===============================================
# Monitor for checking whether robot is circling in place. Monitor issues violation if robot is  
# circling more than one time at beginning of run. Throughout simulation, monitor issues violation if 
# robot does circle in place during climbing or after a cliff event.  
def circling(circle_time,initialization_time,time0):
    print("=======================================================") 
    print("                  Circling in place                    ")
    print(".......................................................")

    violation = False

    # turn angle of robot is between -6 and 6
    # first make all angles posiitve
    positive_z_angle = signals.z_angle.map (lambda val: 360-(abs(val['z_angle'])%360) if val['z_angle']<0 else val['z_angle']%360,tag='positive_z_angle')
    yaw0 = positive_z_angle.map(lambda val: (val["positive_z_angle"])<=6,tag="yaw0") 

    # differntial of z_angle to compute turning. 
    z_angle_values = signals.z_angle.values()
    z_angle_times = list(signals.z_angle.times())
    count = 0
    data = []
    rotation_data = [(0.0,0)]
    for val in z_angle_values:
        data.append((z_angle_times[count],val['z_angle']))
        count +=1
    count = 0
    while count<len(data)-1:
        current_time = data[count+1][0]
        value1 = data[count][1]
        value2 = data[count+1][1]
        current_value  = value1 - value2
        rotation_data.append((current_time,current_value))
        count += 1


    rotation = signal(rotation_data,start=0, end=float('inf'), tag='rotation')
    
    
    # robot is truning left with speed larger than 1
    reorient_left = rotation.map(lambda val: val["rotation"]< 0, tag="reorient_left")
    # robot is turning right with speed larger than 1
    reorient_right = rotation.map(lambda val: val["rotation"] > 0, tag="reorient_right")
    # robot circles twice at begnning 
    circle_twice = signals.z_angle.map(lambda val: abs(val['z_angle'])>730,tag='circle_twice')

    # specification for checking whether robot is circling in place 
    data = (reorient_left|reorient_right) | yaw0 | circle_twice
    # check at beginning of run
    # phi = mtl.parse(f"~((yaw0 & reorient_left) & ((reorient_left) U[5,{circle_time}] ((yaw0 & reorient_left) & (reorient_left U[5,{circle_time}] yaw0) )))") # negation of spec circling counterclockwise in place twice
    phi_l = mtl.parse(f"~(reorient_left U circle_twice)")
    phi_r = mtl.parse(f"~(reorient_right U circle_twice)")
    result_l = phi_l(data,time=10,quantitative=False)
    result_r = phi_r(data,time=10,quantitative=False)
    if not result_r or not result_l:
        print(f"VIOLATION: Initial circling in place. Possible cause: Use of strict equality (or very low tolerance) in re-orientation check")
        violation = True
   
    # check throuhgtout run
    phi = mtl.parse(f"~((yaw0 & reorient_left) & ((reorient_left) U[5,{circle_time}] yaw0 ))") # negation of spec circling counterclockwise in place once
    result = phi(data,time=None,quantitative=False)
    verdict = verify(result) # verdict[0] is first time was robot was circling in place
    # filter out all times related to initial circling error
    filtered_verdict = []
    for t in verdict:
        if t > initialization_time:
            filtered_verdict.append(t)
    if filtered_verdict!=[]:
        print(f"VIOLATION: Circling counterclockwise in place at time {convert_seconds_to_timpstamp(time0,verdict[0])}. Possible cause: Use of strict equality (or very low tolerance) in re-orientation check")
        violation = True
    
    # phi = mtl.parse(f"~((yaw0 & reorient_right) & ((reorient_right) U[5,{circle_time}] ((yaw0 & reorient_right) & (reorient_right U[1,{circle_time}] yaw0) )))") # negation of spec circling clockwise in place twice
    phi = mtl.parse(f"~((yaw0 & reorient_right) & ((reorient_right) U[5,{circle_time}] yaw0 ))") # negation of spec circling clockwise in place once
    result = phi(data,time=None,quantitative=False)
    verdict = verify(result) # verdict[0] is first time was robot was circling in place
    if verdict!=[]:
        print(f"VIOLATION: Circling clockwise in place at time {convert_seconds_to_timpstamp(time0,verdict[0])}. Possible cause: Use of strict equality (or very low tolerance) in re-orientation check")
        violation = True
    

    if not violation:
        print("No violation.")
    print("***********************************************************\n")

# ====================================================================================================
# ====================================================================================================

# ====================================================================================================
# ======================================= Reach top ==================================================
# Monitor checks whether robot reached top of the ramp within "timeout" seconds. Coordinates of robot 
# have to be behind the the beginning of the top part of the ramp given by the line 
# <top_left> -- <top_right>. Monitor issues violation when robot descends before reaching top. 
# Descends caused by backing off cliff are ignored. 
def reached_top(ramp_coordinates, margin, robot_radius, time0, backing_off_time, initialization_time, timeout=30.0):

    # preprocess ramp coordinates
    top_left = ramp_coordinates[1]
    top_right = ramp_coordinates[2]
    violation = False

    print("=======================================================")
    print("                 Reaching top of ramp                  ")
    print(".......................................................")

    # position of robot
    position = signals.x_pos | signals.y_pos | signals.z_pos

    # robot reached top of ramp: passed the line <top_left> -- <top_right>
    on_top = position.map(lambda val: distanceToLine(top_left,top_right,(val['x_pos'],val['y_pos']))+margin>0,tag="on_top")

    data = on_top 

    # reaching top of ramp check
    phi = mtl.parse(f"F[0,{timeout}](on_top)")
    result = phi(data,time=0,quantitative=False)
    if not result:
        print(f"VIOLATION: Top not reached within {timeout} seconds")
        print("-------------------------------------------------------------------------")
        violation =True


    if not violation:
        print("No violation.")
    print("***********************************************************\n")

# ====================================================================================================
# ====================================================================================================

# ====================================================================================================
# ===================================== Return to bottom =============================================
# # Go back down after reaching top of the ramp. Monitor issues violation if robot does not return 
# to base of the ramp after reaching time within "timeout" seconds
def return_to_bottom(ramp_coordinates, margin, time0,timeout=30.0):

    # preprocess coordinates
    bottom_left = ramp_coordinates[0]
    top_left = ramp_coordinates[1]
    top_right = ramp_coordinates[2]
    bottom_right = ramp_coordinates[3]

    print("=======================================================")
    print("             Returning to bottom of ramp               ")
    print(".......................................................")

    # position of robot
    position = signals.x_pos | signals.y_pos | signals.z_pos

    # robot reached top of ramp: passed the line <top_left> -- <top_right>
    on_top = position.map(lambda val: distanceToLine(top_left,top_right,(val['x_pos'],val['y_pos']))+margin>0,tag="on_top")
    at_bottom = position.map(lambda val: distanceToLine(bottom_left,bottom_right,(val['x_pos'],val['y_pos']))-margin<0,tag="at_bottom")
    
    # robot should return to bottom after reaching top within timeout seconds
    data = on_top | at_bottom
    phi = mtl.parse(f"(on_top -> F[0,{timeout}](at_bottom))")
    result = phi(data,time=None,quantitative=False)
    verdict = verify(result)
    if verdict!=[]:
        print(f"VIOLATION: robot reached top at {convert_seconds_to_timpstamp(time0,verdict[0])} and did not return to bottom within {timeout} seconds ")
    else:
        print("No violation. This does not guarantee however that robot reached top. Check reaching top monitor.")
    print("***********************************************************\n")

# ====================================================================================================
# ====================================================================================================

# ====================================================================================================
# ===================================== Rotate on cliff ==============================================
# Monitor checks whether robot turns "turn_angle" degrees after cliff event
def rotate_on_cliff(ramp_coordinates,robot_radius,rotation_time,turn_angle,time0):

    print("=======================================================")
    print("                    Rotate on cliff                    ")
    print(".......................................................")

    # preprocess coordinates
    bottom_left = ramp_coordinates[0]   
    bottom_right = ramp_coordinates[3]
    
    violation = False

    # compute angle at cliff signal: z_angle value when cliff event is true
    # first make all angles posiitve
    positive_z_angle = signals.z_angle.map (lambda val: 360-(abs(val['z_angle'])%360) if val['z_angle']<0 else val['z_angle']%360,tag='positive_z_angle')
    z_angle_values = positive_z_angle.values()
    z_angle_times = list(positive_z_angle.times())
    at_cliff = signals.cliff.map(lambda val: val['cliff']!=-1,tag='at_cliff').filter(lambda val: val['at_cliff'])
    cliff_times = list(at_cliff.times())
    count = 0
    z_angle_count = 0
    angle_at_cliff_data = []
    while count<len(cliff_times):
        time = cliff_times[count]
        while z_angle_times[z_angle_count]<=time:
            z_angle_count += 1
        value = z_angle_values[z_angle_count]['positive_z_angle']
        angle_at_cliff_data.append((time,value))
        count += 1

    turn_on_cliff_angle_data = []
    count_z_angle = 0
    count_angle_at_cliff = 0
    if angle_at_cliff_data!=[]:
        while count_z_angle <len(z_angle_times):
            if count_angle_at_cliff < len(angle_at_cliff_data)-1:
                if z_angle_times[count_z_angle]<angle_at_cliff_data[count_angle_at_cliff+1][0]:
                    value = z_angle_values[count_z_angle]['positive_z_angle'] - angle_at_cliff_data[count_angle_at_cliff][1]
                    turn_on_cliff_angle_data.append((z_angle_times[count_z_angle],value))
                    count_z_angle += 1
                else:
                    count_angle_at_cliff += 1
            else:
                value = z_angle_values[count_z_angle]['positive_z_angle'] - angle_at_cliff_data[count_angle_at_cliff][1]
                turn_on_cliff_angle_data.append((z_angle_times[count_z_angle],value))
                count_z_angle += 1

    turn_on_cliff_angle = signal(turn_on_cliff_angle_data,start=0, end=float('inf'), tag='turn_on_cliff_angle')
    right_rotation = turn_on_cliff_angle.map(lambda val: val['turn_on_cliff_angle']<=-turn_angle, tag='right_rotation')
    left_rotation = turn_on_cliff_angle.map(lambda val: val['turn_on_cliff_angle']>=turn_angle, tag='left_rotation')


    # needed to ignore cliff event at bottom of the ramp
    position = signals.x_pos | signals.y_pos | signals.z_pos
    at_bottom = position.map(lambda val: distanceToLine(bottom_left,bottom_right,(val['x_pos'],val['y_pos']))<0,tag="at_bottom")
    
    cliff_l = (signals.cliff | at_bottom).map(lambda val: val["cliff"]==0.0 and not val['at_bottom'], tag="cliff_l")
    cliff_r = (signals.cliff | at_bottom).map(lambda val: val["cliff"]==2.0 and not val['at_bottom'], tag="cliff_r")
    cliff_c = (signals.cliff | at_bottom).map(lambda val: val["cliff"]==1.0 and not val['at_bottom'], tag="cliff_c")
    no_cliff = signals.cliff.map(lambda val: val["cliff"]<0.0, tag="no_cliff")
    
    data = cliff_l | cliff_r | cliff_c | no_cliff | right_rotation | left_rotation
    
    phi = mtl.parse(f"(cliff_l -> F[0,{rotation_time}](right_rotation | left_rotation))")
    result = phi(data,time=None,quantitative=False)
    verdict = verify(result)

    if verdict!=[]:
        print(f"VIOLATION: Not turning {turn_angle} degrees right on left cliff event at {convert_seconds_to_timpstamp(time0,verdict[0])}")
        violation = True
       

    phi = mtl.parse(f"(cliff_r -> F[0,{rotation_time}](left_rotation | right_rotation))")
    result = phi(data,time=None,quantitative=False)
    verdict = verify(result)

    if verdict!=[]:
        print(f"VIOLATION: Not turning {turn_angle} degrees left on right cliff event at {convert_seconds_to_timpstamp(time0,verdict[0])}")
        violation = True
      

    phi = mtl.parse(f"(cliff_c -> F[0,{rotation_time}](left_rotation | right_rotation))")
    result = phi(data,time=None,quantitative=False)
    verdict = verify(result)

    if verdict!=[]:
        print(f"VIOLATION: Not turning {turn_angle} degrees on center cliff event at {convert_seconds_to_timpstamp(time0,verdict[0])}")
        violation = True
    
    
    if not violation:
        print("No violation.")
    print("***********************************************************\n")

# ====================================================================================================
# ====================================================================================================

# ====================================================================================================
# ===================================== Back off on cliff ============================================
# Robot needs to back off "cliff_margin" distance after cliff event and within 
# "back_off_time" seconds
def back_off_on_cliff(ramp_coordinates, back_off_time, cliff_margin,time0):

    # retrieve ramp coordinates
    bottom_left = ramp_coordinates[0]
    top_left = ramp_coordinates[1]
    top_right = ramp_coordinates[2]
    bottom_right = ramp_coordinates[3]
    cliff_left = ramp_coordinates[4]
    cliff_right = ramp_coordinates[5]

    print("=======================================================")
    print("                   Back off on cliff                   ")
    print(".......................................................")

    # position of robot
    position = signals.x_pos | signals.y_pos | signals.z_pos
    
    away_from_right_cliff = position.map(lambda val: distanceToLine(bottom_right,top_right,(val['x_pos'],val['y_pos']))-cliff_margin>0,tag="away_from_right_cliff")
    away_from_left_cliff = position.map(lambda val: distanceToLine(bottom_left,top_left,(val['x_pos'],val['y_pos']))+cliff_margin<0,tag="away_from_left_cliff")
    away_from_top_cliff = position.map(lambda val: distanceToLine(cliff_left,cliff_right,(val['x_pos'],val['y_pos']))<0,tag="away_from_top_cliff")

    too_right = position.map(lambda val: distanceToLine(bottom_right,top_right,(val['x_pos'],val['y_pos']))-cliff_margin<0 and distanceToLine(bottom_right,top_right,(val['x_pos'],val['y_pos']))>0, tag="too_right")
    too_left = position.map(lambda val: distanceToLine(bottom_left,top_left,(val['x_pos'],val['y_pos']))+cliff_margin>0 and distanceToLine(bottom_left,top_left,(val['x_pos'],val['y_pos']))<0, tag="too_left")
    too_top_edge = position.map(lambda val: distanceToLine(cliff_left,cliff_right,(val['x_pos'],val['y_pos']))+cliff_margin>0 and distanceToLine(cliff_left,cliff_right,(val['x_pos'],val['y_pos']))<0, tag="too_right")

    at_cliff = signals.cliff.map(lambda val: val['cliff']!=-1, tag='at_cliff')
    
    violation = False

    data = away_from_left_cliff | away_from_right_cliff | away_from_top_cliff | too_left | too_right | too_top_edge | at_cliff 

    phi = mtl.parse(f"((at_cliff & too_left) -> F[0,{back_off_time}](away_from_left_cliff))") 
    result = phi(data,time=None,quantitative=False)
    verdict = verify(result)

    if verdict!=[]:
        print(f"VIOLATION: Not backing off of left edge within {back_off_time} seconds at time {convert_seconds_to_timpstamp(time0,verdict[0])}")
        violation = True

    phi = mtl.parse(f"((at_cliff & too_right) -> F[0,{back_off_time}](away_from_right_cliff))") 
    result = phi(data,time=None,quantitative=False)
    verdict = verify(result)

    if verdict!=[]:
        print(f"VIOLATION: Not backing off of right edge with {back_off_time} seconds at time {convert_seconds_to_timpstamp(time0,verdict[0])}")
        violation = True

    phi = mtl.parse(f"((at_cliff & too_top_edge) -> F[0,{back_off_time}](away_from_top_cliff))") 
    result = phi(data,time=None,quantitative=False)
    verdict = verify(result)

    if verdict!=[]:
        print(f"VIOLATION: Not backing off of top edge with {back_off_time} seconds at time {convert_seconds_to_timpstamp(time0,verdict[0])}")
        violation = True

    if not violation:
        print("No violation.")
    print("***********************************************************\n")

# ====================================================================================================
# ====================================================================================================

# ====================================================================================================
# ===================================== Back off on bump  ============================================
# Robot needs to back off "bump_margin" distance after bump event and within 
# "back_off_time" seconds
def back_off_on_bump(obstacles, obstacle_width, robot_width,backing_off_time, backing_off_distance,time0,timeout):

    violation = False

    print("=======================================================")
    print("                   Back off on bump                   ")
    print(".......................................................")

    # position of robot
    position = signals.x_pos | signals.y_pos | signals.z_pos

    # get bump signals
    bump_l = signals.bump.map(lambda val: val["bump"]==0.0, tag="bump_l")
    bump_r = signals.bump.map(lambda val: val["bump"]==2.0, tag="bump_r")
    bump_c = signals.bump.map(lambda val: val["bump"]==1.0, tag="bump_c")
    no_bump = signals.bump.map(lambda val: val["bump"]<0.0, tag="no_bump")

    bump = bump_l | bump_r | bump_c | no_bump 

    for x,y in obstacles:
        # distance to obstacle 
        distance = position.map(lambda val: compute_distance(val['x_pos'],val['y_pos'],x,y),tag=f"distance_to")
        # check if robot is at obstacle x,y 
        at_obstacle = distance.map(lambda val: val[f'distance_to']<obstacle_width+robot_width, tag=f"at_obstacle")
        # distance is larger than backing-off distance
        slope = position.map(lambda val: compute_slope(val['x_pos'],val['y_pos'],x,y),tag=f"slope_to")
        regions = position | slope
        obstacle_left = regions.map(lambda val: compute_left_point(x,y,obstacle_width, 1/(val[f'slope_to']) if val[f'slope_to']!=0 else float('inf')),tag="obstacle_left")
        
        position_to_obs = (position | obstacle_left).map(lambda val: -1 if distanceToLine(val['obstacle_left'],(x,y),(val['x_pos'],val['y_pos']))<0 else 1, tag="position_to_obs")
        far_enough = distance.map(lambda val: val['distance_to']>=backing_off_distance,tag='far_enough')

        # forward_motion = signals.x_twist.map(lambda val: val['x_twist']>0, tag="forward_motion")
        backward_motion = signals.x_twist.map(lambda val: val['x_twist']<0, tag="backward_motion")

        # specification robot backs off distance "backing_off_distance" of obstacle within "timeout" seconds
        data = bump | at_obstacle | far_enough | backward_motion
        phi = mtl.parse(f"(((bump_l | bump_c | bump_r) & at_obstacle) -> ((F[0,{backing_off_time}] far_enough) & (F[0,{backing_off_time}] backward_motion) ))")
        result = phi(data,time=None,quantitative=False)
        verdict = verify(result)

        if verdict!=[]:
            print(f"VIOLATION: Robot did not back off {backing_off_distance} units after bump at time {convert_seconds_to_timpstamp(time0,verdict[0])}")
            violation = True

        
    if not violation:
        print("No violation.")
    print("***********************************************************\n")


# ====================================================================================================
# ====================================================================================================

# ====================================================================================================
# ===================================== Rotate on bump ===============================================
# Monitor checks whether robot turns "turn_angle" degrees after bump event
def rotate_on_bump(rotation_time,turn_angle,time0):

    print("=======================================================")   
    print("                  Rotate on bump                       ")
    print(".......................................................")

    violation = False

    # compute angle at bump signal: z_angle value when bump event is true
    # first make all angles posiitve
    positive_z_angle = signals.z_angle.map (lambda val: 360-(abs(val['z_angle'])%360) if val['z_angle']<0 else val['z_angle']%360,tag='positive_z_angle')
    z_angle_values = positive_z_angle.values()
    z_angle_times = list(positive_z_angle.times())
    at_bump = signals.bump.map(lambda val: val['bump']!=-1,tag='at_bump').filter(lambda val: val['at_bump'])
    bump_times = list(at_bump.times())
    count = 0
    z_angle_count = 0
    angle_at_bump_data = []
    while count<len(bump_times):
        time = bump_times[count]
        while z_angle_times[z_angle_count]<=time:
            z_angle_count += 1
        value = z_angle_values[z_angle_count]['positive_z_angle']
        angle_at_bump_data.append((time,value))
        count += 1

    turn_on_bump_angle_data = []
    count_z_angle = 0
    count_angle_at_bump = 0
    if angle_at_bump_data!=[]:
        while count_z_angle <len(z_angle_times):
            if count_angle_at_bump < len(angle_at_bump_data)-1:
                if z_angle_times[count_z_angle]<angle_at_bump_data[count_angle_at_bump+1][0]:
                    value = z_angle_values[count_z_angle]['positive_z_angle'] - angle_at_bump_data[count_angle_at_bump][1]
                    turn_on_bump_angle_data.append((z_angle_times[count_z_angle],value))
                    count_z_angle += 1
                else:
                    count_angle_at_bump += 1
            else:
                value = z_angle_values[count_z_angle]['positive_z_angle'] - angle_at_bump_data[count_angle_at_bump][1]
                turn_on_bump_angle_data.append((z_angle_times[count_z_angle],value))
                count_z_angle += 1
      

    turn_on_bump_angle = signal(turn_on_bump_angle_data,start=0, end=float('inf'), tag='turn_on_bump_angle')
    right_rotation = turn_on_bump_angle.map(lambda val: val['turn_on_bump_angle']<=-turn_angle, tag='right_rotation')
    left_rotation = turn_on_bump_angle.map(lambda val: val['turn_on_bump_angle']>=turn_angle, tag='left_rotation')

    bump_l = signals.bump.map(lambda val: val["bump"]==0.0, tag="bump_l")
    bump_r = signals.bump.map(lambda val: val["bump"]==2.0, tag="bump_r")
    bump_c = signals.bump.map(lambda val: val["bump"]==1.0, tag="bump_c")
    no_bump = signals.bump.map(lambda val: val["bump"]<0.0, tag="no_bump")    
    
    data = bump_l | bump_r | bump_c | no_bump | right_rotation | left_rotation
    
    phi = mtl.parse(f"(bump_l -> F[0,{rotation_time}](right_rotation))")
    result = phi(data,time=None,quantitative=False)
    verdict = verify(result)


    if verdict!=[]:
        print(f"VIOLATION: Not turning {turn_angle} degrees right on left bump event at {convert_seconds_to_timpstamp(time0,verdict[0])}")
        violation = True
       

    phi = mtl.parse(f"(bump_r -> F[0,{rotation_time}](left_rotation))")
    result = phi(data,time=None,quantitative=False)
    verdict = verify(result)

    if verdict!=[]:
        print(f"VIOLATION: Not turning {turn_angle} degrees left on right bump event at {convert_seconds_to_timpstamp(time0,verdict[0])}")
        violation = True
      

    phi = mtl.parse(f"(bump_c -> F[0,{rotation_time}](left_rotation | right_rotation))")
    result = phi(data,time=None,quantitative=False)
    verdict = verify(result)

    if verdict!=[]:
        print(f"VIOLATION: Not turning {turn_angle} degrees on center bump event at {convert_seconds_to_timpstamp(time0,verdict[0])}")
        violation = True
    
    
    if not violation:
        print("No violation.")
    print("***********************************************************\n")

# ====================================================================================================
# ====================================================================================================    

# ====================================================================================================
# ===================================== Avoid obstacles ==============================================
# After bump, robot needs to go around an obstacle by passing it either on the right or left side 
# whenever possible

def evade_obstacle(obstacles,obstacle_width, robot_width, backing_off_distance,turn_angle, rotation_time,back_off_time, time0,timeout):

    
    bump_l = signals.bump.map(lambda val: val["bump"]==0.0, tag="bump_l")
    bump_r = signals.bump.map(lambda val: val["bump"]==2.0, tag="bump_r")
    bump_c = signals.bump.map(lambda val: val["bump"]==1.0, tag="bump_c")
    no_bump = signals.bump.map(lambda val: val["bump"]<0.0, tag="no_bump")
    bump = signals.bump.map(lambda val: val["bump"]>=0.0, tag="bump")

    bump_data = bump_l | bump_r | bump_c | no_bump | bump

    position = signals.x_pos | signals.y_pos | signals.z_pos


    violation = False

    print("=======================================================")
    print(f"                    Evade obstacle                    ")
    print(".......................................................")
    # iterate over obstacles
    for x,y in obstacles:
       
        #back off on bump
        # distance to obstacle 
        distance = position.map(lambda val: compute_distance(val['x_pos'],val['y_pos'],x,y),tag=f"distance_to")
        # check if robot is at obstacle x,y 
        at_obstacle = distance.map(lambda val: val[f'distance_to']<obstacle_width+robot_width, tag=f"at_obstacle")
        # distance is larger than backing-off distance
        slope = position.map(lambda val: compute_slope(val['x_pos'],val['y_pos'],x,y),tag=f"slope_to")
        regions = position | slope
        obstacle_left = regions.map(lambda val: compute_left_point(x,y,obstacle_width, 1/(val[f'slope_to']) if val[f'slope_to']!=0 else float('inf')),tag="obstacle_left")
        
        position_to_obs = (position | obstacle_left).map(lambda val: -1 if distanceToLine(val['obstacle_left'],(x,y),(val['x_pos'],val['y_pos']))<0 else 1, tag="position_to_obs")
        far_enough = distance.map(lambda val: val['distance_to']>=backing_off_distance,tag='far_enough')

        forward_motion = signals.x_twist.map(lambda val: val['x_twist']>0, tag="forward_motion")
        backward_motion = signals.x_twist.map(lambda val: val['x_twist']<0, tag="backward_motion")

        #rotate on bump
        positive_z_angle = signals.z_angle.map (lambda val: 360-(abs(val['z_angle'])%360) if val['z_angle']<0 else val['z_angle']%360,tag='positive_z_angle')
        z_angle_values = positive_z_angle.values()
        z_angle_times = list(positive_z_angle.times())
        at_bump = signals.bump.map(lambda val: val['bump']!=-1,tag='at_bump').filter(lambda val: val['at_bump'])
        bump_times = list(at_bump.times())
        count = 0
        z_angle_count = 0
        angle_at_bump_data = []
        while count<len(bump_times):
            time = bump_times[count]
            while z_angle_times[z_angle_count]<=time:
                z_angle_count += 1
            value = z_angle_values[z_angle_count]['positive_z_angle']
            angle_at_bump_data.append((time,value))
            count += 1

        turn_on_bump_angle_data = []
        count_z_angle = 0
        count_angle_at_bump = 0
        if angle_at_bump_data!=[]:
            while count_z_angle <len(z_angle_times):
                if count_angle_at_bump < len(angle_at_bump_data)-1:
                    if z_angle_times[count_z_angle]<angle_at_bump_data[count_angle_at_bump+1][0]:
                        value = z_angle_values[count_z_angle]['positive_z_angle'] - angle_at_bump_data[count_angle_at_bump][1]
                        turn_on_bump_angle_data.append((z_angle_times[count_z_angle],value))
                        count_z_angle += 1
                    else:
                        count_angle_at_bump += 1
                else:
                    value = z_angle_values[count_z_angle]['positive_z_angle'] - angle_at_bump_data[count_angle_at_bump][1]
                    turn_on_bump_angle_data.append((z_angle_times[count_z_angle],value))
                    count_z_angle += 1
        

        turn_on_bump_angle = signal(turn_on_bump_angle_data,start=0, end=float('inf'), tag='turn_on_bump_angle')
        right_rotation = turn_on_bump_angle.map(lambda val: val['turn_on_bump_angle']<=-turn_angle, tag='right_rotation')
        left_rotation = turn_on_bump_angle.map(lambda val: val['turn_on_bump_angle']>=turn_angle, tag='left_rotation')


        #evade on center bump
        data = bump_data | backward_motion| far_enough | forward_motion | left_rotation | right_rotation
        phi = mtl.parse(f"(bump_c -> F[0,{back_off_time}](backward_motion U (far_enough &  F[0,{rotation_time}] (left_rotation & F[0,{rotation_time}] (forward_motion & F[0,{back_off_time}](bump | (right_rotation & F[0,{rotation_time}] (forward_motion U (bump | far_enough)))))))))")
        result_c_lr = phi(data,time=None,quantitative=False)
        verdict_c_lr = verify(result_c_lr)

        phi = mtl.parse(f"(bump_c -> F[0,{back_off_time}](backward_motion U (far_enough &  F[0,{rotation_time}] (right_rotation & F[0,{rotation_time}] (forward_motion & F[0,{back_off_time}](bump | (left_rotation & F[0,{rotation_time}] (forward_motion U (bump | far_enough)))))))))")
        result_c_rl = phi(data,time=None,quantitative=False)

        verdict_c_rl = verify(result_c_rl)

        if (verdict_c_lr!=[] and verdict_c_rl!=[]):
            print(f"VIOLATION: Not avoiding obstacle on center bump event at {convert_seconds_to_timpstamp(time0,verdict_c_lr[0])}")
            violation = True

        phi = mtl.parse(f"(bump_l -> F[0,{back_off_time}](backward_motion U (far_enough &  F[0,{rotation_time}] (right_rotation & F[0,{rotation_time}] (forward_motion & F[0,{back_off_time}](bump | (left_rotation & F[0,{rotation_time}] (forward_motion U (bump | far_enough)))))))))")
        result_l = phi(data,time=None,quantitative=False)

        verdict_l = verify(result_l)

        if (verdict_l!=[]):
            print(f"VIOLATION: Not avoiding obstacle on center bump event at {convert_seconds_to_timpstamp(time0,verdict_l[0])}")
            violation = True

        phi = mtl.parse(f"(bump_l -> F[0,{back_off_time}](backward_motion U (far_enough &  F[0,{rotation_time}] (left_rotation & F[0,{rotation_time}] (forward_motion & F[0,{back_off_time}](bump | (right_rotation & F[0,{rotation_time}] (forward_motion U (bump | far_enough)))))))))")
        result_r = phi(data,time=None,quantitative=False)

        verdict_r = verify(result_r)

        if (verdict_r!=[]):
            print(f"VIOLATION: Not avoiding obstacle on center bump event at {convert_seconds_to_timpstamp(time0,verdict_r[0])}")
            violation = True
   
    if not violation:
        print("No violation.")
    print("***********************************************************\n")
            
# ====================================================================================================
# ====================================================================================================
# 
#   
# =============================== End of monitor definitoins ======================================= #
# ================================================================================================== #
######################################################################################################
