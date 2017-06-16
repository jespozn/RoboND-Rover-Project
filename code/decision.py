import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        Rover.aux2 = Rover.delta_yaw
        #if Rover.rock_detected:
        #    Rover.mode = 'pick'
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if (len(Rover.nav_angles) >= Rover.stop_forward) & (Rover.front_nav_dist > 2):  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                if Rover.memory is not None:
                    if (Rover.memory < 5) | ((Rover.total_time - Rover.forget_time) < 20):
                        Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    else:
                        minisx, minisy = search_spot(Rover)
                        if minisx is not None:
                            diff = minisx[0] - Rover.pos[0], minisy[0] - Rover.pos[1]
                            if diff[0] >=0 :
                                yaw = np.tan(diff[1]/diff[0])*180/np.pi
                            else:
                                yaw = np.tan(diff[1]/diff[0])*180/np.pi + 180
                            Rover.steer = np.clip(yaw, -15, 15)
                        else:
                            Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                        #Rover.mode = 'turn'
                else:
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif (len(Rover.nav_angles) < Rover.stop_forward) | (Rover.vel<0.2):
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                #if ((len(Rover.nav_angles) < Rover.go_forward)) | (Rover.front_nav_dist < 1) :
                if (Rover.front_nav_dist < 2) :
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                #elif len(Rover.nav_angles) >= Rover.go_forward:
                else:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
        # turn right
        elif Rover.mode == 'turn':
            # If we're still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                if Rover.start_yaw is not None:
                    delta = Rover.start_yaw - Rover.yaw
                    if delta < 0:
                        Rover.delta_yaw = delta + 360
                    else:
                        Rover.delta_yaw = delta
                if (Rover.memory > 4) & (Rover.delta_yaw < 360):
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -10
                    if Rover.delta_yaw == 0:
                        Rover.start_yaw = Rover.yaw
                        
                else:
                    Rover.forget_time = Rover.total_time
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
                    Rover.delta_yaw = 0
                    
        elif Rover.mode == 'pick':
            # If we're still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                if ~np.isnan(Rover.rock_pos[1]):
                    if np.absolute(Rover.rock_pos[1])*180/np.pi > 2:
                        Rover.brake = 0
                        Rover.steer = Rover.rock_pos[1]*180/np.pi
                    elif Rover.rock_pos[0] > 1:
                        Rover.steer = 0
                        Rover.throttle = Rover.throttle_set
                    else:
                        Rover.throttle = 0
                        Rover.brake = Rover.brake_set
                        Rover.mode = 'forward'
                else:
                    Rover.brake = 0
                    Rover.steer = 0
                    Rover.throttle = Rover.throttle_set
                    Rover.mode = 'forward'
            
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
        Rover.rock_detected = False
    
    return Rover

# Search for an unknown spot around the rover
def search_spot(Rover):
    offset = 5
    x, y = round(Rover.pos[0]), round(Rover.pos[1])
    min = np.amin(Rover.worldmap[y-offset:y+offset, x-offset:x+offset, 2])
    Rover.aux2 = np.amax(Rover.worldmap[y-offset:y+offset, x-offset:x+offset, 2])
    if min < 200:
        minis = Rover.worldmap[:,:,2] == min
        return minis[:,1], minis[:,0]
    else:
        minis = None
        return minis, minis
    
    
    