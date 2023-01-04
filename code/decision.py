import numpy as np


# Set Forward



def set_forward(Rover):
    Rover.mode = 'forward'

    # Check the extent of navigable terrain
    if len(Rover.nav_angles) >= Rover.stop_forward:
        Rover.brake = 0
        # If mode is forward, navigable terrain looks good
        # and velocity is below max, then throttle
        if Rover.vel < Rover.max_vel:
            # Set throttle value to throttle setting
            Rover.throttle = Rover.throttle_set
        else:  # Else coast
            Rover.throttle = 0

        # Set steering to average angle clipped to the range +/- 15
        Rover.steer = np.clip(
            np.mean(Rover.nav_angles * 180/np.pi), -15, 15)

    # If there's a lack of navigable terrain pixels then go to 'stop' mode
    elif len(Rover.nav_angles) < Rover.stop_forward:
        # Set mode to "stop" and hit the brakes!
        Rover.throttle = 0
        # Set brake to stored brake value
        Rover.steer = 0
        Rover.mode = 'stop'
        return Rover



# Set Reverse


def set_reverse(Rover):
    """Called when the Rover.stuck_count reaches values
    set within the Forward & going_to_rock mode.
    Additionally used to backup the Rover after picking
    up a rock."""
    Rover.mode = 'reverse'

    Rover.brake = 0
    Rover.throttle = -0.6

    # Prevents the Rover from getting stuck on top of
    # or within rocks.
    if Rover.vel > -0.02 and Rover.vel <= 0.02:
        Rover.stuck_in_stuck_counter += 1
    else:
        if Rover.stuck_in_stuck_counter >= 0.5:
            Rover.stuck_in_stuck_counter -= 0.5

    # Required to prevent a Numpy RuntimeWarning
    # If the rover is in front of an obstacle
    # np.mean() cannot comput len(Rover.nav_angles)
    if len(Rover.nav_angles) < Rover.go_forward:
        Rover.steer = 0
    elif Rover.stuck_in_stuck_counter >= 25:
        Rover.steer = 15
        Rover.throttle = 0
    else:
        # Steer in the opposite direction as direction
        # that lead to getting stuck
        Rover.steer = -(np.clip(
            np.mean(Rover.nav_angles * 180/np.pi), -15, 15))

    if Rover.stuck_count >= 0.5:
        Rover.stuck_count -= 0.5
    else:
        Rover.throttle = Rover.throttle_set
        Rover.stuck_count = 0
        Rover.stuck_in_stuck_counter = 0
        Rover.mode = 'forward'
        return Rover



# Stopping



def set_stop(Rover):
    """Called when the len(Rover.nav_angles) < Rover.go_forward
    Transitions to Forward mode when untrue."""
    Rover.mode = 'stop'

    # If we're in stop mode but still moving keep braking
    if Rover.vel > 0.2:
        Rover.throttle = 0
        Rover.brake = Rover.brake_set
        Rover.steer = 0

    # If we're not moving (vel < 0.2) then do something else
    elif Rover.vel <= 0.2:
        # Now we're stopped and we have vision data to see if there's a path forward
        if len(Rover.nav_angles) < Rover.go_forward:
            Rover.throttle = 0
            # Release the brake to allow turning
            Rover.brake = 0
            Rover.steer = -15

        # If we're stopped but see sufficient navigable terrain in front then go!
        if len(Rover.nav_angles) >= Rover.go_forward:
            # Set throttle back to stored value
            Rover.throttle = Rover.throttle_set
            # Release the brake
            Rover.brake = 0
            # Set steer to mean angle
            Rover.steer = np.clip(
                np.mean(Rover.nav_angles * 180/np.pi), -17, 17)
            Rover.mode = 'forward'
            return Rover


# Cut Out


def cut_out(Rover):
    Rover.mode = 'cut_out'

    # Prevent Rover from turning into an obstacle
    # when turning out of a circle
    if len(Rover.nav_angles) < Rover.stop_forward:
        Rover.mode = 'stop'
        return Rover

    # Rover.steer_cuts is a list of negative & positive
    # values. The list is used to give the Rover a
    # randomized directional choice. Prevents the Rover
    # from missing areas of the map & infinite circles.
    if Rover.steer_cut_index >= len(Rover.steer_cuts):
        Rover.steer_cut_index = 0
    Rover.steer = Rover.steer_cuts[Rover.steer_cut_index]

    if Rover.cut_out_count >= 1.0:
        Rover.cut_out_count -= 1.0
    else:
        Rover.cut_out_count = 0
        Rover.steer_cut_index += 1
        Rover.mode = 'forward'
        return Rover


# Going to a Rock


def going_to_rock(Rover):
    Rover.mode = 'going_to_rock'

    # Pointing steer angles to the closest Rock
    Rover.steer = np.clip(
        np.mean(Rover.rock_angle * 180/np.pi), -15, 15)

    # Slow Down & Prevent backwards movement
    if Rover.vel > 1 or Rover.vel < -0.03:
        Rover.brake = 1
    else:
        Rover.brake = 0

    # Setting a low max velocity to prevent
    # hard stops when near a sample
    if Rover.vel < 0.8:
        Rover.throttle = 0.2
    else:
        Rover.throttle = 0

    # If the Rover is close enough to pick-up
    if Rover.near_sample == 1:
        Rover.brake = 10
        Rover.mode = 'picking_rock'
        return Rover


# Picking up a Rock


def picking_rock(Rover):

    Rover.mode = 'picking_rock'
    
    Rover.steer = 0

    if not Rover.picking_up:
        Rover.send_pickup = True
    else:
        Rover.send_pickup = False

    if Rover.near_sample == 0:
        Rover.stuck_count = 30
        Rover.mode = 'forward'
        Rover.rocks_collected +=1  
        return Rover



def GoingHome(Rover, pos):
    #We only want to crawl to the target
    if (Rover.vel >= 0.8):
        # Rover.brake = 10
        # Rover.throttle = 0
        # Rover.steer = 
        set_stop(Rover)
    else:

        Rover.brake = 0
        #Find the angle of attack
        target_yaw = np.arctan2(int(pos[1]) - int(Rover.pos[1]),int(pos[0]) - int(Rover.pos[0]))

        #If we have a negative angle, increase it before we figure it out
        if (target_yaw < 0):
           target_yaw += np.pi * 2
        #Convert to degrees
        target_yaw = target_yaw * 180 / np.pi
        
        #If the rover is to the right of the angle, turn left
        if (round(Rover.yaw + 360,0) < round(target_yaw + 360 - Rover.target_yaw_diff,0)):
           Rover.steer = Rover.goal_turning_speed

        #If the rover is to the left of the angle, turn right
        elif (round(Rover.yaw + 360,0) > round(target_yaw + 360 + Rover.target_yaw_diff,0)):
           Rover.steer = Rover.goal_turning_speed * -1

        #Else we are looking at it, go get it.
        else:
           Rover.brake =0
           Rover.throttle = 0.3
           Rover.steer =0

    return Rover


def completestop(Rover):
    Rover.brake = 10
    Rover.throttle = 0
    Rover.steer = 0
    while True :
        {

        }







# THE MAIN() FUNCTION



def decision_step(Rover):


    # Verify if Rover has vision data
    if Rover.nav_angles is not None:
        print(f"Collected Rocks = {Rover.rocks_collected}")
        if Rover.rocks_collected >= 5:
            Rover.distance_to_home_previous = Rover.distance_to_home 
            Rover.distance_to_home = np.sqrt((Rover.pos[0]-Rover.startpos[0])**2 + (Rover.pos[1] - Rover.startpos[1])**2)  
            if(Rover.distance_to_home_previous < Rover.distance_to_home):
                Rover.brake = 10
                Rover.throttle = 0
                Rover.steer = 0
                Rover.yaw +=90
                Rover.mode = 'forward'
                return Rover
            if(Rover.distance_to_home<=1):
                print(f"Distance is less than 1")
                completestop(Rover)
            elif(Rover.distance_to_home<=8):
                print(f"Distance is less than 8")
                GoingHome(Rover,Rover.startpos)
            else :
                print(f"Distance is = {Rover.distance_to_home_previous}")
        # Check for Rover.mode status
        else :
            pass

        # Reverse
        if Rover.mode == 'reverse':
            set_reverse(Rover)

        # Stop

        elif Rover.mode == 'stop':
            if Rover.stuck_count >= 50:
                Rover.mode = 'reverse'
                return Rover
            set_stop(Rover)

        # Forward
        elif Rover.mode == 'forward':
            # Setting brake to 0 first
            Rover.brake = 0

            # Conditions for Cutting Out:
            if Rover.vel >= 1.3:
                # Call the cut_out() function
                if Rover.cut_out_count >= 50.0:
                    Rover.mode = 'cut_out'
                    return Rover
                # If going Forward, Vel > 1.8 AND Steering
                # at -15 or 15: add to the counter
                elif Rover.steer == 15.0 or Rover.steer == -15.0:
                    Rover.cut_out_count += 1
                # Else: subtract from counter
                else:
                    if Rover.cut_out_count >= 1:
                        Rover.cut_out_count -= 1

            # Checking if Rover is Stuck:
            if Rover.throttle == Rover.throttle_set:
                # If Rover is stuck
                if Rover.stuck_count >= 55.0:
                    Rover.mode = 'reverse'
                    return Rover
                # If going Forward, with Vel in range(-0.2, 0.06)
                # AND in full throttle: add to the counter
                elif Rover.vel < 0.06 and Rover.vel > -0.2:
                    Rover.stuck_count += 1
                # Else: subtract from counter
                else:
                    if Rover.stuck_count >= 0.5:
                        Rover.stuck_count -= 0.5

            # If not stuck OR about to cut out, go forward
            set_forward(Rover)

        # Going towards Rock
        elif Rover.mode == 'going_to_rock':
            # Checking if Rover is Stuck:
            if Rover.throttle == 0.2 and Rover.near_sample == 0:
                if Rover.stuck_count >= 60.0:
                    Rover.mode = 'reverse'
                    return Rover
                elif Rover.vel < 0.05 and Rover.vel > -0.2:
                    Rover.stuck_count += 1
                else:
                    if Rover.stuck_count >= 0.5:
                        Rover.stuck_count -= 0.5
            going_to_rock(Rover)

        # Picking Rock
        elif Rover.mode == 'picking_rock':
            picking_rock(Rover)

        # Cut Out to prevent stucking in circle

        elif Rover.mode == 'cut_out':
            cut_out(Rover)

        else:
            print("ERROR: Unknown state")

    # Just to make the rover do something
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    if Rover.near_sample == 1 and Rover.vel == 0 and not Rover.picking_up:
        Rover.stuck_count = 0
        Rover.send_pickup = True
    else:
        Rover.send_pickup = False

    return Rover
