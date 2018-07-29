import numpy as np
from scipy.spatial import distance


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        print('mode:', Rover.mode, Rover.stuck)
        if Rover.mode == 'forward':

            # Check if Rover got stuck
            if distance.euclidean(Rover.pos, Rover.prev_pos) < 0.01:
                Rover.stuck += 1
            else:
                Rover.stuck = 0
            Rover.prev_pos = Rover.pos
            if Rover.stuck > 100:
                Rover.mode = 'stuck'

            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)

                # Check for obstacles in front of Rover
                obs_distances = [t[0] for t in Rover.obs_polar if \
                                    t[0] < 20 and -0.3 < t[1] and t[1] < 0.05]
                if len(obs_distances) > 10:
                    Rover.stuck = 40
                    Rover.mode = 'stuck'

            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

            # Is there a rock nearby?
            if Rover.rock_angles is not None and len(Rover.rock_angles) > 1:
                if np.min(Rover.rock_dists) < 50 and np.mean(Rover.rock_angles) > 0:
                    Rover.mode = 'rock'

            # Park Rover if goals have been reached
            if (Rover.mapped > 97 or Rover.samples_collected > 5) \
                    and distance.euclidean(Rover.init_pos, Rover.pos) < 5:
                Rover.finish_time = Rover.total_time
                Rover.mode = 'parked'

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
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'

        # Turn Rover if it got stuck
        elif Rover.mode in {'stuck', 'stuck_near_rock'}:
            # Continue navigating after picking up a sample
            if Rover.picking_up:
                # Set position outside the worldmap when there are no rocks nearby
                Rover.rock_pos = (-10, -10)
                Rover.stuck = 0
                Rover.mode = 'forward'
            # Stop Rover if it still moving near a rock
            elif (Rover.stuck > 100 and Rover.vel > 0.2) or Rover.near_sample:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
            else:
                Rover.throttle = 0
                # Release the brake to allow turning
                Rover.brake = 0
                Rover.steer = -15
                if Rover.stuck <= 0:
                    if Rover.mode == 'stuck':
                        Rover.mode = 'forward'
                    elif Rover.mode == 'stuck_near_rock':
                        Rover.mode = 'rock'
                Rover.stuck -= 10

        # Try to pick up the rock
        elif Rover.mode == 'rock':
            # Slow down
            if Rover.vel < Rover.max_vel / 2:
                Rover.throttle = Rover.throttle_set / 2
            else:
                Rover.throttle = 0
            Rover.brake = 0
            # Set steering towards the rock
            if len(Rover.rock_angles) > 0:
                # Keep the rock on the left side
                Rover.rock_angles -= 0.3
                Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/np.pi), -15, 15)
                # Memorize position of the rock
                min_rock_dist = np.min(Rover.rock_dists)
                average_rock_angle = np.mean(Rover.rock_angles)
                Rover.rock_pos = (Rover.pos[0] + min_rock_dist * np.cos(average_rock_angle) / 10,
                                  Rover.pos[1] + min_rock_dist * np.sin(average_rock_angle) / 10)
            else:
                # Turn left if visibility of the rock is lost
                Rover.steer = 15

            if Rover.rock_pos != None:
                print('distance to the rock =', distance.euclidean(Rover.pos, Rover.rock_pos))
            # Turn round if Rover is close to the rock enough
            if Rover.rock_pos != None and distance.euclidean(Rover.pos, Rover.rock_pos) < 1:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.stuck = 1700
                Rover.mode = 'stuck_near_rock'
            # Treat cases with wrong coordinates of the rock
            elif Rover.rock_pos != None and distance.euclidean(Rover.pos, Rover.rock_pos) < 4:
                if Rover.stuck > 90:
                    Rover.throttle = 0
                    Rover.brake = Rover.brake_set
                    Rover.stuck = 1700
                    Rover.mode = 'stuck_near_rock'
                Rover.stuck += 2

            # Check for obstacles in front of Rover
            obs_distances = [t[0] for t in Rover.obs_polar if \
                                t[0] < 10 and -0.3 < t[1] and t[1] < 0.05]
            if len(obs_distances) > 10:
                Rover.stuck = 40
                Rover.mode = 'stuck_near_rock'

            # Escape if Rover got stuck
            if distance.euclidean(Rover.pos, Rover.prev_pos) < 0.01:
                Rover.stuck += 1
            else:
                Rover.stuck = 0
            Rover.prev_pos = Rover.pos
            if Rover.stuck > 100:
                Rover.mode = 'stuck'

            # Continue navigating after picking up the rock
            if Rover.picking_up:
                Rover.rock_pos = (-10, -10)
                Rover.mode = 'forward'

        # Hold the brake after finishing the mission
        elif Rover.mode == 'parked':
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            Rover.steer = 0

    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    
    return Rover
