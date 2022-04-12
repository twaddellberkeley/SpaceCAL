

# This node takes input from the jetson Nano display and performs the required command.
# There are a total of 9 motors and 5 projectors that need to be control. 

# Actions that need to be performed  (4 Motors )
##### Motors Level Control #######
# turn on the motors 
# set the mottors home
# set the position of the motors
# pause current action
# kill the motors

##### Motors Rotation Control #######  (5 Motors)
# Turn Motors on
# Set motor velocity
# Start motor rotation
# stop motor rotation
# kill motors 


##### Projector #######   (5 Projectors)
# Create a video queue
# Add video to queue
# Remove video from queue
# Rearrange queue
# Turn projector on 
# Turn projector off
# Turn LED on 
# Turn LED off
# Start playing video queue



def main():
    print('Hi from jetson_main_logic.')


if __name__ == '__main__':
    main()
