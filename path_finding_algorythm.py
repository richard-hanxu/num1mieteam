import math


#for B in top right corner
maze1 = [ ["r","r","r","d","-","d","-","g"],  
          ["u","u","-","r","r","r","r","u"],
          ["d","-","d","-","-","u","-","u"],
          ["r","r","r","r","r","r","-","u"]
         ]



def determine_route(x,y,angle):
    ''' 
    
    inputs from particle filter code -> 
        - x position (in)
        - y position (in)
        - angle (degrees from the bottom vertical, CCW is positive)
            
    output -> 
    
        - angle to rotate
        - forward distance to travel
    
    '''
    
    current_block_x = math.ceil(x/12)  #are numbers 1,2,3,4,5,6,7,8
    current_block_y = math.ceil(y/12)
    
    print(current_block_x)
    print(current_block_y)
    
    maze_1_current_position = maze1[current_block_y-1][current_block_x-1]
    
    if maze_1_current_position == "-":
        return "ERROR! Localization Failed"
    
    elif maze_1_current_position == "g":
        return "Success!" 
    
    elif maze_1_current_position == "u":
        current_block_y -= 1
        
    elif maze_1_current_position == "d":
        current_block_y += 1
        
    elif maze_1_current_position == "r":
        current_block_x += 1

    elif maze_1_current_position == "l":
        current_block_x -= 1

    target_y = current_block_y*12 -6        # values in inches
    target_x = current_block_x*12 -6
    
    print("target_x: ", target_x)
    print("target_y: ", target_y)
    
    
    distance_to_travel = ((target_x - x)**2 + (target_y - y)**2)**(1/2)
   
    target_angle_to_horizontal = math.degrees(math.atan2(abs(target_y - y) , abs(target_x-x)))
    
    # gets the value of the target angle relative to richards refference
    
    if target_x<=x and target_y>=y:                                       # if in Q3
        angle_to_vertical = target_angle_to_horizontal + 270
        print('Q3')
    
    elif target_x<=x and target_y<=y:                                      # if in Q2
        angle_to_vertical =  270 - target_angle_to_horizontal
        print('Q2')
        
    elif target_x>=x and target_y<=y:                                       # if in Q1
        angle_to_vertical =  90 + target_angle_to_horizontal
        print('Q1')
        
    elif target_x>=x and target_y>=y:                                      # if in Q4
        angle_to_vertical =  90 - target_angle_to_horizontal
        print('Q4')
    
    #Determines angle magnitude to rotate
    angle_to_move = angle_to_vertical - angle
    
    
    # Determine's angle direction to rotate
    if angle_to_vertical > angle:
        rotation_direction = "CCW"
    
    elif angle_to_vertical < angle:
        rotation_direction = "CW"
    
    else:
        rotation_direction = False
        
    return (round(distance_to_travel,2), round(angle_to_move,2), rotation_direction)


print(determine_route(59,44,0))