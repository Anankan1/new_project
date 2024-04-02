
def moving_direction(x,y,x_previous,y_previous):
    if x>x_previous:
        if y == y_previous:
            print("image moving left to right")
        elif y> y_previous:
            print("image moving to right bottom corner")
        else:
            print ("image moving to right top corner")
    elif x==x_previous:
        if y == y_previous:
            print("image doesnot move")
        elif y> y_previous:
            print("image moving to straight bottom")
        else:
            print ("image moving to upward")
    else:
        if y == y_previous:
            print("image moving right to left")
        elif y> y_previous:
            print("image moving to left bottom corner")
        else:
            print ("image moving to left top corner")


