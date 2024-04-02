import math
ppl=100 #pixel per inch
known_distance=10000 #know distance in centimeters, it will give by the camera package

def pixel_to_CM_conversion(pixel,ppl):
    centimeter=(pixel*2.54)/(ppl*100)
    return centimeter
def find_angle(d1,d2):
    width= d1-d2
    width_in_cm=pixel_to_CM_conversion(width,ppl)
    angle_value=width_in_cm/known_distance
    result_radians = math.atan(angle_value)
    angle_degree= math.degrees(result_radians)
    return angle_degree
def moving_angle(x,y,x_previous,y_previous):
    x_rotation= find_angle(x,x_previous)
    y_rotation =find_angle(y,y_previous)
    return (x_rotation, y_rotation)

# moving angles in degree. it could be minus value
    def moving_angle(self,x,y,x_previous,y_previous):
        max_length=1.608
        for i in range(0,max_length,1):
            if faces:
                x_rotation= self.find_angle(x,x_previous)
                y_rotation =self.find_angle(y,y_previous)
                y_lift=i
                return (x_rotation, y_rotation,y_lift)
            else:
                continue


a=10
def myname(b):
    c=a+b