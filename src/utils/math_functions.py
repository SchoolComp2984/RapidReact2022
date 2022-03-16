# STATIC

# Joystick power interpolation
def interp(joy):
      ary = [ \
      [-1,-12],\
      [-.85,-6],\
      [-.5,-4],\
      [-.25,0],\
      [.25,0],\
      [.5,4],\
      [.85,6],\
      [1,12]]
      return interp_Array(joy, ary)
      
def interp_Array(joy, ary):
      if joy <= ary[0][0]:
            return ary[0][1]
      if joy >= ary[len(ary) - 1][0]: 
            return ary[len(ary) - 1][1]
      for i in range(len(ary) - 1):
            if ((joy>=ary[i+0][0]) and (joy<=ary[i+1][0])): 
                  return (joy-ary[i+0][0])*(ary[i+1][1]-ary[i+0][1])/(ary[i+1][0]-ary[i+0][0])+ary[i+0][1]
      return 0

def shootInterp(limelight_angle):
       # shooting rpm for different distances
       # make this better
       array = [ \
       [0,15000],\
       [9.8,7700],\
       [12.5,7500],\
       [15,7000],\
       [18,6000]]
       return interp_Array(limelight_angle, array)

def clamp(num, min_value, max_value):
      return max(min(num, max_value), min_value)

def good_interp(func_degree, min_input, max_input, min_output, max_output, value):

    #ACCORDING TO MY DESMOS EXPERIMENT
    #min_input = a
    #min_output = b
    #max_input = c
    #max_output = d
    # d * (x/c - a) ** 2 + b
    # d * (x/c - a) + b

    # VERTEX EQUATION FOR QUADRATIC
    #(h, k)
    #a * (x - h)^2 + k

    new_value = (max_output - min_output) * (value * (1 + min_input) / max_input - min_input) ** func_degree + min_output
    return clamp(new_value, min_output, max_output)

def good_joystick_interp(joy_value, deadzone, degree):
    #joy value -1 to 1
    #deadzone 0 to 1
    new_value = 0
    if abs(joy_value) > deadzone:
        new_value = good_interp(degree, deadzone, 1, 0, 1, abs(joy_value))
        if joy_value < 0:
            new_value *= -1
        else:
            pass
    return new_value