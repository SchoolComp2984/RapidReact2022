# Static
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
         if joy <= ary[0][0]:
               return ary[0][1]
         if joy >= ary[len(ary) - 1][0]: 
               return ary[len(ary) - 1][1]
         for i in range(len(ary) - 1):
               if ((joy>=ary[i+0][0]) and (joy<=ary[i+1][0])): 
                  return (joy-ary[i+0][0])*(ary[i+1][1]-ary[i+0][1])/(ary[i+1][0]-ary[i+0][0])+ary[i+0][1]
         return 0
         """ Old power interpolation
         ary = [ \
         [-1,-12],\
         [-.75,-1],\
         [-.5,-.2],\
         [-.25,0],\
         [.25,0],\
         [.5,.2],\
         [.75,1],\
         [1,12]]
         """
def clamp(num, min_value, max_value):
   return max(min(num, max_value), min_value)