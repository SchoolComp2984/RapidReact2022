class PID:
      def set_pid(self, p, i, d, val):
            self.p = p
            self.i = i
            self.d = d
            self.previous_input = val
            self.integral = 0

      def steer_pid(self, error):
            power = error * self.p
            if self.integral > 0 and (error * self.i < 0):
                  self.integral = 0
            if (self.integral < 0) and (error * self.i > 0):
                  self.integral = 0
            self.integral += error*self.i
            if (-20 < error) and (error < 20):
                  power += self.integral
            else: 
                  self.integral=0
            power += (error - self.previous_input) * self.d
            self.previous_input = error
            return power
            
      def shoot_pid(self, error):
            power = error * self.p
            if abs(power) > 1:
                  self.integral = 0
            self.integral += error*self.i
            self.integral *= 0.99
            #if (-20 < error) and (error < 20):
            power += self.integral
            #else: 
            #self.integral=0
            power += (error - self.previous_input) * self.d
            self.previous_input = error
            return power
            