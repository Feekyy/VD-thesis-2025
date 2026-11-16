#!/usr/bin/env python3

#Discrete PID controller, one sample = one camera frame
#   output = kp * e + ki * sum(e) + kd * filtered(e - e_prev)
class PID:
    def __init__(self, kp, ki=0.0, kd=0.0, i_limit=0.0, d_alpha=0.5):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        #Anti-windup: the accumulated error is kept between -i_limit and +i_limit (0 = no limit)
        self.i_limit = i_limit

        #Smoothing of the derivative (1.0 = raw difference), because the detected center jitters a few pixels
        self.d_alpha = d_alpha
        self.reset()

    #Forgets the history (new target, target lost or pass-through)
    def reset(self):
        self.integral = 0.0
        self.prev_error = None
        self.derivative = 0.0

    #Calculates the control output from the current error
    def update(self, error):
        self.integral += error
        if self.i_limit > 0:
            self.integral = max(-self.i_limit, min(self.i_limit, self.integral))

        #No derivative on the first sample after a reset, there is no previous error yet
        if self.prev_error is not None:
            raw_derivative = error - self.prev_error
            self.derivative = self.d_alpha * raw_derivative + (1.0 - self.d_alpha) * self.derivative
        self.prev_error = error

        return self.kp * error + self.ki * self.integral + self.kd * self.derivative
