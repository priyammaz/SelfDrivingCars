#!/usr/bin/python

import time
import numpy as np


class PID:
    def __init__(
        self,
        Kp=0.0,
        Ki=0.0,
        Kd=0.0,
        set_point=0.0,
        sample_time=0.01,
        out_limits=(None, None),
    ):

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0

        self.set_point = set_point

        self.sample_time = sample_time

        self.out_limits_min = out_limits[0]
        self.out_limits_max = out_limits[1]

        self.last_err = 0.0

        self.last_time = time.time()

        self.output = 0.0

    def update(self, feedback_val):
        """Compute PID control value based on feedback_val.
        """

        ### CALCULATE TIME SINCE LAST STEP ###
        curr_time = time.time()
        diff_time = curr_time - self.last_time

        ### UPDATE THE P, I, D Terms ###
        self.p_term = feedback_val
        self.i_term += feedback_val
        self.d_term = (feedback_val - self.last_err)*self.sample_time#diff_time

        self.last_err = feedback_val
        self.last_time = curr_time
        
        output_value = self.p_term * self.Kp + self.i_term * self.Ki + self.d_term * self.Kd
        output_value = round(np.clip(output_value, self.out_limits_min, self.out_limits_max), 3)

        return output_value



        

    def __call__(self, feeback_val):
        return self.update(feeback_val)
