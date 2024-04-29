import sys, time


class PID:
    def __init__(self, Kp=1.0, Ki=0.25, Kd=0.75, sample_time=0):  # TODO changed kd to 0.75 revert back to 0.5 original value
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.sample_time = sample_time
        self.output = self.target = 0
        self.windup_guard = 20
        self.integral_error = self.previous_error = 0
        self.p_term = self.i_term = self.d_term = 0
        self.previous_time = self.current_time = time.time()

    def recalculate(self, feedback):
        """output = Kp*e(t) + Ki*Integral(0-t, e(t)dt) + Kd*de/dt """
        self.current_time = time.time()
        delta_time = self.current_time - self.previous_time

        error = self.target - feedback
        delta_error = error - self.previous_error

        if delta_time >= self.sample_time:
            self.p_term = self.Kp * error
            self.i_term += error * delta_time
            self.d_term = delta_error/delta_time if delta_time > 0 else 0  # avoid zero div

            if self.i_term < -self.windup_guard:
                self.i_term = -self.windup_guard
            elif self.i_term > self.windup_guard:
                self.i_term = self.windup_guard

            self.previous_time = self.current_time
            self.previous_error = error
            self.output = self.p_term + self.Ki*self.i_term + self.Kd*self.d_term


if __name__ == '__main__':
    pid = PID(Kp=0.2, Ki=0, Kd=0)
    for f in range(-100, 100):
        pid.target = 50
        pid.recalculate(f)
        print(F"{pid.output:.2f}")
