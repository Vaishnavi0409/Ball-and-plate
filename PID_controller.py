class PIDController:
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0, setpoint=0.0,
                 output_limits=(-90, 90), integral_limits=(-100, 100)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint

        self.output_limits = output_limits      # (min, max) for output
        self.integral_limits = integral_limits  # (min, max) for integral term

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_measurement = None  # needed for derivative-on-measurement

    def update(self, measurement, dt=1.0):
        error = self.setpoint - measurement

        # --- Proportional ---
        P = self.Kp * error

        # --- Integral with clamping (anti-windup) ---
        self.integral += error * dt
        self.integral = max(min(self.integral, self.integral_limits[1]),
                            self.integral_limits[0])
        I = self.Ki * self.integral

        # --- Derivative on measurement ---
        if self.prev_measurement is not None and dt > 0:
            derivative = -(measurement - self.prev_measurement) / dt
        else:
            derivative = 0.0
        D = self.Kd * derivative

        # --- PID output ---
        control = P + I + D

        # --- Output limiting ---
        min_out, max_out = self.output_limits
        control = max(min(control, max_out), min_out)

        # --- Save states ---
        self.prev_error = error
        self.prev_measurement = measurement

        return control

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_measurement = None
