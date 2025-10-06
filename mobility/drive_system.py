import os, sys, time, math, threading
from typing import Tuple, Optional
from DFRobot_RaspberryPi_DC_Motor import THIS_BOARD_TYPE, DFRobot_DC_Motor_IIC as Board


class DriveSystem:
    """
    A class to control a differential drive robot using the DFRobot DC Motor Driver.
    Manages motor initialization, velocity control, and a PID feedback loop for
    precise speed regulation based on encoder feedback.
    """
    def __init__(self,
                 i2c_bus: int = 1, 
                 i2c_addr: int = 0x10,
                 wheel_radius_m: float = 0.070,
                 track_width_m: float = 0.155,
                 control_hz: float = 25.0,
                 kp: float = 0.8,
                 ki: float = 0.2,
                 kd: float = 0.0,
                 invert_left: bool = False,
                 invert_right: bool = False,
                 max_angular_rps: float = 1.0,
                 encoder_reduction_ratio: int = 100):
        """
        Initializes the DriveSystem.

        Args:
            i2c_bus: The I2C bus number (default is 1 for most Raspberry Pi models).
            i2c_addr: The I2C address of the motor driver board.
            wheel_radius_m: The radius of the robot's wheels in meters.
            track_width_m: The distance between the centers of the two wheels in meters.
            control_hz: The frequency of the PID control loop in Hz.
            kp, ki, kd: Proportional, Integral, and Derivative gains for the PID controller.
            invert_left, invert_right: Booleans to reverse the direction of the left or right motor if needed.
            max_angular_rps: The maximum angular velocity in radians per second, used for turning control.
            encoder_reduction_ratio: The gear reduction ratio for the motor encoders.
        """
        self.board = Board(i2c_bus, i2c_addr)
        
        # --- FIX: Added a retry loop to prevent hanging if the board isn't found ---
        print("Initialising motor driver...")
        retry_count = 0
        max_retries = 5
        while self.board.begin() != self.board.STA_OK:
            retry_count += 1
            if retry_count > max_retries:
                print("Error: Motor driver board not detected after multiple retries. Exiting.")
                raise IOError("Failed to initialize DFRobot motor driver board.")
            print(f"  Board not detected, retrying ({retry_count}/{max_retries})...")
            time.sleep(1.0)
            
        print("Motor driver initialised and reporting OK")

        self.board.set_encoder_enable(self.board.ALL)
        self.board.set_encoder_reduction_ratio(self.board.ALL, encoder_reduction_ratio)
        # --- FIX: Corrected typo in method name 'set_moter_pwm_frequency' ---
        self.board.set_motor_pwm_frequency(1000)

        self.wheel_radius_m = wheel_radius_m
        self.track_width_m = track_width_m
        self.control_hz = control_hz
        self.dt = 1.0 / control_hz
        self.max_angular_rps = max_angular_rps

        self.kp_l, self.ki_l, self.kd_l = kp, ki, kd
        self.kp_r, self.ki_r, self.kd_r = kp, ki, kd

        self.invert_left = invert_left
        self.invert_right = invert_right

        # PID control state variables
        self._target_rpm_l = 0.0
        self._target_rpm_r = 0.0
        self._integral_l = 0.0
        self._integral_r = 0.0
        self._prev_err_l = 0.0
        self._prev_err_r = 0.0
        
        self._running = False
        self._lock = threading.Lock()
        self._thread = None

    def stop_all(self):
        """Stops all motors and resets PID controllers."""
        self._running = False # Signal the control loop to stop
        time.sleep(self.dt * 2) # Give the loop a moment to exit
        with self._lock:
            self._target_rpm_l = 0.0
            self._target_rpm_r = 0.0
            self._integral_l = 0.0
            self._integral_r = 0.0
        self.board.motor_stop(self.board.ALL)

    def shutdown(self):
        """Shuts down the drive system, stopping motors and cleaning up the control thread."""
        self._running = False
        if self._thread is not None and self._thread.is_alive():
            try:
                self._thread.join(timeout=1.0)
            except Exception as e:
                print(f"Error joining control thread: {e}")
        self.board.motor_stop(self.board.ALL)
        print("Drive system shut down.")

    def set_target_velocities(self, linear_mps: float, angular_rps: float):
        """
        Sets the desired linear and angular velocities for the robot.

        Args:
            linear_mps: Desired forward velocity in meters per second.
            angular_rps: Desired angular (turning) velocity in radians per second.
        """
        v_l, v_r = self._twist_to_wheel_speeds(linear_mps, angular_rps)
        rpm_l = self._mps_to_rpm(v_l)
        rpm_r = self._mps_to_rpm(v_r)
        
        with self._lock:
            self._target_rpm_l = rpm_l
            self._target_rpm_r = rpm_r
            
        if not self._running:
            self._start_loop()

    def get_wheel_speeds(self) -> Tuple[float, float]:
        """
        Gets the current measured speed of each wheel.

        Returns:
            A tuple containing the left and right wheel speeds in meters per second.
        """
        # --- FIX: The get_encoder_speed() method must be called for each motor individually ---
        rpm_l = float(self.board.get_encoder_speed(self.board.M1))
        rpm_r = float(self.board.get_encoder_speed(self.board.M2))
        
        v_l = self._rpm_to_mps(rpm_l)
        v_r = self._rpm_to_mps(rpm_r)
        return v_l, v_r

    def set_pid_gains(self, kp: Optional[float] = None, ki: Optional[float] = None, kd: Optional[float] = None):
        """Updates the PID gains for both wheel controllers."""
        with self._lock:
            if kp is not None: self.kp_l = self.kp_r = kp
            if ki is not None: self.ki_l = self.ki_r = ki
            if kd is not None: self.kd_l = self.kd_r = kd

    def _start_loop(self):
        """Starts the background PID control loop if it's not already running."""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._pid_loop, daemon=True)
        self._thread.start()

    def _pid_loop(self):
        """The main PID control loop that runs in a separate thread."""
        try:
            while self._running:
                t0 = time.time()
                
                # --- FIX: Must read each encoder speed with a separate call ---
                rpm_l_meas = float(self.board.get_encoder_speed(self.board.M1))
                rpm_r_meas = float(self.board.get_encoder_speed(self.board.M2))

                with self._lock:
                    rpm_l_target = self._target_rpm_l
                    rpm_r_target = self._target_rpm_r

                # --- REFACTOR: PID logic is now clearer and more direct ---
                # Left Motor PID Calculation
                err_l = rpm_l_target - rpm_l_meas
                self._integral_l += err_l * self.dt
                self._integral_l = max(min(self._integral_l, 200.0), -200.0) # Integral clamping
                deriv_l = (err_l - self._prev_err_l) / self.dt
                output_l = self.kp_l * err_l + self.ki_l * self._integral_l + self.kd_l * deriv_l
                self._prev_err_l = err_l
                
                # Right Motor PID Calculation
                err_r = rpm_r_target - rpm_r_meas
                self._integral_r += err_r * self.dt
                self._integral_r = max(min(self._integral_r, 200.0), -200.0) # Integral clamping
                deriv_r = (err_r - self._prev_err_r) / self.dt
                output_r = self.kp_r * err_r + self.ki_r * self._integral_r + self.kd_r * deriv_r
                self._prev_err_r = err_r

                # Anti-windup reset: if target is zero, reset the integral
                if abs(rpm_l_target) < 0.5: self._integral_l = 0.0
                if abs(rpm_r_target) < 0.5: self._integral_r = 0.0
                
                pwm_l = max(0.0, min(100.0, abs(output_l)))
                pwm_r = max(0.0, min(100.0, abs(output_r)))

                self._apply_motor(self.board.M1, pwm_l, rpm_l_target, self.invert_left)
                self._apply_motor(self.board.M2, pwm_r, rpm_r_target, self.invert_right)

                elapsed = time.time() - t0
                time.sleep(max(0.0, self.dt - elapsed))
        finally:
            # --- IMPROVEMENT: Ensure motors are stopped if the loop exits for any reason ---
            self.board.motor_stop(self.board.ALL)
            print("PID loop has stopped.")


    def _apply_motor(self, motor_id: int, pwm: float, desired_rpm: float, invert: bool):
        """
        Applies a PWM value to a specific motor, setting its direction.
        
        Args:
            motor_id: The motor to control (e.g., self.board.M1).
            pwm: The PWM duty cycle (0-100).
            desired_rpm: The target RPM, used to determine direction.
            invert: Whether to invert the motor's direction.
        """
        # Create a small deadzone to prevent motor whine at near-zero speeds
        if abs(desired_rpm) < 0.1 or pwm < 1.0:
            # --- FIX: motor_stop takes a single ID, not a list ---
            self.board.motor_stop(motor_id)
            return

        forward = desired_rpm >= 0.0
        if invert:
            forward = not forward
            
        orientation = self.board.CW if forward else self.board.CCW
        
        # --- FIX: motor_movement takes a single ID, not a list ---
        self.board.motor_movement([motor_id], orientation, pwm)

    # --- Kinematic helper functions (unchanged) ---
    def _twist_to_wheel_speeds(self, v: float, w: float) -> Tuple[float, float]:
        half_w = self.track_width_m / 2.0
        v_l = v - w * half_w
        v_r = v + w * half_w
        return v_l, v_r

    def _mps_to_rpm(self, v: float) -> float:
        if self.wheel_radius_m <= 0:
            return 0.0
        return (v / (2.0 * math.pi * self.wheel_radius_m)) * 60.0

    def _rpm_to_mps(self, rpm: float) -> float:
        return (rpm / 60.0) * (2.0 * math.pi * self.wheel_radius_m)