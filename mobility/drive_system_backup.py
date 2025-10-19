import os, sys, time, math, threading
from typing import Tuple, Optional

from mobility.DFRobot_RaspberryPi_DC_Motor import THIS_BOARD_TYPE, DFRobot_DC_Motor_IIC as Board

class DriveSystem:
    """
    A class to control a differential drive robot using the DFRobot DC Motor Driver.
    Manages motor initialization, velocity control, and a PID feedback loop for
    precise speed regulation based on encoder feedback.
    """
    def __init__(self,
                 i2c_bus: int = 1,
                 i2c_addr: int = 0x10,
                 wheel_radius_m: float = 0.032,
                 track_width_m: float = 0.155,
                 control_hz: float = 100.0,
                 kp: float = 1.0,
                 ki: float = 0.04,
                 kd: float = 0.08,
                 invert_left: bool = True,
                 invert_right: bool = False,
                 max_angular_dps: float = 90.0, # Max angular velocity in degrees per second
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
            max_angular_dps: The maximum angular velocity in degrees per second, used for turning control.
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

        print("[ OK ] Motor driver initialised.")

        # Ensure encoders are enabled and ratios set
        self.board.set_encoder_enable(self.board.M1)
        self.board.set_encoder_enable(self.board.M2)
        self.board.set_encoder_reduction_ratio(self.board.M1, encoder_reduction_ratio)
        self.board.set_encoder_reduction_ratio(self.board.M2, encoder_reduction_ratio)
        
        self.board.set_motor_pwm_frequency(1000)

        self.wheel_radius_m = wheel_radius_m
        self.track_width_m = track_width_m
        self.control_hz = control_hz
        self.dt = 1.0 / control_hz
        self.max_angular_dps = max_angular_dps # Stored in degrees per second

        # PID gains for left and right motors (can be set independently if needed)
        self.kp_l, self.ki_l, self.kd_l = kp, ki, kd
        self.kp_r, self.ki_r, self.kd_r = kp, ki, kd

        self.invert_left = invert_left
        self.invert_right = invert_right

        # PID control state variables, protected by lock
        self._target_rpm_l = 0.0
        self._target_rpm_r = 0.0
        self._integral_l = 0.0
        self._integral_r = 0.0
        self._prev_err_l = 0.0
        self._prev_err_r = 0.0

        self._running = False
        self._lock = threading.Lock()
        self._thread = None

        # Ensure motors are stopped initially
        self._stop_all_motors_hardware()

    def _stop_all_motors_hardware(self):
        """Directly sends commands to stop both motors on the hardware."""
        self.board.motor_movement(self.board.M1, self.board.CW, 0)
        self.board.motor_movement(self.board.M2, self.board.CW, 0)

    def stop_all(self):
        """Signals the control loop to stop and then ensures motors are physically off."""
        self._running = False # Signal the control loop to stop
        # Reset PID states immediately
        with self._lock:
            self._target_rpm_l = 0.0
            self._target_rpm_r = 0.0
            self._integral_l = 0.0
            self._integral_r = 0.0
            self._prev_err_l = 0.0
            self._prev_err_r = 0.0
        time.sleep(self.dt * 2) # Give the loop a moment to exit gracefully
        self._stop_all_motors_hardware()

    def shutdown(self):
        """Shuts down the drive system, stopping motors and cleaning up the control thread."""
        print("Shutting down drive system...")
        self._running = False
        if self._thread is not None and self._thread.is_alive():
            try:
                self._thread.join(timeout=2.0) # Give more time for thread to join
            except Exception as e:
                print(f"Error joining control thread: {e}")
        self._stop_all_motors_hardware()
        print("Drive system shut down.")

    def set_target_velocities(self, linear_mps: float, angular_dps: float):
        """
        Sets the desired linear and angular velocities for the robot.
        Starts the PID control loop if not already running.

        Args:
            linear_mps: Desired forward velocity in meters per second.
            angular_dps: Desired angular (turning) velocity in degrees per second.
        """
        # Limit angular_dps to the maximum allowed
        angular_dps = max(-self.max_angular_dps, min(self.max_angular_dps, angular_dps))

        # Convert angular velocity from degrees/sec to radians/sec for kinematics
        angular_rps = math.radians(angular_dps)
        
        v_l, v_r = self._twist_to_wheel_speeds(linear_mps, angular_rps)
        rpm_l = self._mps_to_rpm(v_l)
        rpm_r = self._mps_to_rpm(v_r)

        with self._lock:
            self._target_rpm_l = rpm_l
            self._target_rpm_r = rpm_r

        if not self._running:
            self._start_loop()

    def drive_distance(self, distance_m: float, speed_mps: float = 0.1):
        """
        Drives the robot forward or backward a specified distance.
        This is a blocking call.

        Args:
            distance_m: The distance to drive in meters (positive for forward, negative for backward).
            speed_mps: The speed at which to drive in meters per second.
        """
        if speed_mps <= 0.0:
            raise ValueError("Speed must be positive for distance control.")

        direction_factor = 1.0 if distance_m >= 0.0 else -1.0
        target_distance_abs = abs(distance_m)
        target_speed_mps = abs(speed_mps) * direction_factor

        print(f"Driving {distance_m:.2f}m at {target_speed_mps:.2f} m/s")

        # Reset encoders to zero before starting
        self.board.set_encoder_enable(self.board.M1, False) # Disable to reset
        self.board.set_encoder_enable(self.board.M2, False)
        time.sleep(0.05) # Small delay for reset to take effect
        self.board.set_encoder_enable(self.board.M1, True) # Re-enable
        self.board.set_encoder_enable(self.board.M2, True)
        time.sleep(0.05) # Small delay for enable to take effect

        start_time = time.time()
        distance_traveled = 0.0
        
        self.set_target_velocities(target_speed_mps, 0.0) # Start driving

        try:
            while distance_traveled < target_distance_abs:
                # Read current wheel speeds (blocking call for a moment)
                v_l, v_r = self.get_wheel_speeds()
                
                # Calculate average speed
                current_linear_mps = (v_l + v_r) / 2.0
                
                # Estimate distance traveled (simplified, assuming constant speed over short dt)
                # A more accurate way would be to integrate encoder counts directly
                distance_traveled = abs(current_linear_mps) * (time.time() - start_time) # Rough estimate

                # If the robot is supposed to move forward, and we detect it going backward, or vice-versa
                # This check might be too aggressive for PID systems
                # if (direction_factor > 0 and current_linear_mps < -0.01) or \
                #    (direction_factor < 0 and current_linear_mps > 0.01):
                #     print("Warning: Robot moving in wrong direction during drive_distance. Adjusting...")
                #     # Potentially re-adjust target or PID, or stop. For now, let PID handle it.

                time.sleep(0.05) # Don't hog CPU, allow other operations
                
                # Optional: Add a timeout to prevent infinite loops
                if time.time() - start_time > abs(distance_m / speed_mps) * 2.0 + 5: # 2x expected time + 5s buffer
                    print("Warning: drive_distance timed out.")
                    break

            print(f"Finished driving {distance_traveled:.2f}m. Target: {target_distance_abs:.2f}m")
        finally:
            self.stop_all() # Ensure motors stop even if an error occurs

    def turn_degrees(self, degrees: float, angular_speed_dps: float = 45.0):
        """
        Turns the robot a specified number of degrees.
        This is a blocking call.

        Args:
            degrees: The angle to turn in degrees (positive for left, negative for right).
            angular_speed_dps: The angular speed in degrees per second.
        """
        if angular_speed_dps <= 0.0:
            raise ValueError("Angular speed must be positive for turn_degrees.")
        
        direction_factor = 1.0 if degrees >= 0.0 else -1.0
        target_angle_abs_rad = math.radians(abs(degrees))
        target_angular_dps = abs(angular_speed_dps) * direction_factor

        print(f"Turning {degrees:.2f} degrees at {target_angular_dps:.2f} dps")

        # Reset encoders
        self.board.set_encoder_enable(self.board.M1, False)
        self.board.set_encoder_enable(self.board.M2, False)
        time.sleep(0.05)
        self.board.set_encoder_enable(self.board.M1, True)
        self.board.set_encoder_enable(self.board.M2, True)
        time.sleep(0.05)

        start_time = time.time()
        angle_rotated_rad = 0.0

        self.set_target_velocities(0.0, target_angular_dps) # Start turning

        try:
            while angle_rotated_rad < target_angle_abs_rad:
                v_l, v_r = self.get_wheel_speeds()
                
                # Calculate current angular velocity in rad/s
                # angular_w_rps = (v_r - v_l) / self.track_width_m
                # The _twist_to_wheel_speeds logic is v_l = v - w*half_w, v_r = v + w*half_w
                # So, v_r - v_l = 2 * w * half_w = w * track_width_m
                # Thus, w = (v_r - v_l) / track_width_m
                current_angular_rps = (v_r - v_l) / self.track_width_m
                
                # Integrate angular velocity to get angle
                angle_rotated_rad = abs(current_angular_rps) * (time.time() - start_time)
                
                time.sleep(0.05)
                
                # Optional: Add a timeout
                # Calculate expected time to turn
                expected_turn_time = abs(degrees / angular_speed_dps)
                if time.time() - start_time > expected_turn_time * 2.0 + 5:
                    print("Warning: turn_degrees timed out.")
                    break

            print(f"Finished turning {math.degrees(angle_rotated_rad):.2f} degrees. Target: {abs(degrees):.2f} degrees")
        finally:
            self.stop_all()

    def get_wheel_speeds(self) -> Tuple[float, float]:
        """
        Gets the current measured speed of each wheel from the encoders.

        Returns:
            A tuple containing the left and right wheel speeds in meters per second.
        """
        # --- FIX: Must read each encoder speed with a separate call ---
        rpm_l_raw = self.board.get_encoder_speed(self.board.M1)
        rpm_r_raw = self.board.get_encoder_speed(self.board.M2)
        
        # Apply inversion to the raw RPM values if configured
        rpm_l = rpm_l_raw * (-1 if self.invert_left else 1)
        rpm_r = rpm_r_raw * (-1 if self.invert_right else 1)

        v_l = self._rpm_to_mps(rpm_l)
        v_r = self._rpm_to_mps(rpm_r)
        return v_l, v_r

    def set_pid_gains(self, kp: Optional[float] = None, ki: Optional[float] = None, kd: Optional[float] = None):
        """Updates the PID gains for both wheel controllers."""
        with self._lock:
            if kp is not None:
                self.kp_l = kp
                self.kp_r = kp
            if ki is not None:
                self.ki_l = ki
                self.ki_r = ki
            if kd is not None:
                self.kd_l = kd
                self.kd_r = kd
            print(f"PID gains updated to: P={self.kp_l}, I={self.ki_l}, D={self.kd_l}")

    def _start_loop(self):
        """Starts the background PID control loop if it's not already running."""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._pid_loop, daemon=True)
        self._thread.start()
        print(f"PID control loop started at {self.control_hz} Hz.")

    def _pid_loop(self):
        """The main PID control loop that runs in a separate thread."""
        try:
            while self._running:
                t0 = time.time()
                
                # Read current encoder speeds (RPMs)
                # --- FIX: Must read each encoder speed with a separate call ---
                rpm_l_raw, rpm_r_raw = self.board.get_encoder_speed(self.board.ALL)
                # rpm_r_raw = self.board.get_encoder_speed(self.board.M2)
                
                # Apply inversion for PID calculation (target vs measured direction)
                rpm_l_meas = rpm_l_raw * (-1 if self.invert_left else 1)
                rpm_r_meas = rpm_r_raw * (-1 if self.invert_right else 1)

                with self._lock:
                    rpm_l_target = self._target_rpm_l
                    rpm_r_target = self._target_rpm_r
                    
                    # Left Motor PID Calculation
                    err_l = rpm_l_target - abs(rpm_l_meas) # Error is target minus measured
                    self._integral_l += err_l * self.dt
                    self._integral_l = max(min(self._integral_l, 200.0), -200.0) # Integral clamping
                    deriv_l = (err_l - self._prev_err_l) / self.dt
                    output_l = self.kp_l * err_l + self.ki_l * self._integral_l + self.kd_l * deriv_l
                    self._prev_err_l = err_l
                    
                    # Right Motor PID Calculation
                    err_r = rpm_r_target - abs(rpm_r_meas) # Error is target minus measured
                    self._integral_r += err_r * self.dt
                    self._integral_r = max(min(self._integral_r, 200.0), -200.0) # Integral clamping
                    deriv_r = (err_r - self._prev_err_r) / self.dt
                    output_r = self.kp_r * err_r + self.ki_r * self._integral_r + self.kd_r * deriv_r
                    self._prev_err_r = err_r

                    # Anti-windup reset: if target is zero, reset the integral
                    # This helps prevent overshoot when stopping
                    if abs(rpm_l_target) < 0.5: self._integral_l = 0.0
                    if abs(rpm_r_target) < 0.5: self._integral_r = 0.0
                    
                    # Convert PID output to a positive PWM value (0-100)
                    # The sign of the target RPM determines direction, PWM is magnitude
                    pwm_l = max(0.0, min(100.0, abs(output_l)))
                    pwm_r = max(0.0, min(100.0, abs(output_r)))

                # Apply PWM and direction to motors
                self._apply_motor(self.board.M1, pwm_l, rpm_l_target, self.invert_left)
                self._apply_motor(self.board.M2, pwm_r, rpm_r_target, self.invert_right)

                # Maintain control loop frequency
                elapsed = time.time() - t0
                sleep_time = max(0.0, self.dt - elapsed)
                if sleep_time == 0:
                    print(f"Warning: PID loop is taking longer than {self.dt*1000:.1f}ms (took {elapsed*1000:.1f}ms)")
                time.sleep(sleep_time)
        finally:
            # Ensure motors are stopped if the loop exits for any reason
            self._stop_all_motors_hardware()
            print("PID loop has stopped.")

    def _apply_motor(self, motor_id: int, pwm: float, desired_rpm: float, invert: bool):
        """
        Applies a PWM value to a specific motor, setting its direction.
        
        Args:
            motor_id: The motor to control (e.g., self.board.M1).
            pwm: The PWM duty cycle (0-100).
            desired_rpm: The target RPM, used to determine direction.
            invert: Whether to invert the motor's physical direction relative to its control.
        """
        # Create a small deadzone to prevent motor whine or jitter at near-zero speeds/PWM
        if abs(desired_rpm) < 0.5 or pwm < 1.0: # If target RPM is very low or PWM is very low
            self.board.motor_movement(motor_id, self.board.CW, 0) # Stop motor
            return

        # Determine the physical direction based on desired RPM and inversion flag
        forward_motion = desired_rpm >= 0.0
        if invert:
            forward_motion = not forward_motion
            
        orientation = self.board.CW if forward_motion else self.board.CCW
        
        # --- FIX: motor_movement takes a single ID, not a list ---
        self.board.motor_movement(motor_id, orientation, pwm)

    # --- Kinematic helper functions ---
    def _twist_to_wheel_speeds(self, v: float, w_rad_s: float) -> Tuple[float, float]:
        """
        Converts linear and angular robot velocities into individual wheel velocities.

        Args:
            v: Linear velocity of the robot's center (m/s).
            w_rad_s: Angular velocity of the robot (rad/s).

        Returns:
            A tuple (left_wheel_velocity_mps, right_wheel_velocity_mps).
        """
        half_track_width = self.track_width_m / 2.0
        v_l = v - w_rad_s * half_track_width
        v_r = v + w_rad_s * half_track_width
        return v_l, v_r

    def _mps_to_rpm(self, v_mps: float) -> float:
        """Converts meters per second to revolutions per minute."""
        if self.wheel_radius_m <= 0:
            return 0.0
        # Circumference = 2 * pi * R
        # RPM = (velocity in m/s / circumference in m) * 60 seconds/minute
        return (v_mps / (2.0 * math.pi * self.wheel_radius_m)) * 60.0

    def _rpm_to_mps(self, rpm: float) -> float:
        """Converts revolutions per minute to meters per second."""
        # Circumference = 2 * pi * R
        # m/s = (RPM / 60 seconds/minute) * circumference in m
        return (rpm / 60.0) * (2.0 * math.pi * self.wheel_radius_m)
    
    def _rpm_to_dps(self, rpm_left: float, rpm_right: float) -> float:
        """
        Converts wheel RPMs to the robot's angular velocity in degrees per second.
        
        Args:
            rpm_left: Left wheel RPM.
            rpm_right: Right wheel RPM.
            
        Returns:
            Angular velocity of the robot in degrees per second.
        """
        # Convert wheel RPMs to meters per second
        v_l = self._rpm_to_mps(rpm_left)
        v_r = self._rpm_to_mps(rpm_right)
        
        # Calculate angular velocity (rad/s)
        # w = (v_r - v_l) / track_width_m
        if self.track_width_m == 0:
            return 0.0
        angular_w_rps = (v_r - v_l) / self.track_width_m
        
        # Convert rad/s to deg/s
        return math.degrees(angular_w_rps)