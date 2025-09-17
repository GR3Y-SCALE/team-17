from gpiozero import Servo
from time import sleep 

GripperServo = Servo(18, min_pulse_width=0.0007, max_pulse_width=0.0018)
LiftServo = Servo(19,min_pulse_width=0.0007, max_pulse_width=0.0022)

print ("Griper Servo Test / Press Ctrl+C to end")

try:
    # while True:
    #     GriperServo.min()
    #     sleep(2)

    #     GriperServo.mid()
    #     sleep(2)

    #     GriperServo.max()
    #     sleep(2)              #test code for min max positions
    #
 
    while True:
        GripperServo.detach()
        print("Lift moving to MIN")
        LiftServo.min()      # Move lift servo to its minimum position
        sleep(2)

        print("Lift moving to MID")
        LiftServo.mid()      # Move lift servo to middle position
        sleep(2)

        print("Lift moving to MAX")
        LiftServo.max()      # Move lift servo to maximum position
        sleep(2)

        #print("Gripper opening")
        #GripperServo.min()   # Open gripper
        #sleep(2)

        #print("Gripper closing")
        #GripperServo.value = 0.75   # Close gripper to about 75% of range
        #sleep(2)

        # Detach gripper when closed to stop jitter
        #GripperServo.detach()
        #print("Gripper detached to prevent jitter")

        sleep(3)  # Wait a few seconds before restarting loop

# --- Safe shutdown block ---
except KeyboardInterrupt:
    print("\nStopping program...")

    # Detach and close each servo cleanly
    GripperServo.detach()
    LiftServo.detach()

    GripperServo.close()
    LiftServo.close()

    print("Servos stopped safely. Program ended.")