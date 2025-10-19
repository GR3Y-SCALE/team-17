from mobility.drive_system_backup import DriveSystem
import time

robot = DriveSystem()
# robot.drive_distance(0.1, 0.10)
# robot.turn_degrees(220, 110) #90 degrees
# robot.turn_degrees(240, 100) #90 degrees
# robot.turn_degrees(230, 90) #90 degrees
# robot.turn_degrees(220, 80) #90 degrees
# robot.turn_degrees(170, 60) #90 degrees
robot.turn_degrees(140, 50) #90 degrees




# """
# Encoder Calibration and Testing Script
# This script helps you build a truth table comparing encoder-reported distances
# to actual measured distances to identify calibration issues.
# """

# import time
# import csv
# from datetime import datetime
# from typing import List, Dict
# from mobility.drive_system import DriveSystem  # Import your DriveSystem class

# class EncoderCalibrationTest:
#     """
#     A testing framework to calibrate and validate encoder readings against
#     real-world measurements.
#     """
    
#     def __init__(self, drive_system: DriveSystem):
#         """
#         Initialize the calibration tester.
        
#         Args:
#             drive_system: An initialized DriveSystem instance
#         """
#         self.drive = drive_system
#         self.results = []
        
#     def run_distance_test(self, 
#                          target_distance_m: float, 
#                          speed_mps: float = 0.25,
#                          test_name: str = ""):
#         """
#         Run a single distance test and collect encoder data.
        
#         Args:
#             target_distance_m: Distance to attempt to drive (meters)
#             speed_mps: Speed to drive at (m/s)
#             test_name: Optional name/description for this test
            
#         Returns:
#             dict: Test results including encoder readings and calculations
#         """
#         print(f"\n{'='*60}")
#         print(f"TEST: {test_name if test_name else f'Drive {target_distance_m}m'}")
#         print(f"Target Distance: {target_distance_m} m")
#         print(f"Speed: {speed_mps} m/s")
#         print(f"{'='*60}")
        
#         # Reset encoders
#         self.drive.board.set_encoder_enable(self.drive.board.ALL)
#         time.sleep(0.2)
        
#         # Record initial encoder values
#         initial_rpm_l, initial_rpm_r = self.drive.board.get_encoder_speed(self.drive.board.ALL)
#         print(f"Initial encoder RPMs - Left: {initial_rpm_l}, Right: {initial_rpm_r}")
        
#         # Start the drive using drive_distance method (same as your working test.py)
#         # We'll let it run and monitor, rather than using set_target_velocities directly
        
#         # For monitoring, we'll use a threaded approach
#         direction = 1.0 if target_distance_m >= 0 else -1.0
#         actual_speed = abs(speed_mps) * direction
        
#         # Use the drive_distance method directly (like your test.py)
#         # This ensures we're using the exact same code path that works
#         import threading
        
#         sample_data = []
#         monitoring = {'running': True}
        
#         def monitor_drive():
#             """Background thread to collect encoder data during the drive."""
#             start_time = time.time()
#             last_time = start_time
            
#             while monitoring['running']:
#                 time.sleep(0.05)  # Sample at 20Hz
                
#                 current_time = time.time()
#                 dt = current_time - last_time
                
#                 try:
#                     # Read current encoder speeds
#                     rpm_l, rpm_r = self.drive.board.get_encoder_speed(self.drive.board.ALL)
                    
#                     # Convert to m/s
#                     v_l = self.drive._rpm_to_mps(rpm_l)
#                     v_r = self.drive._rpm_to_mps(rpm_r)
                    
#                     # Store sample
#                     sample_data.append({
#                         'time': current_time - start_time,
#                         'rpm_l': rpm_l,
#                         'rpm_r': rpm_r,
#                         'v_l_mps': v_l,
#                         'v_r_mps': v_r,
#                         'distance': 0  # Will calculate after
#                     })
                    
#                     last_time = current_time
#                 except Exception as e:
#                     print(f"Monitor error: {e}")
#                     break
        
#         # Start monitoring thread
#         monitor_thread = threading.Thread(target=monitor_drive, daemon=True)
#         monitor_thread.start()
        
#         # Execute the drive using the same method as your test.py
#         start_time = time.time()
#         try:
#             self.drive.drive_distance(target_distance_m, speed_mps)
#         finally:
#             monitoring['running'] = False
#             monitor_thread.join(timeout=1.0)
        
#         elapsed_time = time.time() - start_time
        
#         # Calculate cumulative distance from samples
#         distance_traveled_encoder = 0.0
#         for i, sample in enumerate(sample_data):
#             if i > 0:
#                 dt = sample['time'] - sample_data[i-1]['time']
#                 avg_v = (abs(sample['v_l_mps']) + abs(sample['v_r_mps'])) / 2.0
#                 distance_traveled_encoder += avg_v * dt
#             sample['distance'] = distance_traveled_encoder
        
#         # Final encoder readings
#         final_rpm_l, final_rpm_r = self.drive.board.get_encoder_speed(self.drive.board.ALL)
        
#         elapsed_time = time.time() - start_time
        
#         # Compile results
#         result = {
#             'test_name': test_name,
#             'timestamp': datetime.now().isoformat(),
#             'target_distance_m': target_distance_m,
#             'commanded_speed_mps': speed_mps,
#             'encoder_distance_m': distance_traveled_encoder,
#             'actual_distance_m': None,  # USER FILLS THIS IN
#             'elapsed_time_s': elapsed_time,
#             'avg_rpm_l': sum(s['rpm_l'] for s in sample_data) / len(sample_data) if sample_data else 0,
#             'avg_rpm_r': sum(s['rpm_r'] for s in sample_data) / len(sample_data) if sample_data else 0,
#             'final_rpm_l': final_rpm_l,
#             'final_rpm_r': final_rpm_r,
#             'num_samples': len(sample_data),
#             'sample_data': sample_data
#         }
        
#         self.results.append(result)
        
#         # Print summary
#         print(f"\n--- TEST COMPLETE ---")
#         print(f"Elapsed Time: {elapsed_time:.2f} s")
#         print(f"Encoder Reported Distance: {distance_traveled_encoder:.4f} m")
#         print(f"Average Left RPM: {result['avg_rpm_l']:.2f}")
#         print(f"Average Right RPM: {result['avg_rpm_r']:.2f}")
#         print(f"\n⚠️  NOW MEASURE THE ACTUAL DISTANCE WITH A TAPE MEASURE!")
#         print(f"Enter the actual distance when prompted.\n")
        
#         return result
    
#     def run_test_suite(self, 
#                        test_distances: List[float] = [0.5, 1.0, 1.5, 2.0],
#                        speed_mps: float = 0.25):
#         """
#         Run a suite of distance tests at various distances.
        
#         Args:
#             test_distances: List of distances to test (in meters)
#             speed_mps: Speed to use for all tests
#         """
#         print("\n" + "="*60)
#         print("STARTING ENCODER CALIBRATION TEST SUITE")
#         print("="*60)
#         print(f"Testing {len(test_distances)} distances: {test_distances}")
#         print(f"Speed: {speed_mps} m/s")
#         print("\nAfter each test, you'll be prompted to enter the actual distance.")
#         input("Press ENTER to begin...")
        
#         for i, distance in enumerate(test_distances, 1):
#             test_name = f"Test {i}: {distance}m forward"
#             result = self.run_distance_test(distance, speed_mps, test_name)
            
#             # Prompt for actual measurement
#             while True:
#                 try:
#                     actual = input(f"\nEnter ACTUAL measured distance (meters) [or 's' to skip]: ")
#                     if actual.lower() == 's':
#                         result['actual_distance_m'] = None
#                         break
#                     actual_distance = float(actual)
#                     result['actual_distance_m'] = actual_distance
                    
#                     # Calculate error
#                     error = result['encoder_distance_m'] - actual_distance
#                     error_percent = (error / actual_distance * 100) if actual_distance != 0 else 0
#                     result['error_m'] = error
#                     result['error_percent'] = error_percent
                    
#                     print(f"✓ Recorded: Encoder={result['encoder_distance_m']:.4f}m, "
#                           f"Actual={actual_distance:.4f}m, "
#                           f"Error={error:.4f}m ({error_percent:+.2f}%)")
#                     break
#                 except ValueError:
#                     print("Invalid input. Please enter a number or 's' to skip.")
            
#             # Pause between tests
#             if i < len(test_distances):
#                 print("\nReturn robot to starting position for next test.")
#                 input("Press ENTER when ready for next test...")
    
#     def print_truth_table(self):
#         """Print a formatted truth table of all test results."""
#         if not self.results:
#             print("No test results available.")
#             return
        
#         print("\n" + "="*80)
#         print("ENCODER CALIBRATION TRUTH TABLE")
#         print("="*80)
#         print(f"{'Test':<20} {'Target':<10} {'Encoder':<10} {'Actual':<10} {'Error':<10} {'Error %':<10}")
#         print(f"{'Name':<20} {'(m)':<10} {'(m)':<10} {'(m)':<10} {'(m)':<10} {'(%)':<10}")
#         print("-"*80)
        
#         for result in self.results:
#             test_name = result['test_name'][:18]
#             target = result['target_distance_m']
#             encoder = result['encoder_distance_m']
#             actual = result.get('actual_distance_m')
#             error = result.get('error_m')
#             error_pct = result.get('error_percent')
            
#             actual_str = f"{actual:.4f}" if actual is not None else "N/A"
#             error_str = f"{error:+.4f}" if error is not None else "N/A"
#             error_pct_str = f"{error_pct:+.2f}" if error_pct is not None else "N/A"
            
#             print(f"{test_name:<20} {target:<10.4f} {encoder:<10.4f} {actual_str:<10} "
#                   f"{error_str:<10} {error_pct_str:<10}")
        
#         # Calculate average error if we have actual measurements
#         valid_results = [r for r in self.results if r.get('actual_distance_m') is not None]
#         if valid_results:
#             avg_error = sum(r['error_m'] for r in valid_results) / len(valid_results)
#             avg_error_pct = sum(r['error_percent'] for r in valid_results) / len(valid_results)
            
#             print("-"*80)
#             print(f"{'AVERAGE ERROR:':<40} {avg_error:+.4f} m ({avg_error_pct:+.2f}%)")
            
#             # Calculate suggested correction factor
#             avg_encoder = sum(r['encoder_distance_m'] for r in valid_results) / len(valid_results)
#             avg_actual = sum(r['actual_distance_m'] for r in valid_results) / len(valid_results)
#             correction_factor = avg_actual / avg_encoder if avg_encoder != 0 else 1.0
            
#             print(f"{'SUGGESTED CORRECTION FACTOR:':<40} {correction_factor:.6f}")
#             print(f"{'(Multiply encoder readings by this factor)':<40}")
        
#         print("="*80 + "\n")
    
#     def save_results_csv(self, filename: str = None):
#         """
#         Save test results to a CSV file.
        
#         Args:
#             filename: Output filename (default: encoder_calibration_TIMESTAMP.csv)
#         """
#         if not self.results:
#             print("No results to save.")
#             return
        
#         if filename is None:
#             timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
#             filename = f"encoder_calibration_{timestamp}.csv"
        
#         with open(filename, 'w', newline='') as csvfile:
#             fieldnames = ['test_name', 'timestamp', 'target_distance_m', 'encoder_distance_m',
#                          'actual_distance_m', 'error_m', 'error_percent', 'commanded_speed_mps',
#                          'elapsed_time_s', 'avg_rpm_l', 'avg_rpm_r']
            
#             writer = csv.DictWriter(csvfile, fieldnames=fieldnames, extrasaction='ignore')
#             writer.writeheader()
            
#             for result in self.results:
#                 writer.writerow(result)
        
#         print(f"✓ Results saved to: {filename}")
    
#     def save_detailed_csv(self, filename: str = None):
#         """
#         Save detailed sample data to CSV for analysis.
        
#         Args:
#             filename: Output filename (default: encoder_calibration_detailed_TIMESTAMP.csv)
#         """
#         if not self.results:
#             print("No results to save.")
#             return
        
#         if filename is None:
#             timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
#             filename = f"encoder_calibration_detailed_{timestamp}.csv"
        
#         with open(filename, 'w', newline='') as csvfile:
#             fieldnames = ['test_name', 'sample_time', 'rpm_l', 'rpm_r', 
#                          'v_l_mps', 'v_r_mps', 'distance']
            
#             writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
#             writer.writeheader()
            
#             for result in self.results:
#                 test_name = result['test_name']
#                 for sample in result.get('sample_data', []):
#                     row = {'test_name': test_name, 'sample_time': sample['time'],
#                            'rpm_l': sample['rpm_l'], 'rpm_r': sample['rpm_r'],
#                            'v_l_mps': sample['v_l_mps'], 'v_r_mps': sample['v_r_mps'],
#                            'distance': sample['distance']}
#                     writer.writerow(row)
        
#         print(f"✓ Detailed data saved to: {filename}")


# # ===== EXAMPLE USAGE =====
# if __name__ == "__main__":
#     print("Encoder Calibration Test Script")
#     print("================================\n")
    
#     # Initialize your drive system
#     print("Initializing drive system...")
#     drive = DriveSystem()
    
#     # Create calibration tester
#     tester = EncoderCalibrationTest(drive)
    
#     # Option 1: Run a full test suite
#     print("\nOption 1: Run full test suite")
#     print("This will test multiple distances automatically.\n")
    
#     test_distances = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]  # Your specified test distances
    
#     choice = input("Run full test suite? (y/n): ")
#     if choice.lower() == 'y':
#         tester.run_test_suite(test_distances=test_distances, speed_mps=0.25)
#     else:
#         # Option 2: Run individual tests
#         print("\nRunning individual tests...")
#         while True:
#             try:
#                 distance = input("\nEnter distance to test (m) [or 'q' to quit]: ")
#                 if distance.lower() == 'q':
#                     break
                
#                 distance = float(distance)
#                 result = tester.run_distance_test(distance, speed_mps=0.25)
                
#                 # Get actual measurement
#                 actual = input("Enter ACTUAL measured distance (m) [or 's' to skip]: ")
#                 if actual.lower() != 's':
#                     actual_distance = float(actual)
#                     result['actual_distance_m'] = actual_distance
#                     error = result['encoder_distance_m'] - actual_distance
#                     error_percent = (error / actual_distance * 100) if actual_distance != 0 else 0
#                     result['error_m'] = error
#                     result['error_percent'] = error_percent
#                     print(f"Error: {error:.4f}m ({error_percent:+.2f}%)")
                
#                 input("\nReturn robot to start. Press ENTER to continue...")
                
#             except ValueError:
#                 print("Invalid input.")
#             except KeyboardInterrupt:
#                 print("\n\nTest interrupted.")
#                 break
    
#     # Display results
#     tester.print_truth_table()
    
#     # Save results
#     save = input("Save results to CSV? (y/n): ")
#     if save.lower() == 'y':
#         tester.save_results_csv()
#         tester.save_detailed_csv()
    
#     # Cleanup
#     drive.shutdown()
#     print("\nCalibration testing complete!")