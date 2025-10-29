🤖 Ball and Plate Balancing System
A real-time, computer-vision-based project that controls a 2-axis platform to balance a ball or make it follow dynamic paths using a PID controller.

🎥 Demonstration
This video shows the ball dynamically following a circular setpoint (red dot) controlled by the GUI.

(Note: For your README, you should convert circle_video.mp4 to a GIF and embed it here. A common way is to add circle_demo.gif to your repo and use this line:) ![System Demo](circle_demo.gif)

📋 Table of Contents
Features






🚀 Features
Real-time Ball Balancing: Automatically keeps the ball centered on the plate.

Dynamic Path Following: Make the ball trace pre-programmed shapes (Circle and 8-shape) by following a moving setpoint.

Robust Plate Detection: Uses a 2-stage (edge + color) process to lock onto the plate and crop the video feed, reducing computational load.

High-Accuracy Ball Detection: A 3-filter pipeline (Hough Circle + Color + Shape/Size) ensures only the target ball is tracked.

Full GUI Control: The interface.py script provides a graphical interface to:

Tune all PID gains (Kpx, Kpy, Kix, Kiy, Kdx, Kdy) in real-time.

Enable "Circle Movement" and "8-Shape Movement" modes.

Closed-Loop Control: A full PID controller calculates the error between the ball's position and the setpoint, sending precise correction angles to the hardware.

⚙️ How It Works
The system operates in a continuous loop, broken down into three main stages:

1. Plate Detection
Before any processing, the system first finds the balancing plate.

Edge Detection: A first pass is done to find the rectangular shape of the plate.

Color Filtering: The selection is refined using a yellow color filter (as the plate is yellow), ensuring the correct object is targeted.

Region of Interest (ROI) Cropping: The video feed is then trimmed to the size of the detected plate (with a 2-pixel offset) so that the ball detection algorithm only runs inside this clean, relevant area.

2. Ball Detection (3-Filter Pipeline)
To ensure only the correct ball is tracked, a 3-stage filter is applied:

Hough Circle Transform: This detects all circular objects within the cropped video frame.

Color Filter: It then filters these circles, keeping only the one that matches the specific color of the ball.

Shape & Size Filter: Finally, it validates the detected object's circularity and size, ignoring any "absurd" or misshapen circles that the camera might capture.

3. PID Control & Hardware Communication
Once the ball is detected, the control loop takes over:

Error Calculation: The system gets the ball's current (x, y) coordinates and measures the distance (error) to the setpoint (the blue dot).

PID Controller: This error is fed to the PID algorithm. Using the gains (Kpx, Kpy, etc.) set in the GUI, it calculates the precise tilt angles required for the X and Y axes to correct the error.

Serial Communication: The calculated angle values are sent via serial (at a fixed baud rate) to the connected Arduino.

Actuation: The Arduino, running the final_arduino.py logic, interprets these angles and commands the servos to tilt the plate, bringing the ball closer to the setpoint.

📂 File Structure
🛠 Hardware Required
Arduino (or compatible microcontroller)

2x Servo Motors (for X and Y-axis tilt)

The 2-axis ball and plate mechanism (custom-built)

A yellow plate (as the detection is hard-coded for yellow)

A ball with a distinct color (to be configured in the color filter)

A USB webcam

💻 Software & Setup
1. Dependencies
This project requires Python 3 and several libraries. You can install them using pip:

(You may also need a GUI library like Tkinter or PyQt5 if it's not already included in your Python installation).

2. Arduino Setup
Connect your Arduino to the computer.

Upload the logic from the final_arduino.py file to your board. (Note: Ensure the baud rate in interface.py matches the one set in your Arduino code).

3. Hardware Connection
Connect the two servo motors to the correct PWM pins on your Arduino as defined in final_arduino.py.

Assemble the plate mechanism.

Position the webcam above the plate to get a clear, top-down view.

▶️ Usage
Make sure all hardware is connected and the Arduino is running its code.

Run the main Python GUI from your terminal:

The GUI will launch, and the camera feed should appear.

The system will attempt to detect the plate and then the ball.

You can now tune the Kp, Ki, and Kd values to get stable balancing or enable the dynamic movement modes.
