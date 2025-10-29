// Include the Servo library
#include <Servo.h>

// Define the pins for the two servo motors
#define SERVO_X_PIN 9
#define SERVO_Y_PIN 10

// Create servo objects to control the servos
Servo servoX;
Servo servoY;

void setup() {
  // Start serial communication at 9600 bits per second
  Serial.begin(9600);
  
  // Attach the servo objects to their respective pins
  servoX.attach(SERVO_X_PIN);
  servoY.attach(SERVO_Y_PIN);
  
  // Initialize the servos to the neutral (level) position
  servoX.write(90);
  servoY.write(90);
  
  Serial.println("Arduino ready to receive servo angles.");
}

void loop() {
  // Check if data is available to read from the serial port
  if (Serial.available() > 0) {
    // Read the incoming string from Python until a newline character is found
    String data = Serial.readStringUntil('\n');
    
    // Check if the received data is in the expected format (e.g., "<95,88>")
    if (data.startsWith("<") && data.endsWith(">")) {
      
      // Remove the start and end markers
      data = data.substring(1, data.length() - 1);
      
      // Find the comma that separates the two angle values
      int commaIndex = data.indexOf(',');
      
      // Extract the string for the X angle and the Y angle
      String angleX_str = data.substring(0, commaIndex);
      String angleY_str = data.substring(commaIndex + 1);
      
      // Convert the angle strings to integersgive
      int angleX = angleX_str.toInt();
      int angleY = angleY_str.toInt();
      
      // Constrain the values to a safe range (0-180) to prevent servo damage
      angleX = constrain(angleX, 0, 180);
      angleY = constrain(angleY, 0, 180);
      
      // Write the final angles to the servos
      servoX.write(angleX);
      servoY.write(angleY);
    }
  }
}

