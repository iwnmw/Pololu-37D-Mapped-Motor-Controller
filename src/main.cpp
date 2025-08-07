#include <Arduino.h>
#include <Encoder.h>

// Define pin variables
const int INA = 2;
const int INB = 4;
const int PWM_PIN = 3;

// Define minimum, maximum speed and targeted speed (manually update speed while testing)
const int minSpeed = 65; // Minimum speed in RPM. Manually figured out via testing; motor cannot be driven below this speed
const int maxSpeed = 530; // Maximum speed in RPM
const int minPWM = 20; // Adjust this to the smallest PWM value that will still move the motor
const int maxPWM = 255;
float RPMTarget = 530; // Commanded speed in RPM

// Define variables for encoder reading and intialize the encoder object
const int encoderPinA = 5; // Must be interrupt capable; keep in mind if you change to another board
const int encoderPinB = 6;
Encoder myEnc(encoderPinA, encoderPinB);

// Definie values for converting the encoder reading to RPM
const int encoderCPR = 1200; // Counts per revolution of the encoder (output shaft)
unsigned long lastTime = 0;

int speedToPWM(float commandedSpeed) {
  // Generated a 3rd order polynomial to map RPM to PWM using experimental data
  // May need to be adjusted under load (was done unloaded) or for different motors
  float rpmMath = abs(commandedSpeed);
  if (rpmMath < minSpeed) {
    return minPWM; // Return minimum PWM if speed is below minimum
  } else if (rpmMath > maxSpeed) {
    return maxPWM; // Return maximum PWM if speed is above maximum
  } else {
    
    float pwm = 0.000004 * pow(rpmMath, 3) - 0.003 * pow(rpmMath, 2) + 0.7822 * rpmMath - 22.524;
    return constrain((int)pwm, minPWM, maxPWM); // Ensure PWM is within bounds (again) just to be safe
}
}

void sendMotorOutput(int pwmValue) {

  // Ensure speed command is within bounds
  if (abs(RPMTarget) > maxSpeed or abs(RPMTarget) < minSpeed) {
    Serial.println("Speed out of bounds, setting to 0");
    RPMTarget = 0; // Set to zero if out of bounds
  }

  // Determine direction first
  if (RPMTarget > 0) {
    digitalWrite(INA, HIGH);
    digitalWrite(INB, LOW);
  } else if (RPMTarget < 0) {
    digitalWrite(INA, LOW);
    digitalWrite(INB, HIGH);
  } else {
    // If RPMTarget is zero, stop the motor
    digitalWrite(INA, LOW);
    digitalWrite(INB, LOW);
  }
  analogWrite(PWM_PIN, pwmValue);
}


void setup() {
  Serial.begin(115200);

  // Initialize pins
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  Serial.println("Motor Control Initialized");
}

long oldPosition = 0;

void loop() {
  // put your main code here, to run repeatedly:
  long newPosition = myEnc.read();
  unsigned long currentTime = millis();
  float dt = currentTime - lastTime; // Time difference in millisecond
  long deltaPosition = newPosition - oldPosition;
  float RPMActual = -(deltaPosition / (float)encoderCPR) * (60000.0 / dt); // Convert encoder counts to RPM
  lastTime = currentTime;
  oldPosition = newPosition;

  // Compute the PWM value based on the target speed and send it to the motor
  sendMotorOutput(speedToPWM(RPMTarget));

  // Print the encoder position and the target speed
  Serial.print("Actual RPM: ");
  Serial.print(RPMActual);
  Serial.print(" Target RPM: ");  Serial.print(RPMTarget);
  Serial.print(" PWM Value: ");
  Serial.print(speedToPWM(RPMTarget));
  Serial.print(" Time: "); Serial.println(currentTime);

  // Small delay to reduce noise in encoder reading
  delay(100); // Adjust as necessary for your application
}
