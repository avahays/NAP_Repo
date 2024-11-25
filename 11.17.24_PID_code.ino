#include <math.h>
#include <PID_v1.h>

// Thermistor calculation function
double Thermistor(int RawADC) {
  const double V_in = 5.0;           // Supply voltage
  const double R_pullup = 10000.0;  // Pull-up resistor value in ohms
  const double Beta = 3950.0;       // Beta value from datasheet
  const double T0 = 298.15;         // Reference temperature in Kelvin (25°C)
  const double R0 = 10000.0;        // Resistance at T0 (10kΩ)

  // Calculate thermistor resistance
  double V_out = V_in * RawADC / 1023.0;
  double R_thermistor = R_pullup * (V_out / (V_in - V_out));

  // Calculate temperature in Kelvin using Beta equation
  double tempK = 1.0 / ((1.0 / T0) + (1.0 / Beta) * log(R_thermistor / R0));

  // Convert Kelvin to Celsius
  double temp = tempK - 273.15;

  return temp;
}

// Pins
#define THERMISTORPIN A0 // Thermistor pin
#define TEC_PWM_PIN 6    // MOSFET control pin (PWM)

// PID variables
double Setpoint, Input, Output;
double Kp = 1.0, Ki = 0.1, Kd = 0.05; // Initial PID tuning values

// Anti-windup variables
double integralSum = 0.0;  // Tracks the integral term manually

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE); // Reverse for TEC control

void setup() {
  pinMode(TEC_PWM_PIN, OUTPUT);
  
  Serial.begin(9600);

  // Initialize PID
  Setpoint = 18.0;                // Target temperature in Celsius
  myPID.SetOutputLimits(0, 255);  // Output limits match PWM range
  myPID.SetMode(AUTOMATIC);       // Turn PID on

  // Debug output
  Serial.println("System initialized.");
}

void loop() {
  // Read thermistor and calculate temperature
  float reading = analogRead(THERMISTORPIN);
  float temperature = Thermistor(reading);
  Input = temperature;

  // PID calculation
  double error = Setpoint - Input;

  // Implementing anti-windup for the integral term
  if (abs(error) > 0.5) { // Deadband to avoid overcompensating for minor errors
    integralSum += error * Ki; // Accumulate error into the integral term
    integralSum = constrain(integralSum, 0, 255); // Constrain integral term
  }

  // Calculate the PID output manually
  double proportional = Kp * error;
  double derivative = Kd * (error - Output); // Error rate of change
  Output = proportional + integralSum + derivative;

  // Constrain final output
  Output = constrain(Output, 0, 255);

  // Drive TEC using PWM
  analogWrite(TEC_PWM_PIN, (int)Output);

  // Debugging output
  //Serial.print("Temperature (C): ");
  //Serial.println(temperature);
  //Serial.print("Setpoint: ");
  //Serial.println(Setpoint);
  Serial.print("Error: ");
  Serial.println(error);
  Serial.print("Integral Sum: ");
  Serial.println(integralSum);
  Serial.print("Output (PWM): ");
  Serial.println(Output);

  delay(500); // Adjust delay for stable reading
}