
/*
 * Project: PID Based DC Motor Speed Control
 * Platform: Arduino Uno
 * Language: Embedded C++
 *
 * Features:
 * - Encoder feedback
 * - PWM motor control
 * - PID controller
 * - RPM measurement
 */

#define ENCODER_PIN 2
#define ENA_PIN 9
#define IN1_PIN 7
#define IN2_PIN 8

// Encoder pulses per revolution.
// Calibrate this value for your motor.
const float PULSES_PER_REV = 20.0;

const unsigned long SAMPLE_TIME = 100;

volatile unsigned long encoderPulses = 0;

float targetRPM = 120.0;
float actualRPM = 0.0;

// PID coefficients (initial tuning values)
float Kp = 1.5;
float Ki = 0.5;
float Kd = 0.02;

float error = 0;
float previousError = 0;
float integral = 0;
float derivative = 0;

float output = 0;

unsigned long previousTime = 0;

void encoderISR() {
    encoderPulses++;
}

void setup() {
    Serial.begin(9600);

    pinMode(ENCODER_PIN, INPUT_PULLUP);

    pinMode(ENA_PIN, OUTPUT);
    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);

    attachInterrupt(
        digitalPinToInterrupt(ENCODER_PIN),
        encoderISR,
        RISING
    );

    // Set motor direction
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);

    previousTime = millis();

    Serial.println("PID Motor Speed Control");
    Serial.println("Target RPM, Actual RPM, PWM");
}

void loop() {
    unsigned long currentTime = millis();

    if (currentTime - previousTime >= SAMPLE_TIME) {

        float dt = (currentTime - previousTime) / 1000.0;

        previousTime = currentTime;

        // Read encoder pulses safely
        noInterrupts();
        unsigned long pulses = encoderPulses;
        encoderPulses = 0;
        interrupts();

        // Calculate RPM
        actualRPM =
            (pulses / PULSES_PER_REV) * (60.0 / dt);

        // PID calculations
        error = targetRPM - actualRPM;

        integral += error * dt;
        integral = constrain(integral, -500, 500);

        derivative = (error - previousError) / dt;

        output = (Kp * error)
               + (Ki * integral)
               + (Kd * derivative);

        output = constrain(output, 0, 255);

        previousError = error;

        // Apply PWM output
        analogWrite(ENA_PIN, (int)output);

        // Serial monitoring
        Serial.print(targetRPM);
        Serial.print(", ");
        Serial.print(actualRPM);
        Serial.print(", ");
        Serial.println(output);
    }
}
