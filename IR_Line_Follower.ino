/*****************************************************
Line Follower for NXP Cup - First Release
Made by: Ponta Ciprian Ioan

Useful links:
https://www.instructables.com/Line-Follower-Robot-PID-Control-Android-Setup/
*****************************************************/

#include <Pixy2.h>

#define SWITCH_PIN              2

#define RIGHT_MOTOR_ENABLE      10
#define RIGHT_MOTOR_DIRECTION   12
#define LEFT_MOTOR_ENABLE       11
#define LEFT_MOTOR_DIRECTION    13

#define MAX_SPEED               255
#define STOP_SPEED              0

#define FRONT_DIRECTION         LOW
#define BACK_DIRECTION          HIGH

#define SENSOR_NUMBER           5

#define SENSOR_THRESHOLD_VALUE  50

#define ANGLE_SETPOINT          0
#define SAMPLE_TIME             50
#define KP                      6.4
#define KD                      0.04

Pixy2 pixy;
const int sensors[] = {A0, A1, A2, A3, A4};     // Used Analog Pins

void setup() 
{
    Serial.begin(9600);
    
    pinMode(SWITCH_PIN, INPUT);
    
    pinMode(RIGHT_MOTOR_ENABLE,     OUTPUT);
    pinMode(RIGHT_MOTOR_DIRECTION,  OUTPUT);
    pinMode(LEFT_MOTOR_ENABLE,      OUTPUT);
    pinMode(LEFT_MOTOR_DIRECTION,   OUTPUT);
    
    digitalWrite(RIGHT_MOTOR_DIRECTION, FRONT_DIRECTION);
    digitalWrite(LEFT_MOTOR_DIRECTION,  FRONT_DIRECTION);
    
    pixy.init();
}

void getDigitalValueFromSensors(int sensorsDigitalValue[])
{
    // Analog to Digital Conversion
    
    for(int i = 0; i < SENSOR_NUMBER; i++)
    {
        int value = analogRead(sensors[i]);
        
        if(value < SENSOR_THRESHOLD_VALUE)
        {
            sensorsDigitalValue[i] = 0;
        }
        else
        {
            sensorsDigitalValue[i] = 1;
        }
    }
}

int getAngle(int sensorsDigitalValue[])
{
    // Estimation of angle from certain sensor positions
    
    if(sensorsDigitalValue[0] == 0 &&           // On S0
       sensorsDigitalValue[1] == 1 &&
       sensorsDigitalValue[2] == 1 &&
       sensorsDigitalValue[3] == 1 &&
       sensorsDigitalValue[4] == 1)
    {
        return (-45);                           
    }
    else if(sensorsDigitalValue[0] == 0 &&      // Between S0 and S1
            sensorsDigitalValue[1] == 0 &&
            sensorsDigitalValue[2] == 1 &&
            sensorsDigitalValue[3] == 1 &&
            sensorsDigitalValue[4] == 1)
    {
        return (-40);                              
    }
    else if(sensorsDigitalValue[0] == 1 &&      // On S1
            sensorsDigitalValue[1] == 0 &&
            sensorsDigitalValue[2] == 1 &&
            sensorsDigitalValue[3] == 1 &&
            sensorsDigitalValue[4] == 1)
    {
        return (-30);                               
    }
    else if(sensorsDigitalValue[0] == 1 &&      // Between S1 and S2
            sensorsDigitalValue[1] == 0 &&
            sensorsDigitalValue[2] == 0 &&
            sensorsDigitalValue[3] == 1 &&
            sensorsDigitalValue[4] == 1)
    {
        return (-20);                               
    }
    else if(sensorsDigitalValue[0] == 1 &&      // On S2
            sensorsDigitalValue[1] == 1 &&
            sensorsDigitalValue[2] == 0 &&
            sensorsDigitalValue[3] == 1 &&
            sensorsDigitalValue[4] == 1)
    {
        return 0;                               
    }
    else if(sensorsDigitalValue[0] == 1 &&      // Between S2 and S3
            sensorsDigitalValue[1] == 1 &&
            sensorsDigitalValue[2] == 0 &&
            sensorsDigitalValue[3] == 0 &&
            sensorsDigitalValue[4] == 1)
    {
        return 20;                               
    }
    else if(sensorsDigitalValue[0] == 1 &&      // On S3
            sensorsDigitalValue[1] == 1 &&
            sensorsDigitalValue[2] == 1 &&
            sensorsDigitalValue[3] == 0 &&
            sensorsDigitalValue[4] == 1)
    {
        return 30;                               
    }
    else if(sensorsDigitalValue[0] == 1 &&      // Between S3 and S4
            sensorsDigitalValue[1] == 1 &&
            sensorsDigitalValue[2] == 1 &&
            sensorsDigitalValue[3] == 0 &&
            sensorsDigitalValue[4] == 0)
    {
        return 40;                               
    }
    else if(sensorsDigitalValue[0] == 1 &&      // On S4
            sensorsDigitalValue[1] == 1 &&
            sensorsDigitalValue[2] == 1 &&
            sensorsDigitalValue[3] == 1 &&
            sensorsDigitalValue[4] == 0)
    {
        return 45;                               
    }
    else
    {
        return 360;                             // Out of track, Continue previous move
    }
}


void showSensorValues()                         
{
    // Used for debugging
    
    for(int i = 0; i < SENSOR_NUMBER; i++)
    {
        Serial.print("Sensor " + String(i) + ": " + String(analogRead(sensors[i])) + "\t");
    }
    Serial.println();
    delay(1000);
}

void setMotorSpeed(int leftSpeed, int rightSpeed)
{
    // Used for setting motor speed
    
    digitalWrite(LEFT_MOTOR_ENABLE,  leftSpeed);
    digitalWrite(RIGHT_MOTOR_ENABLE, rightSpeed);
}

void stopRobot()
{
    // Used for stoping the robot
    
    setMotorSpeed(0, 0);
}

void motorDriver(double output)
{
    // The output from the pid is checked to be inbetween [-255, 255],
    // and according to the output value, an offset is applied in order to
    // slow a certain motor so that the robot can take the turn.
    
    int leftOffset = 0;
    int rightOffset = 0;
    
    if(output < 0)
    {
        if(output < -255)
        {
            output = -255;
        }
        
        leftOffset = 0;
        rightOffset = -(int)output;
    }
    else
    {
        if(output > 255)
        {
            output = 255;
        }
        
        leftOffset = (int)output;
        rightOffset = 0;
    }
    
    setMotorSpeed(MAX_SPEED - leftOffset, MAX_SPEED - rightOffset);
}

void pidControl(int angle)
{
    // PD Algorithm is used for controlling the robot
    
    static double prevError = 0;
    double error;
    double derivative;
    double output;
    
    if(angle != 360)
    {
        error = (double)(ANGLE_SETPOINT - angle);
        derivative = (error - prevError) / (double)SAMPLE_TIME;
        output = KP * error + KD * derivative;
        prevError = error;

        motorDriver(output);
    }
    else
    {
        // Do nothing, continue previous move
    }
    
    /* Used for Debugging */
    // Serial.print("Angle: "      + String(angle)         + "\t");
    // Serial.print("Error: "      + String(error)         + "\t");
    // Serial.print("Derivative: " + String(derivative)    + "\t");
    // Serial.print("PrevError: "  + String(prevError)     + "\t");
    // Serial.print("Output: "     + String(output)        + "\t");
    // Serial.println();
}

void loop()
{   
    // We use a jumper fire as a switch
    // 5V connected      to D2 => The robot is stopped
    // 5V disconnected from D2 => The robot is following the black line
    
    if(digitalRead(SWITCH_PIN) == LOW)
    {
        int sensorsDigitalValue[SENSOR_NUMBER]; 
        getDigitalValueFromSensors(sensorsDigitalValue);
        pidControl(getAngle(sensorsDigitalValue));
        delay(SAMPLE_TIME);
    }
    else
    {
        // showSensorValues();
        stopRobot();
    }
}
