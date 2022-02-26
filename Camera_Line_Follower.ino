/*****************************************************
Line Follower using camera for NXP Cup - First Release
Made by: Ponta Ciprian Ioan

Useful links:
https://www.instructables.com/Line-Follower-Robot-PID-Control-Android-Setup/
*****************************************************/

#include <Pixy2.h>

#define RIGHT_MOTOR_ENABLE      5
#define LEFT_MOTOR_ENABLE       6

#define MAX_SPEED               255
#define STOP_SPEED              0

#define ANGLE_SETPOINT          0
#define SAMPLE_TIME             50
#define KP                      15
#define KI                      0
#define KD                      0

Pixy2 pixy;

void setup() 
{
    Serial.begin(115200);
    
    pinMode(RIGHT_MOTOR_ENABLE, OUTPUT);
    pinMode(LEFT_MOTOR_ENABLE,  OUTPUT);
    
    pixy.init();
    pixy.changeProg("line");
}

void setMotorSpeed(int leftSpeed, int rightSpeed)
{
    // Used for setting motor speed
    
    analogWrite(LEFT_MOTOR_ENABLE,  leftSpeed);
    analogWrite(RIGHT_MOTOR_ENABLE, rightSpeed);
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
    
    Serial.print("("+ String(MAX_SPEED - leftOffset) + ", " + String(MAX_SPEED - rightOffset) + ")" + "\t");
}

void pidControl(int32_t angle)
{
    // PID Algorithm is used for controlling the robot
    
    static double errorSum = 0;
    static double prevError = 0;
    
    
    double error = (double)(ANGLE_SETPOINT - angle);
    errorSum += (error * (double)SAMPLE_TIME);
    double derivative = (error - prevError) / ((double)SAMPLE_TIME / 1000);
    double output = (double)KP * error + (double)KI * errorSum + (double)KD *  derivative;
    prevError = error;
    
    motorDriver(output);
    
    /* Used for Debugging */
    Serial.print("Angle: "      + String(angle)         + "\t");
    Serial.print("Error: "      + String(error)         + "\t");
    Serial.print(" Error Sum: " + String(error)         + "\t");
    Serial.print("Derivative: " + String(derivative)    + "\t");
    Serial.print("PrevError: "  + String(prevError)     + "\t");
    Serial.print("Output: "     + String(output)        + "\t");
    Serial.println();
}

void loop()
{   
    int8_t results = pixy.line.getAllFeatures();
    Serial.println(results);
    
    if(results & LINE_VECTOR)
    {
        int32_t angleError = (int32_t)pixy.line.vectors->m_x1 - (int32_t)(pixy.frameWidth / 2);
        pidControl(angleError);
        delay(SAMPLE_TIME);
    }
}
