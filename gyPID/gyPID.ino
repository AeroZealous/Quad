#include <GlobalDefined.h>
#include <Wire.h>
#include <AQMath.h>
#include <Device_I2C.h>
#include <Gyroscope_ITG3200.h>
#include <Accelerometer_BMA180.h>
#include <Receiver_328p.h> // for AeroQuad shield v1.x with Arduino Due/Uno and mini shield v1.0 using a standard PWM receiver
#include <Motors_PWM_Timer.h>
#include <FourtOrderFilter.h>
#include <Kinematics_ARG.h>
#include <Kinematics.h>
//Choose how many channels you have available (6, 8 or 10)
#define LASTCHANNEL 6

// -------------  End of configuration ----------------- //

#define FRONT_LEFT  MOTOR1
#define FRONT_RIGHT MOTOR2
#define REAR_RIGHT  MOTOR3
#define REAR_LEFT   MOTOR4

unsigned long timer;
//
bool simulated = false;
//The channel values read from RECEIVER
int rcThrottle, rcYaw, rcPitch, rcRoll, rcMode;
//The values read from the GYRO
int gyroll, gypitch, gyyaw;
//The values read from the ACCELEROMETER
float acx, acy, acz;
//The offset of accelerometer at ground level
float ac0x, ac0y, ac0z;
//The filtered acceleration
float filteredAccel[3] = {0.0, 0.0, 0.0};

//Compensation values calculated for various axes
int pitchF, rollL, yawCCW;
//D variables
int error_p_prev, error_r_prev, error_y_prev;
//I variable
int error_p_total, error_r_total, error_y_total;

//the maximum change in rcThrottle the pitch and roll sticks can make
#define PITCH_MAX   200
#define ROLL_MAX    200

#define constrainPPM(x) (constrain((x), 1000,2000))
float x0, y0, z0;

/*************** for accelerometer*****/
float target_roll = 0, target_pitch = 0, target_yaw = 0;
float roll_angle_prev, pitch_angle_prev = 0, yaw_angle_prev = 0;

/******************/


void setup() {
    Serial.begin(115200);
    initializeReceiver(LASTCHANNEL);
    initializeMotors(FOUR_Motors);
    receiverXmitFactor = 1.0;

    Wire.begin();
    initializeGyro();
    initializeAccel();
    calibrateGyro();
    computeAccelBias();
    accelScaleFactor[XAXIS] = accelScaleFactor[YAXIS] = accelScaleFactor[ZAXIS] = -0.047;
    measureAccel();
    //    pitch_angle_prev = degrees(atan2(accel_x, sqrt(pow(accel_y, 2) + pow(accel_z, 2))));
    //    roll_angle_prev = degrees(atan2(accel_y, sqrt(pow(accel_z, 2) + pow(accel_x, 2))));
    //    yaw_angle_prev = degrees(atan2(accel_z, sqrt(pow(accel_y, 2) + pow(accel_x, 2))));
    setupFourthOrder();
    initializeKinematics();
}

#define sendData(x) Serial.print(" "); \
                    Serial.print(x);  \
                    Serial.print(" ");
char query = 'x';

/**
 * Call this function at start of loop to check for incoming commands that may override sensors data
 */
void serialReceive() {
    if (Serial.available()) {
        query = Serial.read();
    }
    //calibrate gyro
    if (query == 'c') {
        calibrateGyro();
        query = 'g';
    }
    //gyro zero
    if (query == 'g') {
        sendData("gyro_zero");
        sendData(gyroZero[XAXIS]);
        sendData(gyroZero[YAXIS]);
        sendData(gyroZero[ZAXIS]);
        query = 'x';
        return;
    }
    if (query == 'O') {
        simulated = !simulated;
        sendData("Simulation");
        sendData(simulated);
        query = 'x';
    }
}

/**
   Call this function at loop end to send data to ground station. Follow the format for ground station to be able to parse it.
 */
void serialTransmit() {
    //Timing
    long timedelta = millis() - timer;

    if (query == 'x')return;
    if (query != 's')return;

    sendData("{"); //packet start header delimiter
    /*----TIMING----*/
    sendData("ms"); //time header, also time unit
    sendData(timedelta);
    /*----GYRO----*/
    sendData("gy");
    sendData(degrees(gyroRate[XAXIS]));
    sendData(degrees(gyroRate[YAXIS]));
    sendData(degrees(gyroRate[ZAXIS]));
    /*----RAW ACCELEROMETER----*/
    sendData("ac");
    sendData(meterPerSecSec[XAXIS]);
    sendData(meterPerSecSec[YAXIS]);
    sendData(meterPerSecSec[ZAXIS]);
    /*----FILTERED ACCELEROMETER----*/
    sendData("acfil");
    sendData(filteredAccel[XAXIS]);
    sendData(filteredAccel[YAXIS]);
    sendData(filteredAccel[ZAXIS]);
    /*----ABSOLUTE ANGLE----*/
    sendData("ang");
    sendData(degrees(kinematicsAngle[XAXIS]));
    sendData(degrees(kinematicsAngle[YAXIS]));
    sendData(degrees(kinematicsAngle[ZAXIS]));
    /*----RECEIVER COMMANDS----*/
    sendData("rc6");
    sendData(rcRoll);
    sendData(rcPitch);
    sendData(rcYaw);
    sendData(rcThrottle);
    sendData(rcMode);
    /*----MOTOR COMMANDS----*/
    sendData("motor");
    sendData(motorCommand[FRONT_LEFT]);
    sendData(motorCommand[FRONT_RIGHT]);
    sendData(motorCommand[REAR_RIGHT]);
    sendData(motorCommand[REAR_LEFT]);
    sendData("}");
    Serial.println();
    /*
     * Network load : //TODO
     */
}

void loop() {

    if ((millis() - timer) > 20) { //50Hz
        timer = millis();

        readReceiver();
        measureGyro();
        measureAccel();

        rcRoll = receiverCommand[XAXIS];
        rcPitch = receiverCommand[YAXIS];
        rcYaw = receiverCommand[ZAXIS];
        rcThrottle = receiverCommand[THROTTLE];
        rcMode = receiverCommand[MODE];

        serialReceive();
        if (simulated) {
            rcMode = 2000;
        }

        gyroll = degrees(gyroRate[XAXIS]);
        gypitch = degrees(gyroRate[YAXIS]);
        gyyaw = degrees(gyroRate[ZAXIS]);

        acx = meterPerSecSec[XAXIS];
        acy = meterPerSecSec[YAXIS];
        acz = meterPerSecSec[ZAXIS];

        for (int axis = XAXIS; axis <= ZAXIS; axis++) {
            filteredAccel[axis] = computeFourthOrder(meterPerSecSec[axis], &fourthOrder[axis]);
        }
        calculateKinematics(gyroRate[XAXIS], gyroRate[YAXIS], gyroRate[ZAXIS], filteredAccel[XAXIS], filteredAccel[YAXIS], filteredAccel[ZAXIS], 0.02);

        pitchF = 0;
        rollL = 0;
        yawCCW = 0;

        target_pitch = map(rcPitch, 1000, 2000, -30, 30);
        target_roll = map(rcRoll, 1000, 2000, -30, 30);

        //err_pitch = map(target_pitch - pitch_angle, 0, 45, 0, 100);
        //err_roll = map(target_roll - roll_angle, 0, 45, 0, 100);


        /* Proportional
           Short summary: The larger the error, the harder it tries to compensate.
           Thus the compensation is *proportional* to the error
         */
#define P 1.5
        pitchF += gypitch * P; //+ve gypitch indicates forward dive, so increase forward thrust
        rollL -= gyroll * P; //+ve gyroll indicates right side dive, so compensate by increasing right thrust
#define Py 3
        //+ve gyyaw indicates anticlockwise drift of the body, so compensate by increasing anticlockwise motors speed
        yawCCW += gyyaw * Py; // thus inducing anti-clockwise reaction on the body


        /* Derivative
           Short summary: It tries to reduce the rate of change of error, for better or worse
           It acts as friction to reduce overcompensation by P code
         */
#define D 2
        int delta_pitch = gypitch - error_p_prev;
        int delta_roll = gyroll - error_r_prev;
        int delta_yaw = gyyaw - error_y_prev;

        pitchF += delta_pitch * D;
        //        pitchR -= delta_pitch * D;

        rollL -= delta_roll * D;
        //        rollR += delta_roll * D;
        yawCCW += delta_yaw * 0.1;

        error_p_prev = gypitch;
        error_r_prev = gyroll;
        error_y_prev = gyyaw;

        /* Integral
           Short summary: It remembers the bias due to error and tries to equal the +ve and -ve errors.
           It is possible that at a slanted orientation the gyro readings are zero and hence both P and D become inactive.
           But I remembers that there is error from original position which has not been compensated and acts in that direction.
         */
        /*TODO : THINKING ABOUT REPLACING THIS WITH ABSOLUTE ANGLE*/
#define I 4
#define I_MAX 100
//        error_p_total += gypitch * 0.005;
//        pitchF += constrain(I * error_p_total, -I_MAX, I_MAX);
//
//        error_r_total += gyroll * 0.005;
//        rollL -= constrain(I * error_r_total, -I_MAX, I_MAX);
        
        pitchF += constrain(I * (degrees(kinematicsAngle[YAXIS]) - target_pitch), -I_MAX, I_MAX );
        rollL -= constrain(I * (degrees(kinematicsAngle[XAXIS]) - target_roll), -I_MAX, I_MAX);

        /******************* TODO: ABSOLUTE ANGLE *************************/


        /***************************************************************/

        /* writing into the motors : last step */
        if (rcMode < 1500) { //Safety Check
            //MOTORS
            motorCommand[FRONT_LEFT] = 1000;
            motorCommand[FRONT_RIGHT] = 1000;
            motorCommand[REAR_LEFT] = 1000;
            motorCommand[REAR_RIGHT] = 1000;

            //Also zero the integral error
            error_p_total = error_r_total = 0;
            error_y_total = 0;
        } else {
            motorCommand[FRONT_LEFT] = constrainPPM(rcThrottle + pitchF + rollL - yawCCW);
            motorCommand[FRONT_RIGHT] = constrainPPM(rcThrottle + pitchF - rollL + yawCCW);
            motorCommand[REAR_LEFT] = constrainPPM(rcThrottle - pitchF + rollL + yawCCW);
            motorCommand[REAR_RIGHT] = constrainPPM(rcThrottle - pitchF - rollL - yawCCW);
        }
        writeMotors();
        serialTransmit();
    }
}
