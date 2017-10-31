package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by hfu on 10/17/17.
 */

public class Navigation {

    float distance;
    float speed;
    float heading;
    long lastDistanceTimestamp;
    long lastSpeedTimestamp;
    long lastHeadingTimestamp;

    PIDControl pidControlDistance;
    PIDControl pidControlSpeed;
    PIDControl pidControlHeading;

    int turnState = 0;
    int turnDistanceRightWheel = 0;
    int leftWheelLandMark = 0;
    int rightWheelLandMark= 0;
    float currentTurnAngle =0.0f;
    int convergeCount = 0;
    float angleErrorTolerance = 0.5f;

    Telemetry telemetry = null;

    public Navigation(Telemetry t) {

        // distance control
        pidControlDistance = new PIDControl();
        pidControlDistance.setKp(0.001f);
        pidControlDistance.setKi(0.001f);
        pidControlDistance.setKd(0.001f);
        pidControlDistance.setMaxIntegralError(2.0f/pidControlDistance.fKp);

        // speed control
        pidControlSpeed = new PIDControl();
        pidControlSpeed.setKp(0.001f);
        pidControlSpeed.setKi(0.001f);
        pidControlSpeed.setKd(0.001f);
        pidControlSpeed.setMaxIntegralError(2.0f/pidControlSpeed.fKp);

        //heading control
        pidControlHeading = new PIDControl();
        pidControlHeading.setKp(0.05f);
        pidControlHeading.setKi(0.001f);
        pidControlHeading.setKd(0.001f);
        pidControlHeading.setMaxIntegralError(2.0f/pidControlHeading.fKp);

        // update time stamp
        lastDistanceTimestamp= System.currentTimeMillis();
        lastSpeedTimestamp=lastDistanceTimestamp;
        lastHeadingTimestamp=lastDistanceTimestamp;

        // other objects
        telemetry = t;
    }

    /**
     return power
     */
    public float getMaintainDistancePower(float targetDistance) {
        long lastT = lastDistanceTimestamp;
        lastDistanceTimestamp = System.currentTimeMillis();
        return pidControlDistance.update(targetDistance-distance, lastDistanceTimestamp-lastT);
    }

    /**
     return power
     */
    public float getMaintainSpeedPower(float targetSpeed) {
        long lastT = lastSpeedTimestamp;
        lastSpeedTimestamp = System.currentTimeMillis();
        return pidControlSpeed.update(targetSpeed-speed, lastSpeedTimestamp-lastT);
    }

    /**
    return power
     */
    public float getMaintainHeadingPower(float targetHeading) {
        long lastT = lastHeadingTimestamp;
        lastHeadingTimestamp = System.currentTimeMillis();
        return pidControlHeading.update(targetHeading-heading, lastHeadingTimestamp-lastT);
    }

    public void resetDistanceControl(){
        lastDistanceTimestamp = System.currentTimeMillis();
        pidControlDistance.reset();
    }

    public void resetSpeedControl() {
        lastSpeedTimestamp = System.currentTimeMillis();
        pidControlSpeed.reset();
    }

    public void resetHeadingControl() {
        lastHeadingTimestamp = System.currentTimeMillis();
        pidControlHeading.reset();
    }

    public void resetTurn(DcMotor [] leftMs, DcMotor [] rightMs){
        leftWheelLandMark  =0;
        for ( int i =0; i < leftMs.length; i++ ){
            leftWheelLandMark += leftMs[i].getCurrentPosition();
        }
        leftWheelLandMark /= leftMs.length;
        rightWheelLandMark  =0;
        for ( int i =0; i < rightMs.length; i++ ){
            rightWheelLandMark += rightMs[i].getCurrentPosition();
        }
        rightWheelLandMark /= rightMs.length;
        resetHeadingControl();

        convergeCount =0;
        turnState =0;
    }

    public int turnByEncoderOpenLoop (double power, float angle,
                                      float axleLength,
                                      DcMotor [] leftMs, DcMotor[] rightMs) {
        switch (turnState) {
            case 0:
                // compute encoder distance
                turnDistanceRightWheel = (int)getTurnDistanceRightWheel (angle,axleLength);
                turnState = 1;
                break;
            case 1:
                // wait until encoders reach the distance
                int lD =0;
                for ( int i =0; i < leftMs.length; i++ ){
                    lD += leftMs[i].getCurrentPosition();
                }
                lD = lD/leftMs.length - leftWheelLandMark;
                boolean leftDone= false;
                if (-turnDistanceRightWheel > 0) {
                    if (lD > -turnDistanceRightWheel) {
                        for ( int i =0; i < leftMs.length; i++ ) {
                            leftMs[i].setPower(0.0);
                        }
                        leftDone = true;
                    } else {
                        for ( int i =0; i < leftMs.length; i++ ) {
                            leftMs[i].setPower(-power);
                        }
                    }
                } else {
                    if (lD < -turnDistanceRightWheel) {
                        for ( int i =0; i < leftMs.length; i++ ) {
                            leftMs[i].setPower(0.0);
                        }
                        leftDone = true;
                    } else {
                        for ( int i =0; i < leftMs.length; i++ ) {
                            leftMs[i].setPower(power);
                        }
                    }
                }
                int rD = 0;
                for ( int i =0; i < rightMs.length; i++ ) {
                    rD += rightMs[i].getCurrentPosition();
                }
                rD = rD/rightMs.length - rightWheelLandMark;
                boolean rightDone= false;
                if (turnDistanceRightWheel > 0) {
                    if (rD > turnDistanceRightWheel) {
                         for ( int i =0; i < rightMs.length; i++ ) {
                             rightMs[i].setPower(0.0);
                         }
                        rightDone = true;
                    } else {
                         for ( int i =0; i < rightMs.length; i++ ) {
                             rightMs[i].setPower(power);
                         }
                    }
                } else {
                    if (rD < turnDistanceRightWheel) {
                        for ( int i =0; i < leftMs.length; i++ ) {
                            rightMs[i].setPower(0.0);
                        }
                        rightDone = true;
                    } else {
                         for ( int i =0; i < rightMs.length; i++ ) {
                             rightMs[i].setPower(-power);
                         }
                    }
                }
                if ( leftDone && rightDone) {
                    turnState = 2;
                }
                break;
            default:
                return 0;
        }
        return 1;
    }

    public int turnByEncoderCloseLoop (double power, float targetAngle,
                                      float axleLength,
                                      DcMotor [] leftMs, DcMotor [] rightMs) {
        switch (turnState) {
            case 0:
                // get current turn angle
                int lD = 0;
                for ( int i =0; i < leftMs.length; i++ ) {
                    lD += leftMs[i].getCurrentPosition();
                }
                lD = lD/leftMs.length - leftWheelLandMark;
                int rD = 0;
                for ( int i =0; i < rightMs.length; i++ ) {
                    rD += rightMs[i].getCurrentPosition();
                }
                rD = rD/rightMs.length-rightWheelLandMark;
                currentTurnAngle = getTurnAngle (lD, rD, axleLength);
                turnState = 1;
                break;
            case 1:

                float errorAngle = (targetAngle-currentTurnAngle)%360;

                if (Math.abs(errorAngle) < angleErrorTolerance) {
                    convergeCount ++;
                } else {
                    convergeCount = 0;
                }

                if (convergeCount > 3) {
                    turnState = 1;
                }

                // adjust power
                long currentT = System.currentTimeMillis();
                float powerDiff = pidControlHeading.update(errorAngle,
                        currentT-lastHeadingTimestamp);
                lastHeadingTimestamp = currentT;
                for ( int i =0; i < leftMs.length; i++ ) {
                   leftMs[i].setPower(power+powerDiff);
                }
                for ( int i =0; i < rightMs.length; i++ ) {
                   rightMs[i].setPower(power-powerDiff);
                }
                break;
            default:
                return 0;
        }
        return 1;
    }

    public int turnByGyroCloseLoop (double power, float currentAngle,
                                    float targetAngle, float axleLength,
                                    DcMotor [] leftMs, DcMotor [] rightMs) {
        switch (turnState) {
            case 0:
                float errorAngle = (targetAngle-currentAngle)%360;

                if (Math.abs(errorAngle) < angleErrorTolerance) {
                    convergeCount ++;
                } else {
                    convergeCount = 0;
                }

                if (convergeCount > 3) {
                    turnState = 1;
                }

                // adjust power
                long currentT = System.currentTimeMillis();
                float powerDiff = pidControlHeading.update(errorAngle,
                        currentT-lastHeadingTimestamp);
                lastHeadingTimestamp = currentT;
                for ( int i =0; i < leftMs.length; i++ ) {
                    leftMs[i].setPower(power+powerDiff);
                }
                for ( int i =0; i < rightMs.length; i++ ) {
                    rightMs[i].setPower(power-powerDiff);
                }
                break;
            default:
                return 0;
        }
        return 1;
    }

    // left turn is positive
    float getTurnAngle (float leftDistance, float rightDistance, float axleDistance){
        return (float)((360 * rightDistance * (rightDistance - leftDistance)) / (2 * 3.14 * axleDistance * rightDistance));
    }

    // if the result is positive, set right Motor move forward (left turn),
    // other wise (right turn)
    float getTurnDistanceRightWheel (float angle, float axleDistance){
        return (float)(axleDistance * angle * 3.14) / 360;
    }

}
