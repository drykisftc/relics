package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by hfu on 10/17/17.
 */

public class Navigation {

    double distance;
    double speed;
    double heading;
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
    double currentTurnAngle =0.0f;
    int convergeCount = 0;
    double angleErrorTolerance = 1.1f;
    int convergeCountThreshold = 4;

    double maxTurnDeltaPower = 0.5;

    Telemetry telemetry = null;

    public Navigation(Telemetry t) {

        // distance control
        pidControlDistance = new PIDControl();
        pidControlDistance.setKp(0.01f);
        pidControlDistance.setKi(0.005f);
        pidControlDistance.setKd(0.0000001f);
        pidControlDistance.setMaxIntegralError(2.0f/pidControlDistance.fKi);

        // speed control
        pidControlSpeed = new PIDControl();
        pidControlSpeed.setKp(0.001f);
        pidControlSpeed.setKi(0.001f);
        pidControlSpeed.setKd(0.001f);
        pidControlSpeed.setMaxIntegralError(2.0f/pidControlSpeed.fKi);

        //heading control
        pidControlHeading = new PIDControl();
        pidControlHeading.setKp(0.01f);//0.004
        pidControlHeading.setKi(0.02f);
        pidControlHeading.setKd(0.00000001f);
        pidControlHeading.setMaxIntegralError(0.6f/pidControlHeading.fKi);

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
    public double getMaintainDistancePower(double targetDistance, double currentDistance) {
        long lastT = lastDistanceTimestamp;
        lastDistanceTimestamp = System.currentTimeMillis();
        return pidControlDistance.update(targetDistance-currentDistance, lastDistanceTimestamp-lastT);
    }

    /**
     return power
     */
    public double getMaintainSpeedPower(double targetSpeed) {
        long lastT = lastSpeedTimestamp;
        lastSpeedTimestamp = System.currentTimeMillis();
        return pidControlSpeed.update(targetSpeed-speed, lastSpeedTimestamp-lastT);
    }

    /**
    return power
     */
    public double getMaintainHeadingPower(double targetHeading) {
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

    public int turnByEncoderOpenLoop (double p, double angle,
                                      double axleLength,
                                      DcMotor [] leftMs, DcMotor[] rightMs) {
        double power = Math.abs(p);
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
                            leftMs[i].setPower(power);
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
                            leftMs[i].setPower(-power);
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
                for ( int i =0; i < leftMs.length; i++ ) {
                    leftMs[i].setPower(0);
                }
                for ( int i =0; i < rightMs.length; i++ ) {
                    rightMs[i].setPower(0);
                }
                return 0;
        }
        return 1;
    }

    public int turnByEncoderCloseLoop (double power, double targetAngle,
                                      double axleLength,
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

                double errorAngle = (double)getAngleError(targetAngle,currentTurnAngle);

                if (Math.abs(errorAngle) < angleErrorTolerance) {
                    convergeCount ++;
                } else {
                    convergeCount = 0;
                }

                if (convergeCount > convergeCountThreshold) {
                    turnState = 1;
                }

                // adjust power
                long currentT = System.currentTimeMillis();
                double powerDiff = pidControlHeading.update(errorAngle,
                        currentT-lastHeadingTimestamp);
                lastHeadingTimestamp = currentT;
                for ( int i =0; i < leftMs.length; i++ ) {
                   leftMs[i].setPower(power-powerDiff);
                }
                for ( int i =0; i < rightMs.length; i++ ) {
                   rightMs[i].setPower(power+powerDiff);
                }
                break;
            default:
                for ( int i =0; i < leftMs.length; i++ ) {
                    leftMs[i].setPower(0);
                }
                for ( int i =0; i < rightMs.length; i++ ) {
                    rightMs[i].setPower(0);
                }
                return 0;
        }
        return 1;
    }

    public int turnByGyroCloseLoop (double power, double currentA,
                                    double targetA,
                                    DcMotor [] leftMs, DcMotor [] rightMs) {
        double currentAngle = normalizeHeading(currentA);
        double targetAngle = normalizeHeading(targetA);
        switch (turnState) {
            case 0:
                double errorAngle = (double)getAngleError(targetAngle,currentAngle);

                if (Math.abs(errorAngle) < angleErrorTolerance) {
                    convergeCount ++;
                } else {
                    convergeCount = 0;
                }

                if (convergeCount > convergeCountThreshold) {
                    turnState = 1;
                } else {

                    // adjust power
                    long currentT = System.currentTimeMillis();
                    double powerDiff = Range.clip(pidControlHeading.update(errorAngle,
                            (currentT - lastHeadingTimestamp) / 1000.0f), -maxTurnDeltaPower, maxTurnDeltaPower); // use seconds
                    lastHeadingTimestamp = currentT;
                    for (int i = 0; i < leftMs.length; i++) {
                        leftMs[i].setPower(Range.clip(power - powerDiff, -1.0, 1.0));
                    }
                    for (int i = 0; i < rightMs.length; i++) {
                        rightMs[i].setPower(Range.clip(power + powerDiff, -1.0, 1.0));
                    }
                    telemetry.addData("delta power", powerDiff);
                }
                telemetry.addData("error:" , errorAngle);

                break;
            default:
                for ( int i =0; i < leftMs.length; i++ ) {
                    leftMs[i].setPower(0);
                }
                for ( int i =0; i < rightMs.length; i++ ) {
                    rightMs[i].setPower(0);
                }
                return 0;
        }
        telemetry.addData("turnState:", turnState);
        telemetry.addData("Target   :", targetAngle);
        telemetry.addData("Heading  :", currentAngle);

        return 1;
    }

    // left turn is positive
    double getTurnAngle (double leftDistance, double rightDistance, double axleDistance){
        return (double)((360 * rightDistance * (rightDistance - leftDistance)) / (2 * 3.14 * axleDistance * rightDistance));
    }

    // if the result is positive, set right Motor move forward (left turn),
    // other wise (right turn)
    double getTurnDistanceRightWheel (double angle, double axleDistance){
        return (double)(axleDistance * angle * 3.14) / 360;
    }

    double normalizeHeading360 (double value) {
        // set it to [-360, 360]
        double v = value/360.0;
        double v2 = (v-(int)v) * 360.0;

        // set it in [0,360]
        while (v2 < 0)  v2 += 360;
        return v2;
    }

    // calculate error in -179 to +180 range  (
    double getAngleError(double targetAngle, double currentAngle) {

        double robotError;
        robotError = targetAngle - currentAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    double normalizeHeading(double robotError) {
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }


}
