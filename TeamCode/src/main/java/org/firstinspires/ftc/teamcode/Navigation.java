package org.firstinspires.ftc.teamcode;

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

    public Navigation() {

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
        pidControlHeading.setKp(0.0001f);
        pidControlHeading.setKi(0.0001f);
        pidControlHeading.setKd(0.0001f);
        pidControlHeading.setMaxIntegralError(2.0f/pidControlHeading.fKp);

        // update time stamp
        lastDistanceTimestamp= System.currentTimeMillis();
        lastSpeedTimestamp=lastDistanceTimestamp;
        lastHeadingTimestamp=lastDistanceTimestamp;
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

}
