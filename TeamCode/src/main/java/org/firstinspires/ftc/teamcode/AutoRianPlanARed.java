/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/*
 * This is an example LinearOpMode that shows how to use
 * the REV Robotics Color-Distance Sensor.
 *
 * It assumes the sensor is configured with the name "sensor_color_distance".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 */
@Autonomous(name = "Rian_PlanA_Red", group = "Rian")

public class AutoRianPlanARed extends AutoRelic {


    protected BNO055IMU imuSensor = null;

    protected HardwareRian robot= null;

    protected int leftBackStamp;
    protected int leftFrontStamp;
    protected int rightBackStamp;
    protected int rightFrontStamp;

    public AutoRianPlanARed () {
        leftMotors = new DcMotor[2];
        leftMotors[0] = robot.motorLeftFrontWheel;
        leftMotors[0] = robot.motorLeftBackWheel;
        rightMotors = new DcMotor[2];
        rightMotors[0] = robot.motorRightFrontWheel;
        rightMotors[0] = robot.motorRightBackWheel;
    }

    @Override
    public void init() {
        teamColor = "red";
        robot = new HardwareRian();
        robot.init(hardwareMap);

        jewelArm = robot.jewelArm;
        jewelHitter = robot.jewelHitter;

        jewelSensor = robot.jewelSensor;
        jewelSensorDistance = robot.jewelSensorDistance;
        imuSensor = robot.imu;

        jewelKicker = new JewelKicker(jewelSensor,jewelArm,jewelHitter,telemetry);
        jewelKicker.init();

        navigation = new Navigation(telemetry);

        vuforia = new HardwareVuforia(VuforiaLocalizer.CameraDirection.BACK);
        vuforia.init(hardwareMap);

        telemetry.addData("jewelArm", jewelArm.getPosition());
        telemetry.addData("jewelHitter", jewelHitter.getPosition());
        telemetry.update();
    }


    @Override
    public void start() {
        robot.start();
        vuforia.start();
        state = 0;
        timeStamp = System.currentTimeMillis();
        vuforia.vumarkImage = "Unknown";
        jewelKicker.start();
    }

    @Override
    public void loop() {
        switch (state) {
            case 0:

                // jewel handling
                state = jewelKicker.loop(0, 1, teamColor);

                vuforia.identifyGlyphCrypto();

                break;
            case 1:

                //read vumark
                computeGlyphColumnDistance();

                //move forward with encoder
                wheelDistanceAverage = getWheelOdometer();

                if (wheelDistanceAverage < columnDistance) {

                    moveAtPower(vuforiaDetectingPower);

                } else {
                    vuforia.relicTrackables.deactivate();
                    moveAtPower(0.0);
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                    state = 2;

                }

                //move forward
                /*if (!(robot.jewelSensorDistance.getDistance(DistanceUnit.CM) < cryptoBoxStopDistance)) {

                    moveAtPower(vuforiaDetectingPower);

                } else {

                    moveAtPower(0);
                    state = 2;
                }*/

                break;
            case 2:
//                if (fGlyphTurnAngle == 0.0f || 0 == navigation.turnByEncoderOpenLoop(glyTurnPower,fGlyphTurnAngle,
//                        robot.axleDistance, leftMotors, rightMotors)) {
//                    turnAtPower(0.0);
//                    wheelDistanceLandMark = getWheelOdometer();
//                    telemetry.addData("left", robot.motorLeftFrontWheel.getCurrentPosition() - leftFrontStamp + robot.motorLeftBackWheel.getCurrentPosition() - leftBackStamp);
//                    telemetry.addData("left", robot.motorRightBackWheel.getCurrentPosition() - rightBackStamp + robot.motorRightFrontWheel.getCurrentPosition() - rightFrontStamp);
//                    getWheelLandmarks();
//                    navigation.resetTurn(leftMotors, rightMotors);
//                    state = 3;
//                }

                if ((robot.motorLeftFrontWheel.getCurrentPosition() - leftFrontStamp + robot.motorLeftBackWheel.getCurrentPosition() - leftBackStamp > 3575) && (robot.motorRightBackWheel.getCurrentPosition() - rightBackStamp + robot.motorRightFrontWheel.getCurrentPosition() - rightFrontStamp < -3575)) {

                    turnAtPower(0.0);
                    wheelDistanceLandMark = getWheelOdometer();
                    telemetry.addData("left", robot.motorLeftFrontWheel.getCurrentPosition() - leftFrontStamp + robot.motorLeftBackWheel.getCurrentPosition() - leftBackStamp);
                    telemetry.addData("left", robot.motorRightBackWheel.getCurrentPosition() - rightBackStamp + robot.motorRightFrontWheel.getCurrentPosition() - rightFrontStamp);
                    state = 3;

                } else {

                    turnAtPower(glyTurnPower);
                }

                break;
            case 3:
                // move straight

                wheelDistanceAverage = getWheelOdometer();

                if (wheelDistanceAverage - wheelDistanceLandMark < cryptoBoxDistance) {

                    moveAtPower(0.5);

                } else {

                    moveAtPower(0.0);
                    timeStamp = System.currentTimeMillis();
                    state = 4;

                }

                break;
            case 4:
                // release the glyph

                time = System.currentTimeMillis();

                if (time - timeStamp < 1000) {
                    releaseGlyph();
                } else {

                    timeStamp = System.currentTimeMillis();
                    state = 5;

                }

                break;
            case 5:

                time = System.currentTimeMillis();

                if (time - timeStamp < 1000) {
                    moveAtPower(backupPower);
                } else {
                    moveAtPower(0.0);
                    state = 6;
                }

                break;
            case 6:

                // stop
                robot.stop();
                break;
            default:
        }

        telemetry.addData("state", state);
        telemetry.addData("vumark", vuforia.vumarkImage);
        telemetry.update();
    }

    public float getTurnDistance (float angle, float axleDistance){
        // if the res is positive, set left Motor move forward (right turn),
        // other wise (left turn)
        return (float)(axleDistance * angle * 3.14) / 360;
    }

    public void moveAtPower(double p) {

        robot.motorLeftBackWheel.setPower(p);
        robot.motorLeftFrontWheel.setPower(p);
        robot.motorRightBackWheel.setPower(p);
        robot.motorRightFrontWheel.setPower(p);
    }

    public void sideMoveAtPower(double p) {
        robot.motorLeftFrontWheel.setPower(-p);
        robot.motorRightBackWheel.setPower(-p);
        robot.motorRightFrontWheel.setPower(p);
        robot.motorLeftBackWheel.setPower(p);
    }

    // positive power moves left
    public int sideMoveByDistance (double power, int d) {
        int distance = Math.abs(d);
        if (power == 0) {
            return 0; // zero power do nothing
        } else if (power < 0){
            distance = -distance;
        }

        if (robot.motorRightBackWheel.getCurrentPosition() - rightBackStamp + robot.motorLeftFrontWheel.getCurrentPosition() - leftFrontStamp > -distance
                && robot.motorLeftBackWheel.getCurrentPosition() - leftBackStamp + robot.motorRightFrontWheel.getCurrentPosition() - rightFrontStamp < distance) {
            sideMoveAtPower(sideMovePower);
        } else {
            sideMoveAtPower(0.0);
            return 0;
        }
        return 1;
    }

    // positive power to turn left
    public void turnAtPower(double p) {
        robot.motorLeftBackWheel.setPower(-p);
        robot.motorLeftFrontWheel.setPower(-p);
        robot.motorRightBackWheel.setPower(p);
        robot.motorRightFrontWheel.setPower(p);
    }

    public int getWheelOdometer() {
        return (robot.motorLeftBackWheel.getCurrentPosition() +
                robot.motorLeftFrontWheel.getCurrentPosition() +
                robot.motorRightBackWheel.getCurrentPosition() +
                robot.motorRightFrontWheel.getCurrentPosition())/4;
    }

    public void getWheelLandmarks () {
        leftBackStamp = robot.motorLeftBackWheel.getCurrentPosition();
        leftFrontStamp = robot.motorLeftFrontWheel.getCurrentPosition();
        rightBackStamp = robot.motorRightBackWheel.getCurrentPosition();
        rightFrontStamp = robot.motorRightFrontWheel.getCurrentPosition();
    }

    public void releaseGlyph () {
        robot.leftLiftWheel1.setPower(1.0);
        robot.leftLiftWheel2.setPower(1.0);
        robot.leftLiftWheel3.setPower(1.0);
        robot.rightLiftWheel1.setPower(-1.0);
        robot.rightLiftWheel2.setPower(-1.0);
        robot.rightLiftWheel3.setPower(-1.0);
    }

    public void stopGlyphWheels(){
        robot.leftLiftWheel1.setPower(0.0);
        robot.leftLiftWheel2.setPower(0.0);
        robot.leftLiftWheel3.setPower(0.0);
        robot.rightLiftWheel1.setPower(0.0);
        robot.rightLiftWheel2.setPower(0.0);
        robot.rightLiftWheel3.setPower(0.0);
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

}
