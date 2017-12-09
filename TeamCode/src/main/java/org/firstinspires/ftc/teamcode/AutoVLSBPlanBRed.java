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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

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
@Autonomous(name = "VLSB_PlanB_Red", group = "a_VLSB")

public class AutoVLSBPlanBRed extends AutoVLSBPlanARed {


    protected BNO055IMU imuSensor = null;

    protected HardwareVLSB robot= null;

    protected int leftBackStamp;
    protected int leftFrontStamp;
    protected int rightBackStamp;
    protected int rightFrontStamp;


    public AutoVLSBPlanBRed() {
        teamColor = "red";

        glyphLiftPosition = 2200;
        centerGlyphAngleOffset = 0;
        cryptoBoxDistance = -500;

        vuforiaDetectingPower = -0.2;

        leftColumnDistance = 3800;
        centerColumnDistance = 2350;
        rightColumnDistance = 900;
    }

    @Override
    public void loop() {
        switch (state) {
            case 0:

                // jewel handling
                state = jewelKicker.loop(0, 1, teamColor);

                vuforia.identifyGlyphCrypto();

                getWheelLandmarks();

                break;
            case 1:

                //read vumark
                computeGlyphColumnDistance();

                //move backward with encoder
                if (0 == moveByDistance(vuforiaDetectingPower, offBalanceStoneDistance)) {
                    moveAtPower(0.0);
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                    state = 2;
                }

                break;
            case 2:
                //side movement
                if ( 0 == sideMoveByDistance(sideMovePower, columnDistance) ){
                    wheelDistanceLandMark = getWheelOdometer();
                    state = 3;
                }

                break;
            case 3:
                // move straight
                if ( 0 == moveByDistance(-0.5, cryptoBoxDistance)) {
                    moveAtPower(0.0);
                    timeStamp = System.currentTimeMillis();
                    state = 4;
                }

                break;
            case 4:
                // release the glyph

                time = System.currentTimeMillis();

                if (time - timeStamp < 700) {
                    releaseGlyph();
                } else {

                    timeStamp = System.currentTimeMillis();
                    state = 5;

                }

                break;
            case 5:

                time = System.currentTimeMillis();

                if (time - timeStamp < 1300) {
                    moveAtPower(backupPower);
                } else {
                    moveAtPower(0.0);
                    state = 6;
                }

                break;
            case 6:

                if (0 == moveByDistance(-0.50, backupDistance)) {

                    moveAtPower(0.0);
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    // navigation.resetTurn(leftMotors, rightMotors);
                    // lower glyph bars
                    VortexUtils.moveMotorByEncoder(robot.liftMotor, 0, liftMotorHolderPower);

                    state = 7;

                }

                break;
            case 7:

                if (System.currentTimeMillis() - timeStamp > 300) {
                    getWheelLandmarks();
                    // navigation.resetTurn(leftMotors, rightMotors);
                    state = 10;
                }

                break;
            case 8:
                // turn 180
                if (0 == navigation.turnByEncoderOpenLoop(glyTurnPower, fCenterTurnAngle + centerGlyphAngleOffset, robot.axleDistance, leftMotors, rightMotors)) {
                    state = 9;
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                }
                break;
            case 9:
                // turn 180
                if (0 == navigation.turnByGyroCloseLoop(0.0, (double) robot.imu.getAngularOrientation().firstAngle,fGlyphTurnAngle+fCenterTurnAngle+centerGlyphAngleOffset,leftMotors,rightMotors)) {
                    state = 10;
                    getWheelLandmarks();
                }

                // set glyph bars in collect positions
                collectGlyph();

                break;
            case 10:
                // move to center
                if (0 == moveByDistance(0.8, glyph2CenterDistance)) {
                    moveAtPower(0.0);
                    //navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    timeStamp = System.currentTimeMillis();
                    state = 11;
                }

                break;
            case 11:
                // collect glyph
                collectGlyph();

                if ( System.currentTimeMillis() - timeStamp < 3000) {
                    state = 12;
                    getWheelLandmarks();
                }
                break;
            case 12:
                // back up
                if (0 == moveByDistance(-0.8, glyph2CenterDistance)) {
                    moveAtPower(0.0);
                    //navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    state = 13;
                }
                // lift glyph bar
                VortexUtils.moveMotorByEncoder(robot.liftMotor, glyphLiftPosition, liftMotorHolderPower);

                break;
            case 13:
                // wait 1 second
                if (System.currentTimeMillis() - timeStamp > 300) {
                    getWheelLandmarks();
                    //navigation.resetTurn(leftMotors, rightMotors);
                    state = 15;
                }
                break;
            case 14:
                // turn
                if (0 == navigation.turnByEncoderOpenLoop(glyTurnPower, fCenterTurnAngle, robot.axleDistance, leftMotors, rightMotors)) {
                    state = 15;
                    getWheelLandmarks();
                    moveAtPower(0.2);
                    navigation.resetTurn(leftMotors, rightMotors);
                }
                break;
            case 15:
                // move back to glyph grid
                if (0 == waitByDistance(0.3, -backupDistance+400)) {
                    moveAtPower(0.0);
                    getWheelLandmarks();
                    timeStamp = System.currentTimeMillis();
                    state = 16;
                }
                break;
            case 16:
                // release glyph
                releaseGlyph();
                if (System.currentTimeMillis() - timeStamp > 500) {
                    getWheelLandmarks();
                    state = 17;
                }
                break;
            case 17:
                // backup
                if (0 == moveByDistance(-0.15, 300)) {
                    moveAtPower(0.0);
                    state = 18;
                }
                break;
            default:
                // stop
                robot.stop();
                break;
        }

        telemetry.addData("state", state);
        telemetry.addData("vumark", vuforia.vumarkImage);
        telemetry.addData("teamColor", teamColor);
        telemetry.update();
    }

    public void getWheelLandmarks () {
        leftBackStamp = robot.motorLeftBackWheel.getCurrentPosition();
        leftFrontStamp = robot.motorLeftFrontWheel.getCurrentPosition();
        rightBackStamp = robot.motorRightBackWheel.getCurrentPosition();
        rightFrontStamp = robot.motorRightFrontWheel.getCurrentPosition();
        wheelDistanceLandMark = (leftBackStamp+leftFrontStamp+rightBackStamp+rightFrontStamp)/4;
    }

    public void sideMoveAtPower(double p) {
        robot.motorLeftFrontWheel.setPower(-p);
        robot.motorRightBackWheel.setPower(-p);
        robot.motorRightFrontWheel.setPower(p);
        robot.motorLeftBackWheel.setPower(p);
    }

    public void moveAtSpeed(double p){
        robot.motorLeftFrontWheel.setPower(p);
        robot.motorRightBackWheel.setPower(p);
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

    public void releaseGlyph () {
        robot.smolL.setPosition(0.00);
        robot.lowerBeltServo1.setPower(-1.0);
        robot.lowerBeltServo2.setPower(-1.0);
    }

    public void collectGlyph () {
        robot.smolL.setPosition(0.50);
        robot.leftLiftWheel.setPower(-1.0);
        robot.rightLiftWheel.setPower(1.0);
        robot.lowerBeltServo1.setPower(-1.0);
        robot.lowerBeltServo2.setPower(-1.0);
    }

    public void stopGlyphWheels(){
        robot.leftLiftWheel.setPower(0.0);
        robot.rightLiftWheel.setPower(0.0);
    }

}