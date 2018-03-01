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

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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
@Autonomous(name = "Harvester_PlanA_Red", group = "A_Harvester")

public class AutoHarvesterPlanARed extends AutoRelic {


    protected BNO055IMU imuSensor = null;

    protected HardwareHarvester robot= null;

    protected int leftBackStamp;
    protected int leftFrontStamp;
    protected int rightBackStamp;
    protected int rightFrontStamp;

    protected OpenGLMatrix vuforiaMatrix;
    protected VectorF trans;
    protected Orientation rot;

    public AutoHarvesterPlanARed() {
        teamColor = "red";

        glyphLiftPosition = 1500;
        centerGlyphAngleOffset = 0;
        vuforiaDetectingPower = -0.2;
        move2GlyphBoxPower = -0.6;
        move2CenterPower = 0.8;
        fGlyphTurnAngle = -90;
        center2GlyphBoxPower = -0.8;
        glyTurnPower = -0.4;

        cryptoBoxDistance = 680;
        center2GlyphDistance = 3500;

//        leftColumnDistance = 3860;
//        centerColumnDistance = 3150;
//        rightColumnDistance = 2500;
        leftColumnDistance = 3600;
        centerColumnDistance = 2950;
        rightColumnDistance = 2300;

    }

    @Override
    public void init() {
        robot = new HardwareHarvester();
        robot.init(hardwareMap);

        leftMotors = new DcMotor[2];
        leftMotors[0] = robot.motorLeftFrontWheel;
        leftMotors[1] = robot.motorLeftBackWheel;
        rightMotors = new DcMotor[2];
        rightMotors[0] = robot.motorRightFrontWheel;
        rightMotors[1] = robot.motorRightBackWheel;
        robot.defaultGlyphWheelPower = 0.15;

        jewelArm = robot.jewelArm;
        jewelHitter = robot.jewelHitter;

        jewelSensor = robot.jewelSensor;
        jewelSensorDistance = robot.jewelSensorDistance;
        imuSensor = robot.imu;

        jewelKicker = new JewelKicker(jewelSensor,jewelArm,jewelHitter,telemetry);
        //jewelKicker.init();
        jewelKicker.jewelArmActionPosition = 0.27;
        jewelKicker.jewelArmRestPosition = 0.55;
        jewelKicker.jewelHitterRestPosition= 0.5;
        jewelArmPos = jewelKicker.jewelArmActionPosition;
        jewelHitterPos = jewelKicker.jewelHitterRestPosition;

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
        vuforia.vumarkImage = "unknown";
        jewelKicker.start();
        robot.initAllDevices();

    }

    @Override
    public void loop() {
        switch (state) {
            case 0:
                // jewel handling
                state = jewelKicker.loop(0, 1, teamColor);

                // move jewel arm to avoid jewel holes
                jewelKicker.jewelArmActionPosition = jewelArmPos + 0.08*rand.nextDouble()-0.04;
                jewelKicker.jewelHitterRestPosition = jewelHitterPos + 0.02*rand.nextDouble()-0.01;

                robot.levelGlyph();

                //read vumark
                computeGlyphColumnDistance();

                getWheelLandmarks();

                break;
            case 1:

                // lift glyph bar
                VortexUtils.moveMotorByEncoder(robot.liftMotor, 10, liftMotorHolderPower);

                //set jewel hitter position
                robot.retractJewelArm();

                //move forward with encoder
                //read vumark
                double movePower = vuforiaDetectingPower;
                if ("unknown" == vuforia.vumarkImage.toLowerCase() ) {
                    computeGlyphColumnDistance();
                } else {
                    movePower = vuforiaDetectingPower*3.0;
                }

                if (0 == moveByDistance(movePower, columnDistance)) {
                    vuforia.relicTrackables.deactivate();
                    moveAtPower(0.0);
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                    state = 2;
                }

                break;
            case 2:
                // Turn to face center
                if (fGlyphTurnAngle == 0.0f || 0 == navigation.turnByEncoderOpenLoop(glyTurnPower,fGlyphTurnAngle,
                        robot.axleDistance, leftMotors, rightMotors)) {
                    turnAtPower(0.0);
                    telemetry.addData("left", robot.motorLeftFrontWheel.getCurrentPosition() - leftFrontStamp + robot.motorLeftBackWheel.getCurrentPosition() - leftBackStamp);
                    telemetry.addData("right", robot.motorRightBackWheel.getCurrentPosition() - rightBackStamp + robot.motorRightFrontWheel.getCurrentPosition() - rightFrontStamp);
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                    state = 3;
                }

                break;
            case 3:
                // move straight to crypto box
                if (0 == moveByDistance(move2GlyphBoxPower, cryptoBoxDistance)) {
                    moveAtPower(0.0);
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    state = 4;
                }

                break;
            case 4:
                // release the glyph
                time = System.currentTimeMillis();

                if (time - timeStamp < 800) {
                    robot.dumpGlyph();
                } else {

                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    state = 5;

                }

                break;

            case 5:
                // back up from the crypto box
                if (0 == moveByDistance(-move2GlyphBoxPower, 500)) {

                    moveAtPower(0.0);
                    robot.loadGlyph();
                    timeStamp = System.currentTimeMillis();

                    // lower glyph bars
                    VortexUtils.moveMotorByEncoder(robot.liftMotor, 0, liftMotorHolderPower);

                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    state = 6;

                }
                break;
            case 6:
                // push glyph into place
                if (0 == moveByDistance(move2GlyphBoxPower*3.0, 500)) {
                    moveAtSpeed(0.0);
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                    state = 7;
                }
                break;
            case 7:
                // back up from the crypto box
                if (0 == moveByDistance(0.8, 200)) {

                    moveAtPower(0.0);
                    robot.loadGlyph();
                    timeStamp = System.currentTimeMillis();

                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    state = 8;

                }
                break;
            case 8:
                // correct angle just in case it got knocked out the cource
                if (0 == navigation.turnByGyroCloseLoop(0.0, (double) robot.imu.getAngularOrientation().firstAngle,fGlyphTurnAngle+glyphOffAngle,leftMotors,rightMotors)) {
                    state = 9;
                    getWheelLandmarks();
                    robot.glyphWheelLoad();
                    navigation.resetTurn(leftMotors, rightMotors);
                }
                break;
            case 9:
                // move to center fast
                if (0 == moveByDistance(move2CenterPower, (int)(glyph2CenterDistance*0.75) + backupDistance + 500)) {
                    moveAtPower(0.0);
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    timeStamp = System.currentTimeMillis();
                    robot.loadGlyph();
                    robot.glyphWheelLoad();
                    state = 10;
                }

                break;
            case 10:
                // move to center slower to collect glyph
                if (0 == moveByDistance(collectingGlyphPower, (int)(glyph2CenterDistance*0.25))) {
                    moveAtPower(0.0);
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    timeStamp = System.currentTimeMillis();
                    state = 11;
                }

                break;
            case 11:
                // wait 0.5 second for robot to collect
                if (System.currentTimeMillis() - timeStamp > 500) {
                    state = 12;
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                    timeStamp = System.currentTimeMillis();
                }
                break;
            case 12:
                // correct angle just increase it got knocked out of course
                if (0 == navigation.turnByGyroCloseLoop(0.0, (double) robot.imu.getAngularOrientation().firstAngle,fGlyphTurnAngle+glyphOffAngle,leftMotors,rightMotors)) {
                    state = 13;
                    getWheelLandmarks();
                    robot.levelGlyph();
                    VortexUtils.moveMotorByEncoder(robot.liftMotor, glyphLiftPosition, liftMotorMovePower);

                    // spit out jammed glyphs
                    //robot.defaultGlyphWheelPower = 0.3;
                    //robot.glyphWheelUnload();

                    navigation.resetTurn(leftMotors, rightMotors);
                    timeStamp = System.currentTimeMillis();
                }
                break;
            case 13:

                // unload
                if (System.currentTimeMillis() - timeStamp > 1000) {
                    robot.glyphWheelUnload();
                }

                // back up
                if (0 == moveByDistance(center2GlyphBoxPower, center2GlyphDistance)) {
                    moveAtPower(0.0);
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    timeStamp = System.currentTimeMillis();

                    state = 14;
                }

                break;
            case 14:
                // correct angle just in case it got knocked out the course
                if (0 == navigation.turnByGyroCloseLoop(0.0, (double) robot.imu.getAngularOrientation().firstAngle,fGlyphTurnAngle,leftMotors,rightMotors)) {
                    state = 15;
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                }
                break;
            case 15:
                // move to center slower to collect glyph
                if (0 == moveByDistance(-collectingGlyphPower, 300)) {
                    moveAtPower(0.0);
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    timeStamp = System.currentTimeMillis();
                    state = 16;
                }

                break;
            case 16:
                // release glyph
                robot.dumpGlyph();

                if (System.currentTimeMillis() - timeStamp > 1000) {
                    getWheelLandmarks();
                    state = 17;
                }
                break;
            case 17:
                // backup
                if (0 == moveByDistance(-move2GlyphBoxPower , 400)) {
                    moveAtPower(0.0);
                    state = 18;
                }
                break;
            default:
                robot.loadGlyph();
                VortexUtils.moveMotorByEncoder(robot.liftMotor, 0, liftMotorMovePower);
                // stop
                vuforia.relicTrackables.deactivate();
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
        }
        if (Math.abs(robot.motorRightBackWheel.getCurrentPosition() - rightBackStamp) + Math.abs(robot.motorLeftFrontWheel.getCurrentPosition() - leftFrontStamp) < distance
                && Math.abs(robot.motorLeftBackWheel.getCurrentPosition() - leftBackStamp) + Math.abs(robot.motorRightFrontWheel.getCurrentPosition() - rightFrontStamp) < distance) {
            sideMoveAtPower(power);
        } else {
            sideMoveAtPower(0.0);
            return 0;
        }
        return 1;
    }

    public void stopGlyphWheels(){
        robot.leftLiftWheel.setPower(0.0);
        robot.rightLiftWheel.setPower(0.0);
    }

}
