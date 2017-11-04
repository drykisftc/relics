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
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.Random;

@Autonomous(name = "Nathen_PlanA_Red", group = "Nathen")
public class AutoNathanPlanARed extends AutoRelic {

    protected HardwareNathen robot= null;

    Random rand = new Random(System.currentTimeMillis());
    int glyphLiftPosition = 0;
    int glyphLiftPosition2= 0;
    double jewelArmPos = 0;
    double jewelHitterPos = 0;
    float centerGlyphAngleOffset = 0;
    double encoderCountPerInch = 86.75;//80.79;

    public AutoNathanPlanARed () {
        // team specific
        teamColor = "red";
        fGlyphTurnAngle = -90;
        fCenterTurnAngle = -180;

        cryptoBoxDistance = 600;
        glyphLiftPosition= 2200;
        glyphLiftPosition2 = 2000;

        //the glyph box is 22.6 inches wide, 1826 steps
        rightColumnDistance = 2250;
        centerColumnDistance = (int)(rightColumnDistance + 7.63*encoderCountPerInch);
        leftColumnDistance = (int)(rightColumnDistance + 15.26*encoderCountPerInch);

        backupDistance = -1000;
        glyph2CenterDistance = 2080;
        center2GlyphDistance = 2400;

        glyTurnPower = 0.20;
        centerGlyphAngleOffset = 0;

    }

    @Override
    public void init() {

        //
        robot = new HardwareNathen();
        robot.init(hardwareMap);
        robot.start();

        robot.gyro.calibrate();

        jewelArm = robot.jewelArm;
        jewelHitter = robot.jewelHitter;

        jewelSensor = robot.jewelSensor;
        jewelKicker = new JewelKicker(jewelSensor,jewelArm,jewelHitter,telemetry);
        jewelKicker.init();

        navigation = new Navigation(telemetry);
        navigation.pidControlHeading.setKp(0.004);
        navigation.pidControlHeading.setKi(0.002);
        navigation.pidControlHeading.setKd(0.0000001);
        navigation.maxTurnDeltaPower = 0.4;
        navigation.convergeCountThreshold = 6;
        navigation.angleErrorTolerance = 2.1;

        vuforia = new HardwareVuforia(VuforiaLocalizer.CameraDirection.FRONT);
        vuforia.init(hardwareMap);

        leftMotors = new DcMotor[1];
        leftMotors[0] = robot.motorLeftWheel;
        rightMotors = new DcMotor[1];
        rightMotors[0] = robot.motorRightWheel;

        jewelKicker.jewelArmActionPosition= 0.0;
        jewelKicker.jewelArmRestPosition= 1.0;

        jewelKicker.jewelHitterRestPosition = 0.48;
        jewelKicker.jewelHitterRedPosition = 0.0;
        jewelKicker.jewelHitterBluePosition = 1.0;

        jewelArmPos = jewelKicker.jewelArmActionPosition;
        jewelHitterPos = jewelKicker.jewelHitterRestPosition;

        telemetry.addData("jewelArm", jewelArm.getPosition());
        telemetry.addData("jewelHitter", jewelHitter.getPosition());
        telemetry.update();
    }

    @Override
    public void init_loop () {
        if (robot.gyro.isCalibrating())  {
            telemetry.addData(">", "Gyro is calibrating.  DO NOT start!!!!");
            telemetry.addData(">", "Wait! Wait! Wait! ");
        }
        else {
            telemetry.addData(">", "Press Start.");
        }
    }

    @Override
    public void start() {
        robot.start();
        vuforia.start();
        state = 0;
        timeStamp = System.currentTimeMillis();
        vuforia.vumarkImage = "Unknown";
        jewelKicker.start();
        jewelKicker.jewelWaitTime = 2000;

        robot.gyro.resetZAxisIntegrator();

        // hold the glyph
        robot.leftHand.setPosition(robot.leftHandClosePosition);
        robot.rightHand.setPosition(robot.rightHandClosePosition);
    }

    @Override
    public void loop() {
        switch (state) {
            case 0:

                // jewel handling
                state = jewelKicker.loop(0, 1, teamColor);

                // hitter arm to avoid jewel holes
                jewelKicker.jewelArmActionPosition = jewelArmPos + 0.2*rand.nextDouble()-0.1;
                jewelKicker.jewelHitterRestPosition = jewelHitterPos + 0.06*rand.nextDouble()-0.03;

                computeGlyphColumnDistance();

                if ("left" == vuforia.vumarkImage) {
                    centerGlyphAngleOffset = 0;
                } else if ("right" == vuforia.vumarkImage) {
                    centerGlyphAngleOffset = 12;
                }

                getWheelLandmarks();

                break;
            case 1:

                VortexUtils.moveMotorByEncoder(robot.liftMotor, glyphLiftPosition, robot.liftMotorHolderPower);

                //read vumark
                computeGlyphColumnDistance();

                if ("left" == vuforia.vumarkImage) {
                    centerGlyphAngleOffset = 0;
                } else if ("right" == vuforia.vumarkImage) {
                    centerGlyphAngleOffset = 12;
                }

                //move forward with encoder
                if ( 0 == moveByDistance(vuforiaDetectingPower, columnDistance )) {
                    vuforia.relicTrackables.deactivate();
                    moveAtPower(0.0);
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                    timeStamp = System.currentTimeMillis();
                    state = 2;
                }

                break;
            case 2:
                // wait 1 second
                if (System.currentTimeMillis() - timeStamp > 300) {
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                    state = 3;
                }
                break;
            case 3:
                // turn by encoder
                if (0 == navigation.turnByEncoderOpenLoop(glyTurnPower,fGlyphTurnAngle, robot.axleDistance, leftMotors, rightMotors)) {
                    state = 5;
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                    moveAtPower(0.2);
                }
                break;
//            case 4:// turn by gyro
//                if (0 == navigation.turnByGyroCloseLoop(0.0,robot.gyro.getHeading(), fGlyphTurnAngle,leftMotors,rightMotors) ) {
//                    state = 5;
//                    getWheelLandmarks();
//                    moveAtPower(0.2);
//                }
            case 5:
                // move straight
                if (0 == waitByDistance(0.2, cryptoBoxDistance)) {
                    moveAtPower(0.0);
                    timeStamp = System.currentTimeMillis();
                    state = 6;
                }

                break;
            case 6:
                // release the glyph
                time = System.currentTimeMillis();
                if (time - timeStamp < 1000) {
                    robot.leftHand.setPosition(robot.leftHandOpenPosition);
                    robot.rightHand.setPosition(robot.rightHandOpenPosition);
                } else {
                    state = 7;
                    getWheelLandmarks();
                    moveAtPower(-0.2);
                    timeStamp = System.currentTimeMillis();
                }
                break;
            case 7:
                // backup
                if (0 == moveByDistance(-0.25, backupDistance)) {
                    moveAtPower(0.0);
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                    // lower glyph bars
                    VortexUtils.moveMotorByEncoder(robot.liftMotor, 0, robot.liftMotorHolderPower);

                    state = 8;
                }
                break;
            case 8:
                // wait 1 second
                if (System.currentTimeMillis() - timeStamp > 300) {
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                    state = 9;
                }
                break;
            case 9:
                // turn 180
                 if (0 == navigation.turnByEncoderOpenLoop(glyTurnPower, fCenterTurnAngle+centerGlyphAngleOffset, robot.axleDistance, leftMotors, rightMotors)) {
                    state = 10;
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                }
                break;
            case 10:
                // turn 180
                 if (0 == navigation.turnByGyroCloseLoop(0.0,robot.gyro.getHeading(),fGlyphTurnAngle+fCenterTurnAngle+centerGlyphAngleOffset,leftMotors,rightMotors)) {
                    state = 11;
                    getWheelLandmarks();
                }
                break;
            case 11:
                // move to center
                 if (0 == moveByDistance(0.3, glyph2CenterDistance)) {
                    moveAtPower(0.0);
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    timeStamp = System.currentTimeMillis();
                    state = 12;
                }

                // set glyph bars in collect positions
                robot.leftHand.setPosition(robot.leftHandChargePosition);
                robot.rightHand.setPosition(robot.rightHandChargePosition);

                break;
            case 12:
                // collect glyph
                 robot.leftHand.setPosition(robot.leftHandClosePosition);
                 robot.rightHand.setPosition(robot.rightHandClosePosition);
                 if ( System.currentTimeMillis() - timeStamp < 3000) {
                     state = 13;
                     getWheelLandmarks();
                 }
                break;
            case 13:
                // back up
                 if (0 == moveByDistance(-0.25, glyph2CenterDistance)) {
                    moveAtPower(0.0);
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    state = 14;
                }
                // lift glyph bar
                VortexUtils.moveMotorByEncoder(robot.liftMotor, glyphLiftPosition, robot.liftMotorHolderPower);

                break;
            case 14:
                // wait 1 second
                if (System.currentTimeMillis() - timeStamp > 300) {
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                    state = 15;
                }
                break;
            case 15:
                // turn
                if (0 == navigation.turnByEncoderOpenLoop(glyTurnPower, fCenterTurnAngle, robot.axleDistance, leftMotors, rightMotors)) {
                    state = 17;
                    getWheelLandmarks();
                    moveAtPower(0.2);
                    navigation.resetTurn(leftMotors, rightMotors);
                }
                break;
//            case 16:
//                // turn
//                if (0 == navigation.turnByGyroCloseLoop(0.0,robot.gyro.getHeading(), fGlyphTurnAngle+2,leftMotors,rightMotors)) {
//                    state = 17;
//                    getWheelLandmarks();
//                    moveAtPower(0.2);
//                }
//                break;
            case 17:
                // move back to glyph grid
                if (0 == waitByDistance(0.2, -backupDistance+400)) {
                    moveAtPower(0.0);
                    getWheelLandmarks();
                    timeStamp = System.currentTimeMillis();
                    state = 18;
                }
                break;
            case 18:
                // release glyph
                robot.leftHand.setPosition(robot.leftHandOpenPosition);
                robot.rightHand.setPosition(robot.rightHandOpenPosition);
                if (System.currentTimeMillis() - timeStamp > 500) {
                    getWheelLandmarks();
                    state = 19;
                }
                break;
            case 19:
                // backup
                 if (0 == moveByDistance(-0.15, 300)) {
                     moveAtPower(0.0);
                 }
                break;
            default:
                robot.stop();
        }

        telemetry.addData("state", state);
        telemetry.addData("vumark", vuforia.vumarkImage);
        telemetry.addData("Column distance ", columnDistance);
        telemetry.addData("Gyro ", robot.gyro.getHeading());
        telemetry.update();
    }

}
