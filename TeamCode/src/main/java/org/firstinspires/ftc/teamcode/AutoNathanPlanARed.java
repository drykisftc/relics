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

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.Random;

/*
 * This is an example LinearOpMode that shows how to use
 * the REV Robotics Color-Distance Sensor.
 *
 * It assumes the sensor is configured with the name "sensor_color_distance".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 */
@Autonomous(name = "Nathen_PlanA_Red", group = "Nathen")
public class AutoNathanPlanARed extends AutoRelic {

    protected HardwareNathen robot= null;

    Random rand = new Random(System.currentTimeMillis());
    int glyphLiftPosition = 0;
    double jewelArmPos = 0;
    double jewelHitterPos = 0;

    public AutoNathanPlanARed () {
        // team specific
        teamColor = "red";
        fGlyphTurnAngle = -90;
        cryptoBoxDistance = 800;
        glyphLiftPosition= 1500;

        //        cryptoBoxStopDistance = 20;
        //        vuforiaDetectingPower = 0.2;
        rightColumnDistance = 2130;
        centerColumnDistance = 2280;
        leftColumnDistance = 3630;
        //        cryptoBoxDistance = 500;
        //        backupDistance = -100;
        //        axleDistance = 18.1f;

        glyTurnPower = 0.25;
    }

    @Override
    public void init() {

        //
        robot = new HardwareNathen();
        robot.init(hardwareMap);
        robot.start();

        jewelArm = robot.jewelArm;
        jewelHitter = robot.jewelHitter;

        jewelSensor = robot.jewelSensor;
        jewelKicker = new JewelKicker(jewelSensor,jewelArm,jewelHitter,telemetry);
        jewelKicker.init();

        navigation = new Navigation(telemetry);

        vuforia = new HardwareVuforia(VuforiaLocalizer.CameraDirection.FRONT);
        vuforia.init(hardwareMap);

        leftMotors = new DcMotor[1];
        leftMotors[0] = robot.motorLeftWheel;
        rightMotors = new DcMotor[1];
        rightMotors[0] = robot.motorRightWheel;

        jewelKicker.jewelArmActionPosition= 0.0;
        jewelKicker.jewelArmRestPosition= 1.0;

        jewelKicker.jewelHitterRestPosition = 0.5;
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
//        if (robot.gyro.isCalibrating())  {
//            telemetry.addData(">", "Gyro is calibrating.  DO NOT start!!!!");
//            telemetry.addData(">", "Wait! Wait! Wait! ");
//        }
//        else {
//            telemetry.addData(">", "Press Start.");
//        }
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

                // gitter arm to avoid jewel holes
                jewelKicker.jewelArmActionPosition = jewelArmPos + 0.15*rand.nextDouble()-0.075;
                jewelKicker.jewelHitterRestPosition = jewelHitterPos + 0.1*rand.nextDouble()-0.05;

                vuforia.identifyGlyphCrypto();
                wheelDistanceLandMark = (robot.motorLeftWheel.getCurrentPosition() +
                        robot.motorRightWheel.getCurrentPosition())/2;

                break;
            case 1:

                VortexUtils.moveMotorByEncoder(robot.liftMotor, glyphLiftPosition, robot.liftMotorHolderPower);

                //read vumark
                computeGlyphColumnDistance();

                //move forward with encoder
                wheelDistanceAverage = (robot.motorLeftWheel.getCurrentPosition() +
                                            robot.motorRightWheel.getCurrentPosition())/2;

                if (wheelDistanceAverage < columnDistance) {

                    moveAtSpeed(vuforiaDetectingPower);

                } else {
                    moveAtSpeed(0.0);
                    vuforia.relicTrackables.deactivate();
                    navigation.resetTurn(leftMotors, rightMotors);
                    wheelDistanceLandMark = (robot.motorLeftWheel.getCurrentPosition() +
                            robot.motorRightWheel.getCurrentPosition())/2;
                    state = 4;
                }

                break;
            case 4:
                // turn
                if (0 == navigation.turnByEncoderOpenLoop(glyTurnPower,fGlyphTurnAngle,
                        robot.axleDistance, leftMotors, rightMotors)) {
                    state = 6;
                    wheelDistanceLandMark = (robot.motorLeftWheel.getCurrentPosition() +
                            robot.motorRightWheel.getCurrentPosition())/2;
                    navigation.resetTurn(leftMotors, rightMotors);
                }

                break;
//            case 5:
//                // turn
//                if (0 == navigation.turnByGyroCloseLoop(0.0,robot.getGyroHeading(),
//                        fGlyphTurnAngle, leftMotors, rightMotors)) {
//                    state = 6;
//                    wheelDistanceLandMark = (robot.motorLeftWheel.getCurrentPosition() +
//                            robot.motorRightWheel.getCurrentPosition())/2;
//                }
//                break;
            case 6:
                // move straight
                wheelDistanceAverage = (robot.motorLeftWheel.getCurrentPosition() +
                        robot.motorRightWheel.getCurrentPosition())/2;

                if (wheelDistanceAverage - wheelDistanceLandMark < cryptoBoxDistance) {
                    moveAtSpeed(0.25);

                } else {
                    moveAtSpeed(0.0);
                    timeStamp = System.currentTimeMillis();
                    state = 7;
                }

                break;
            case 7:
                // release the glyph

                time = System.currentTimeMillis();
                if (time - timeStamp < 2000) {
                    robot.leftHand.setPosition(robot.leftHandOpenPosition);
                    robot.rightHand.setPosition(robot.rightHandOpenPosition);
                } else {
                    state = 8;
                    wheelDistanceLandMark = (robot.motorLeftWheel.getCurrentPosition() +
                            robot.motorRightWheel.getCurrentPosition())/2;
                    moveAtSpeed(-0.2);
                    timeStamp = System.currentTimeMillis();
                }

                break;
            case 8:
                // backup
                wheelDistanceAverage = (robot.motorLeftWheel.getCurrentPosition() +
                        robot.motorRightWheel.getCurrentPosition())/2;

                if (wheelDistanceAverage - wheelDistanceLandMark < backupDistance) {
                    moveAtSpeed(0.0);
                    state = 9;
                }

                break;
            default:
                robot.stop();
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

    public void moveAtSpeed (double speed) {
        robot.motorLeftWheel.setPower(speed);
        robot.motorRightWheel.setPower(speed);
    }

    public void turnAtSpeed (double speed) {
        robot.motorLeftWheel.setPower(speed);
        robot.motorRightWheel.setPower(-speed);
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

}
