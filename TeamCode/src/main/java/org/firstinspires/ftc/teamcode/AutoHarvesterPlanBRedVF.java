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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/*
 * This is an example LinearOpMode that shows how to use
 * the REV Robotics Color-Distance Sensor.
 *
 * It assumes the sensor is configured with the name "sensor_color_distance".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 */
@Autonomous(name = "Harvester_PlanB_Red_VF", group = "A_Harvester")

public class AutoHarvesterPlanBRedVF extends AutoHarvesterPlanBRed {

    final int leftVuDis = -968;  //42.5 inches
    final int centerVuDis = -760; // 35 inches
    final int rightVuDis = -574; // 27.5 inches

    int center2GlyphDistance = 3500;
    int vuforiaMissCount = 0;
    int vuforiaHitCount = 0;
    int vuforiaCheckDistance = 0;
    int vuforiaTargetDistance = leftVuDis;

    @Override
    public void loop() {
        switch (state) {
            case 0:
                robot.defaultGlyphWheelPower = 0.5;
                vuforiaMissCount = 0;
                vuforiaHitCount = 0;
                vuforiaCheckDistance = leftColumnDistance;
                vuforiaTargetDistance = leftVuDis;

                // jewel handling
                state = jewelKicker.loop(0, 1, teamColor);

                // hitter arm to avoid jewel holes
                jewelKicker.jewelArmActionPosition = jewelArmPos + 0.08 * rand.nextDouble() - 0.04;
                jewelKicker.jewelHitterRestPosition = jewelHitterPos + 0.02 * rand.nextDouble() - 0.01;

                robot.levelGlyph();

                computeGlyphColumnDistance();

                break;
            case 1:

                //read vumark
                double movePower = vuforiaDetectingPower;
                if ("unknown" == vuforia.vumarkImage.toLowerCase()) {
                    computeGlyphColumnDistance();
                } else {
                    movePower = vuforiaDetectingPower * 3.0;
                }

                // lift glyph bar
                VortexUtils.moveMotorByEncoder(robot.liftMotor, 10, liftMotorHolderPower);

                //set jewel hitter position
                robot.jewelHitter.setPosition(0.00);
                robot.jewelArm.setPosition(0.90);

                if (0 == moveByDistance(movePower, offBalanceStoneDistance)) {
                    moveAtPower(0.0);
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                    state = 2;
                }

                break;
            case 2:
                // move left
                if (0 == sideMoveByDistance(sideMovePower*0.5, columnDistance)) {
                    wheelDistanceLandMark = getWheelOdometer();
                    state = 3;
                }

                break;
            case 3:
                // turn if necessary
                if (fGlyphTurnAngle == 0.0f || 0 == navigation.turnByEncoderOpenLoop(glyTurnPower, fGlyphTurnAngle,
                        robot.axleDistance, leftMotors, rightMotors)) {
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                    state = 4;
                }
                break;
            case 4:

                if (0 == moveByDistance(glyphDeliverPower, cryptoBoxDistance)) {
                    timeStamp = System.currentTimeMillis();
                    state = 5;
                }

                break;
            case 5:
                // release the glyph
                time = System.currentTimeMillis();

                if (time - timeStamp < 800) {

                    robot.dumpGlyph();

                } else {

                    timeStamp = System.currentTimeMillis();
                    state = 6;
                }
                break;
            case 6:
                //back up
                if (0 == moveByDistance(-glyphDeliverPower*2, backupDistance)) {
                    timeStamp = System.currentTimeMillis();
                    robot.levelGlyph();
                    getWheelLandmarks();
                    state = 7;
                }

                break;
            case 7:
                // push
                if (0 == moveByDistance(glyphDeliverPower * 2, pushDistance + 150)) {
                    moveAtPower(0.0);
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);

                    state = 8;
                }

                break;
            case 8:
                // backup
                if (0 == moveByDistance(-glyphDeliverPower*3, 500)) {

                    moveAtPower(0.0);
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    wheelDistanceLandMark = getWheelOdometer();
                    navigation.resetTurn(leftMotors, rightMotors);
                    // lower glyph bars
                    VortexUtils.moveMotorByEncoder(robot.liftMotor, 0, liftMotorHolderPower);
                    robot.loadGlyph();
                    state = 9;
                }

                break;
            case 9:
                // correct angle just increase it got knocked out the cource
                if (0 == navigation.turnByGyroCloseLoop(0.0,
                        (double) robot.imu.getAngularOrientation().firstAngle,
                        fGlyphTurnAngle, leftMotors, rightMotors)) {
                    state = 10;
                    getWheelLandmarks();
                    wheelDistanceLandMark = getWheelOdometer();
                    navigation.resetTurn(leftMotors, rightMotors);
                    robot.retractJewelArm();
                    timeStamp = System.currentTimeMillis();
                }
                break;
            case 10:
                // move side way
                if (0 == sideMoveByDistance(sideMovePower, sideWayDistance - columnDistance)) {
                    wheelDistanceLandMark = getWheelOdometer();
                    getWheelLandmarks();
                    robot.glyphWheelLoad();
                    robot.extendGlyphBlocker();
                    state = 11;
                }

                break;
            case 11:
                // move to center
                if (0 == moveByDistance(move2CenterPower, (int)(glyph2CenterDistance*0.8))) {
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    timeStamp = System.currentTimeMillis();
                    state = 12;
                }
                break;
            case 12:
                // back up
                if (0 == moveByDistance(-move2CenterPower, 300)) {
                    moveAtPower(0.0);
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    timeStamp = System.currentTimeMillis();
                    state = 13;
                }
                break;
            case 13:
                //wiggle
                navigation.turnByGyroCloseLoop(0.0,
                        (double) robot.imu.getAngularOrientation().firstAngle,
                        fGlyphTurnAngle+rand.nextInt(20)-10,
                        leftMotors, rightMotors);
                // move to center slower to collect glyph
                if (0 == moveByDistance(collectingGlyphPower, (int) (glyph2CenterDistance * 0.2)+400)) {
                    moveAtPower(0.0);
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    timeStamp = System.currentTimeMillis();
                    state = 14;
                }
                break;
            case 14:

                // wiggle to improve intake
                navigation.turnByGyroCloseLoop(0.0,
                        (double) robot.imu.getAngularOrientation().firstAngle,
                        fGlyphTurnAngle+rand.nextInt(30)-15,
                        leftMotors, rightMotors);

                    // collect glyph
                if (System.currentTimeMillis() - timeStamp > 1000) {
                    state = 15;
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                }
                break;
            case 15:
                // correct angle just increase it got knocked out the cource
                if (0 == navigation.turnByGyroCloseLoop(0.0, (double) robot.imu.getAngularOrientation().firstAngle, fGlyphTurnAngle, leftMotors, rightMotors)) {
                    state = 16;
                    getWheelLandmarks();
                    robot.levelGlyph();
                    robot.retractGlyphBlocker();
                    // lift
                    VortexUtils.moveMotorByEncoder(robot.liftMotor, glyphLiftPosition, liftMotorMovePower);
                    navigation.resetTurn(leftMotors, rightMotors);
                    robot.retractJewelArm();
                    timeStamp = System.currentTimeMillis();
                }
                break;
            case 16:
                // unload
                if (System.currentTimeMillis() - timeStamp > 900) {
                    robot.glyphWheelUnload();
                }

                // move away from center
                if (0 == moveByDistance(-move2CenterPower, center2GlyphDistance)) {
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    wheelDistanceLandMark = getWheelOdometer();
                    navigation.resetTurn(leftMotors, rightMotors);
                    state = 17;
                }
                break;
            case 17:
                // move side way to left column
                if (0 == sideMoveByDistance(-sideMovePower, sideWayDistance - vuforiaCheckDistance-1000)) {
                    wheelDistanceLandMark = getWheelOdometer();
                    getWheelLandmarks();
                    timeStamp = System.currentTimeMillis();
                    vuforiaMissCount = 0;
                    vuforiaHitCount =0;
                    robot.retractJewelArm();
                    state = 18;
                }

                break;
            case 18:

                // make sure the angle is right and wait for vuforia to catch up
                if (0 == navigation.turnByGyroCloseLoop(0.0,
                        (double) robot.imu.getAngularOrientation().firstAngle,
                        fGlyphTurnAngle, leftMotors, rightMotors)
                        && System.currentTimeMillis() - timeStamp > 1000) {
                    state = 19;
                    getWheelLandmarks();
                    robot.levelGlyph();
                    navigation.resetTurn(leftMotors, rightMotors);
                    robot.retractJewelArm();
                    timeStamp = System.currentTimeMillis();
                }

                if ("unknown" == vuforia.vumarkImage.toLowerCase()) {
                    vuforia.identifyGlyphCrypto();
                }
                break;
            case 19: {

                if ("unknown" == vuforia.vumarkImage.toLowerCase()) {
                    vuforia.identifyGlyphCrypto();
                }
                OpenGLMatrix pose = vuforia.getGlyphCryptoPosition();

                // if too many errors, move on
                if (vuforiaMissCount > 18) {
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    state = 20  ;
                }

                if (null == pose) {
                    vuforiaMissCount++;
                } else {
                    telemetry.addData("Pose", format(pose));

                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    //double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);
                    double tD = robot.getVuforiaFrontBackDistance(pose);
                    double tG = robot.getVuforiaLeftRightDistance(pose);
                    telemetry.addData("vuforia distance Y=", tY);
                    telemetry.addData("vuforia distance Z=", tZ);
                    telemetry.addData("vuforia degree 1=", rot.firstAngle);
                    telemetry.addData("vuforia degree 2=", rot.secondAngle);
                    telemetry.addData("vuforia degree 3=", rot.thirdAngle);
                    telemetry.addData("Wall distance   =", tD);
                    telemetry.addData("Image distance   =", tG);

                    // adjust distance by vuforia values
                    double errorZ = vuforiaTargetDistance - tD;
                    telemetry.addData("vuforia distance error", errorZ);
                    if ( Math.abs(errorZ) > 200 || Math.abs(rot.secondAngle) > 30) {
                        vuforiaMissCount++;
                    } else {
                        if (Math.abs(errorZ) < 15) {
                            vuforiaHitCount ++;
                        } else {
                            vuforiaHitCount = 0;
                        }

                        if ( vuforiaHitCount > 10) {
                            state = 20;
                            getWheelLandmarks();
                            timeStamp = System.currentTimeMillis();
                            wheelDistanceLandMark = getWheelOdometer();
                            sideMoveAtPower(0);
                            // set glyph box distance. image to glyph box distance is 34.5 inch , camera to flipper distance is 4
                            cryptoBoxDistance = robot.imageDistance2GlyphBoxBDistance(tG);
                        } else {
                            //leftDiagonalMoveAtPower(Range.clip(errorZ * -0.0015, -0.35, 0.35));
                            sideMoveAtPower(Range.clip(errorZ * 0.015, -0.2, 0.2));
                            //sideMoveAtPower(Range.clip(navigation.getMaintainDistancePower(targetVuDis,tZ), -0.65, 0.65));
                        }
                        vuforiaMissCount = 0;
                    }
                }
            }
                break;
            case 20:
                vuforia.relicTrackables.deactivate();

                int  deltaDistance = 0;
                if (vuforia.vumarkImage == "left") {
                    state = 21;
                    getWheelLandmarks();
                    wheelDistanceLandMark = getWheelOdometer();
                    navigation.resetTurn(leftMotors, rightMotors);
                } else if (vuforia.vumarkImage == "center") {
                    deltaDistance = leftColumnDistance - centerColumnDistance;
                } else {
                    deltaDistance = leftColumnDistance - rightColumnDistance;
                }

                if (0 == sideMoveByDistance(-sideMovePower/2, deltaDistance)) {
                    wheelDistanceLandMark = getWheelOdometer();
                    getWheelLandmarks();
                    timeStamp = System.currentTimeMillis();
                    vuforiaMissCount = 0;
                    vuforiaHitCount =0;
                    robot.retractJewelArm();
                    robot.retractGlyphBlocker();
                    state = 21;
                }

                break;
            case 21:
                // correct angle just increase it got knocked out the cource
                if (0 == navigation.turnByGyroCloseLoop(0.0,
                        (double) robot.imu.getAngularOrientation().firstAngle,
                        fGlyphTurnAngle, leftMotors, rightMotors)) {
                    state = 22;
                    getWheelLandmarks();
                    robot.levelGlyph();
                    // lift
                    VortexUtils.moveMotorByEncoder(robot.liftMotor, glyphLiftPosition, liftMotorMovePower);
                    navigation.resetTurn(leftMotors, rightMotors);
                    timeStamp = System.currentTimeMillis();
                }
                break;
            case 22:
                // move to glyph
                if (0 == moveByDistance(glyphDeliverPower*2, cryptoBoxDistance)) {
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    state = 23;
                }
                break;
            case 23:
                // release glyph
                robot.dumpGlyph();
                if (System.currentTimeMillis() - timeStamp > 1000) {
                    getWheelLandmarks();
                    state = 24;
                }
                break;
            case 24:
                // push
                if (0 == moveByDistance(glyphDeliverPower*3, 300)) {
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    state = 25;
                }
                break;
            case 25:
                // back up
                if (0 == moveByDistance(-glyphDeliverPower*3, backupDistance+300)) {
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    state = 26;
                }
                break;
            default:
                robot.loadGlyph();
                robot.retractGlyphBlocker();
                VortexUtils.moveMotorByEncoder(robot.liftMotor, 0, liftMotorMovePower);
                // stop
                vuforia.relicTrackables.deactivate();
                robot.stop();
        }

        telemetry.addData("teamColor", teamColor);
        telemetry.addData("Crypto", vuforia.vumarkImage);
        telemetry.addData("state", state);
        telemetry.update();
    }


}
