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

/*
 * This is an example LinearOpMode that shows how to use
 * the REV Robotics Color-Distance Sensor.
 *
 * It assumes the sensor is configured with the name "sensor_color_distance".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 */
@Autonomous(name = "VLSB_PlanB_Red", group = "B_VLSB")

public class AutoVLSBPlanBRed extends AutoVLSBPlanARed {

    int sideWayDistance = 6200;

    public AutoVLSBPlanBRed() {

        super();

        teamColor = "red";
        fGlyphTurnAngle = 0.0f;

        leftColumnDistance = 3800;
        centerColumnDistance = 2350;
        rightColumnDistance = 900;

        glyph2CenterDistance = 3000;

        backupDistance = 500;

        sideMovePower = -0.2;

        glyphDeliverPower = -0.2;
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

                vuforia.identifyGlyphCrypto();
                getWheelLandmarks();

                break;
            case 1:

                //read vumark
                computeGlyphColumnDistance();

                // lift glyph bar
                VortexUtils.moveMotorByEncoder(robot.liftMotor, glyphLiftPosition, liftMotorHolderPower);

                //set jewel hitter position
                robot.jewelHitter.setPosition(1.00);
                robot.beltDepositGlyph();

                if (0 == moveByDistance(vuforiaDetectingPower, offBalanceStoneDistance)) {
                    moveAtPower(0.0);
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                    state = 2;
                }

                break;
            case 2:
                // move left
                if ( 0 == sideMoveByDistance(sideMovePower, columnDistance) ){
                    wheelDistanceLandMark = getWheelOdometer();
                    state = 3;
                }

                break;
            case 3: // turn if necessary
                if (fGlyphTurnAngle == 0.0f || 0 == navigation.turnByEncoderOpenLoop(glyTurnPower,fGlyphTurnAngle,
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

                if (time - timeStamp < 1000) {
                    releaseGlyph();
                } else {

                    timeStamp = System.currentTimeMillis();
                    state = 6;

                }

                break;
            case 6:
                //back up
                if (0 == moveByDistance(-glyphDeliverPower, backupDistance)) {
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    state = 20;
                }

                break;
            case 7:
                // move side way
                if ( 0 == sideMoveByDistance(sideMovePower, sideWayDistance-columnDistance) ){
                    wheelDistanceLandMark = getWheelOdometer();
                    getWheelLandmarks();
                    VortexUtils.moveMotorByEncoder(robot.liftMotor, 0, liftMotorHolderPower);
                    collectGlyph();
                    state = 8;
                }

                break;
            case 8:
                // move to center
                if (0 == moveByDistance(move2CenterPower, glyph2CenterDistance)) {
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    state = 9;
                }
                break;

            case 9:
                // collect glyph
                if ( System.currentTimeMillis() - timeStamp < 2000) {
                    state = 10;
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                }
                break;
            case 10:
                // correct angle just increase it got knocked out the cource
                if (0 == navigation.turnByGyroCloseLoop(0.0, (double) robot.imu.getAngularOrientation().firstAngle,fGlyphTurnAngle,leftMotors,rightMotors)) {
                    state = 11;
                    getWheelLandmarks();
                    // lift glyph bar
                    VortexUtils.moveMotorByEncoder(robot.liftMotor, glyphLiftPosition*3, liftMotorHolderPower);
                    navigation.resetTurn(leftMotors, rightMotors);
                }
                break;
            case 11:
                // move away from center
                if (0 == moveByDistance(-move2CenterPower, glyph2CenterDistance)) {
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    state = 12;
                }
                break;
            case 12:
                // move side way
                if ( 0 == sideMoveByDistance(-sideMovePower, sideWayDistance-columnDistance) ){
                    wheelDistanceLandMark = getWheelOdometer();
                    getWheelLandmarks();
                    VortexUtils.moveMotorByEncoder(robot.liftMotor, 0, liftMotorHolderPower);
                    collectGlyph();
                    state = 13;
                }
                break;
            case 13:
                // move to glyph
                if (0 == moveByDistance(glyphDeliverPower, -backupDistance)) {
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    state = 14;
                }
                break;
            case 14:
                // release glyph
                releaseGlyph();
                if (System.currentTimeMillis() - timeStamp > 1000) {
                    getWheelLandmarks();
                    state = 15;
                }
                break;
            case 15:
                // back up
                if (0 == moveByDistance(-glyphDeliverPower, backupDistance)) {
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    state = 16;
                }
                break;
            case 16:
                // move side way
                if ( 0 == sideMoveByDistance(-sideMovePower, columnDistance) ){
                    wheelDistanceLandMark = getWheelOdometer();
                    getWheelLandmarks();
                    VortexUtils.moveMotorByEncoder(robot.liftMotor, 0, liftMotorHolderPower);
                    collectGlyph();
                    state = 17;
                }
                break;
            case 17:
                // move to center
                if (0 == moveByDistance(glyphDeliverPower, -glyph2CenterDistance*2)) {
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    collectGlyph();
                    state = 18;
                }
                break;
            case 18:
                //collect glyphs
                if ( System.currentTimeMillis() - timeStamp > 2000) {
                    stopGlyphWheels();
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                    state = 19;
                }
                break;
            case 19:
                // turn 180 degrees
                if (0 == navigation.turnByGyroCloseLoop(0.0, (double) robot.imu.getAngularOrientation().firstAngle,180,leftMotors,rightMotors)) {
                    state = 20;
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                }
                break;
            case 20:
                // move back to crypto box
                if (0 == moveByDistance(-glyphDeliverPower, -glyph2CenterDistance*2)) {
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    state = 21;
                }
                break;
            case 21:
                // move side way
                if (columnDistance == leftColumnDistance) {
                    columnDistance = centerColumnDistance;
                } else if (columnDistance == centerColumnDistance) {
                    columnDistance = leftColumnDistance;
                } else if (columnDistance == rightColumnDistance) {
                    columnDistance = leftColumnDistance;
                }

                if ( 0 == sideMoveByDistance(-sideMovePower, columnDistance) ){
                    wheelDistanceLandMark = getWheelOdometer();
                    getWheelLandmarks();
                    VortexUtils.moveMotorByEncoder(robot.liftMotor, 0, liftMotorHolderPower);
                    state = 22;
                }
                break;
            case 22:
                // deposite glyph
                reverseWheels();
                if (System.currentTimeMillis() - timeStamp > 3000) {
                    getWheelLandmarks();
                    state = 23;
                }
                break;
            case 23:
                // backup
                if (0 == moveByDistance(move2GlyphBoxPower, 300)) {
                    moveAtPower(0.0);
                    state = 24;
                }
                break;
            default:
                // stop
                vuforia.relicTrackables.deactivate();
                robot.stop();
        }

        telemetry.addData("teamColor", teamColor);
        telemetry.update();
    }
}
