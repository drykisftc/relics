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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.Random;

@Autonomous(name = "Nathen_PlanB_Red", group = "Nathen")
@Disabled
public class AutoNathanPlanBRed extends AutoNathanPlanARed {

   int columnStopDistance = 2186;
   double columnTurnAngle = 90;
   double glyphAngle = 0.0;

    public AutoNathanPlanBRed () {
        // team specific
        super();

        cryptoBoxDistance = 600;

        rightColumnDistance = 350;
        centerColumnDistance = (int)(rightColumnDistance + 7.63*encoderCountPerInch);
        leftColumnDistance = (int)(rightColumnDistance + 15.26*encoderCountPerInch);

        columnDistance = rightColumnDistance;
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
                if ( 0 == moveByDistance(vuforiaDetectingPower, columnStopDistance )) {
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
                //if (0 == navigation.turnByEncoderOpenLoop(glyTurnPower,columnTurnAngle, robot.axleDistance, leftMotors, rightMotors)) {
                if (0 == navigation.turnByGyroCloseLoop(glyTurnPower,robot.gyro.getHeading(), columnTurnAngle, leftMotors, rightMotors)) {
                    state = 4;
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                }
                break;

            case 4:
                 if (0 == moveByDistance(0.2, columnDistance)) {
                    moveAtPower(0.0);
                    timeStamp = System.currentTimeMillis();
                    state = 5;
                }
                break;
            case 5:
                // turn by encoder
                if (0 == navigation.turnByGyroCloseLoop(glyTurnPower,robot.gyro.getHeading(), glyphAngle, leftMotors, rightMotors)) {
                    state = 6;
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                }
                break;
            case 6:
                // move straight
                if (0 == moveByDistance(0.2, cryptoBoxDistance)) {
                    moveAtPower(0.0);
                    timeStamp = System.currentTimeMillis();
                    state = 7;
                }

                break;
            case 7:
                // release the glyph
                time = System.currentTimeMillis();
                if (time - timeStamp < 1000) {
                    robot.leftHand.setPosition(robot.leftHandOpenPosition);
                    robot.rightHand.setPosition(robot.rightHandOpenPosition);
                } else {
                    state = 8;
                    getWheelLandmarks();
                    moveAtPower(-0.2);
                    timeStamp = System.currentTimeMillis();
                }
                break;
            case 8:
                // backup
                if (0 == moveByDistance(-0.25, backupDistance)) {
                    moveAtPower(0.0);
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                    // lower glyph bars
                    VortexUtils.moveMotorByEncoder(robot.liftMotor, 0, robot.liftMotorHolderPower);

                    state = 9;
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
