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

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.SystemProperties;

/*
 * This is an example LinearOpMode that shows how to use
 * the REV Robotics Color-Distance Sensor.
 *
 * It assumes the sensor is configured with the name "sensor_color_distance".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 */
@Autonomous(name = "RianPlanB_Red", group = "Rian")

public class AutoRianPlanBRed extends AutoRianPlanARed {

    @Override
    public void start() {
        super.start();
        teamColor = "red";
        fGlyphTurnAngle = 180.0f;

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

                break;
            case 1:

                //read vumark
                vuforia.identifyGlyphCrypto();
                if (vuforia.vumarkImage == "left") {
                    columnDistance = leftColumnDistance;
                } else if (vuforia.vumarkImage == "center") {
                    columnDistance = centerColumnDistance;
                } else if (vuforia.vumarkImage == "right") {
                    columnDistance = rightColumnDistance;
                } else {
                    columnDistance = rightColumnDistance;
                }

                OpenGLMatrix pose = vuforia.getGlyphCryptoPosition();
                telemetry.addData("Pose", format(pose));

                //move forward with encoder
                wheelDistanceAverage = (robot.motorLeftBackWheel.getCurrentPosition() +
                        robot.motorLeftFrontWheel.getCurrentPosition() +
                        robot.motorRightBackWheel.getCurrentPosition() +
                        robot.motorRightFrontWheel.getCurrentPosition())/4;

                if (wheelDistanceAverage < 2600) {

                    moveAtSpeed(vuforiaDetectingSpeed);

                } else {

                    moveAtSpeed(0.0);
                    leftBackStamp = robot.motorLeftBackWheel.getCurrentPosition();
                    leftFrontStamp = robot.motorLeftFrontWheel.getCurrentPosition();
                    rightBackStamp = robot.motorRightBackWheel.getCurrentPosition();
                    rightFrontStamp = robot.motorRightFrontWheel.getCurrentPosition();
                    state = 2;

                }

                break;
            case 2:
                // move left
                if (robot.motorRightBackWheel.getCurrentPosition() - rightBackStamp + robot.motorLeftFrontWheel.getCurrentPosition() - leftFrontStamp > -columnDistance && robot.motorLeftBackWheel.getCurrentPosition() - leftBackStamp + robot.motorRightFrontWheel.getCurrentPosition() - rightFrontStamp < columnDistance) {

                    robot.motorLeftFrontWheel.setPower(-0.2);
                    robot.motorRightBackWheel.setPower(-0.2);
                    robot.motorRightFrontWheel.setPower(0.2);
                    robot.motorLeftBackWheel.setPower(0.2);

                } else {

                    wheelDistanceLandMark = (robot.motorLeftBackWheel.getCurrentPosition() +
                                                 robot.motorLeftFrontWheel.getCurrentPosition() +
                                                 robot.motorRightBackWheel.getCurrentPosition() +
                                                 robot.motorRightFrontWheel.getCurrentPosition())/4;
                    state = 3;

                }

                break;

            case 3:

                wheelDistanceAverage = (robot.motorLeftBackWheel.getCurrentPosition() +
                                        robot.motorLeftFrontWheel.getCurrentPosition() +
                                        robot.motorRightBackWheel.getCurrentPosition() +
                                        robot.motorRightFrontWheel.getCurrentPosition())/4;

                if (wheelDistanceAverage - wheelDistanceLandMark < cryptoBoxDistance) {

                    moveAtSpeed(0.2);

                } else {

                    timeStamp = System.currentTimeMillis();
                    state = 4;

                }

                break;
            case 4:
                // release the glyph

                time = System.currentTimeMillis();

                if (time - timeStamp < 1000) {

                    robot.leftLiftWheel1.setPower(1.0);
                    robot.leftLiftWheel2.setPower(1.0);
                    robot.leftLiftWheel3.setPower(1.0);
                    robot.rightLiftWheel1.setPower(-1.0);
                    robot.rightLiftWheel2.setPower(-1.0);
                    robot.rightLiftWheel3.setPower(-1.0);

                } else {

                    timeStamp = System.currentTimeMillis();
                    state = 5;

                }

                break;
            case 5:
                //back up
                time = System.currentTimeMillis();

                if (time - timeStamp < 1000) {

                    moveAtSpeed(-0.1);

                } else {

                    moveAtSpeed(0.0);
                    state = 6;

                }

                break;
            case 6:
                // stop
                robot.stop();

                break;
            default:
        }

        telemetry.update();
    }
}
