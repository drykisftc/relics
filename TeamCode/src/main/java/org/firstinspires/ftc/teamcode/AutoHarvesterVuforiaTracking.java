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
@Autonomous(name = "Harvester_VuforiaTracking", group = "A_Harvester_Vuforia")

public class AutoHarvesterVuforiaTracking extends AutoRelic {


    protected BNO055IMU imuSensor = null;

    protected HardwareHarvester robot= null;

    protected int leftBackStamp;
    protected int leftFrontStamp;
    protected int rightBackStamp;
    protected int rightFrontStamp;

    protected OpenGLMatrix vuforiaMatrix;
    protected VectorF trans;
    protected Orientation rot;

    public AutoHarvesterVuforiaTracking() {
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
        state = 1;
        timeStamp = System.currentTimeMillis();
        vuforia.vumarkImage = "Unknown";
        jewelKicker.start();
        robot.initAllDevices();

    }

    @Override
    public void loop() {
        switch (state) {
            case 0:
                // look for vuforia picture
                vuforia.identifyGlyphCrypto();
                if (vuforia.vumarkImage != "Unknown") {

                    state = 1;

                }
                break;
            case 1:
                // follow vuforia
                if (vuforia.vumarkImage == "left" ||
                        vuforia.vumarkImage == "center" ||
                        vuforia.vumarkImage == "right") {
                    if (0 == moveToVuforia(vuforia.getGlyphCryptoPosition(), 0, 0, 200, 0, 0, -90, robot.axleDistance, leftMotors, rightMotors)) {
                        state = 0;
                    }
                } else {
                    state = 0;
                }
                break;
            default:
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
