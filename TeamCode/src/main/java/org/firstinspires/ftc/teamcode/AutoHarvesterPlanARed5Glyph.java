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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
@Autonomous(name = "Harvester_PlanA_Red_5_glyph", group = "A_Harvester_red")

public class AutoHarvesterPlanARed5Glyph extends AutoRelic {


    protected BNO055IMU imuSensor = null;

    protected HardwareHarvester robot= null;

    protected int deliverCount = 0;
    protected int RKSensorCount = 0;

    protected int leftBackStamp;
    protected int leftFrontStamp;
    protected int rightBackStamp;
    protected int rightFrontStamp;

    protected OpenGLMatrix vuforiaMatrix;
    protected VectorF trans;
    protected Orientation rot;

    final int leftVuDis = -968;  // 42.5 inches
    final int centerVuDis = -760; // 35 inches
    final int rightVuDis = -574; // 27.5 inches

    int vuforiaMissCount = 0;
    int vuforiaHitCount = 0;
    int vuforiaCheckDistance = 0;
    int vuforiaTargetDistance = -672;
    int cryptoBoxTargetDistance = -494; // 40 inches? change this landmark


    protected double distanceToVumark;

    public AutoHarvesterPlanARed5Glyph() {
        teamColor = "red";

        glyphLiftPosition = 1500;
        centerGlyphAngleOffset = 0;
        vuforiaDetectingPower = -1.0;
        move2GlyphBoxPower = -0.6;
        move2CenterPower = 0.8;
        fGlyphTurnAngle = -90;
        center2GlyphBoxPower = -0.8;
        glyTurnPower = -0.4;
        glyphOffAngle = 45;

        cryptoBoxDistance = 500;
        center2GlyphDistance = 3500;
        pushDistance = 500;

//        leftColumnDistance = 3860;
//        centerColumnDistance = 3150;
//        rightColumnDistance = 2500;
        leftColumnDistance = 3600;
        centerColumnDistance = 2950;
        rightColumnDistance = 2300;

        columnDistance = rightColumnDistance;
        RKArmDistance = 6; // in cm

        glyph2CenterDistance = 1800;

        sideMovePower = 0.8;

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
        robot.defaultGlyphWheelPower = 0.5;

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

//        vuforia = new HardwareVuforia(VuforiaLocalizer.CameraDirection.BACK);
//        vuforia.init(hardwareMap);

        telemetry.addData("jewelArm", jewelArm.getPosition());
        telemetry.addData("jewelHitter", jewelHitter.getPosition());
        telemetry.update();
    }


    @Override
    public void start() {
        robot.start();
//        vuforia.start();
        state = 0;
        timeStamp = System.currentTimeMillis();
//        vuforia.vumarkImage = "unknown";
        jewelKicker.start();
        robot.initAllDevices();

    }

    @Override
    public void loop() {
        /*
        state 4 releases the glyph
        state 16 backs up to the crypto box after collections
        code loops back to state 4 from 14 to repeat deposite process

          */
        switch (state) {
            case 0:
//                cryptoBoxDistance = 50;
//                vuforiaMissCount = 0;
//                vuforiaHitCount = 0;
//                resetDeliverHistory2(2);

                robot.retractGlyphBlocker();

                // jewel handling
                state = jewelKicker.loop(0, 1, teamColor);

                // move jewel arm to avoid jewel holes
                jewelKicker.jewelArmActionPosition = jewelArmPos + 0.08*rand.nextDouble()-0.04;
                jewelKicker.jewelHitterRestPosition = jewelHitterPos + 0.02*rand.nextDouble()-0.01;

                robot.levelGlyph();

                //read vumark
//                computeGlyphColumnDistance();

                getWheelLandmarks();

                robot.retractGlyphBlocker();

                timeStamp = System.currentTimeMillis();

                break;
            case 1:
                // lift glyph bar
                VortexUtils.moveMotorByEncoder(robot.liftMotor, 10, liftMotorHolderPower);

                //set jewel hitter position
                robot.retractJewelArm();

                //move forward with encoder
                //read vumark
//                    double movePower = vuforiaDetectingPower;
//                    if ("unknown" == vuforia.vumarkImage.toLowerCase()) {
//                        computeGlyphColumnDistance();
//                    } else {
//                        movePower = vuforiaDetectingPower * 3.0;
//                    }
//
                if (0 == moveByDistance(vuforiaDetectingPower, columnDistance)) {
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
                // back up to crypto box
                if (0 == moveByDistance(move2GlyphBoxPower, cryptoBoxDistance)) {
                    moveAtPower(0.0);
//                    robot.extendRKArm();
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    state = 4;
                }

                break;
            case 4:
                // use ultra sensor
                if (0 == fitColumn(0.2)) {
                    moveAtPower(0.0);
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    state = 5;
                }
                /*robot.extendRKArm();

                // correct position with RKArm
                if (robot.RKSensor.getDistance(DistanceUnit.CM) > RKArmDistance + 0.3) {
                    sideMoveAtPower(-0.2);
                } else if (robot.RKSensor.getDistance(DistanceUnit.CM) < RKArmDistance - 0.3) {
                    sideMoveAtPower(0.2);
                } else {
                    RKSensorCount ++;
                    moveAtPower(0.0);
                }

                if (RKSensorCount > 5) {
                    moveAtPower(0.0);
                    robot.retractRKArm();
                    timeStamp = System.currentTimeMillis();
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    state = 5;
                }

                telemetry.addData("RKSensor distance: ", robot.RKSensor.getDistance(DistanceUnit.CM));*/
                break;
            case 5:
                // release the glyph
                time = System.currentTimeMillis();

                if (time - timeStamp < 800) {
                    robot.dumpGlyph();
                } else {

                    robot.retractRKArm();
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    deliverCount ++;
                    state = 6;

                }

                break;

            case 6:
                // Move forward from the crypto box
                if (0 == moveByDistance(-move2GlyphBoxPower, pushDistance)) {

                    moveAtPower(0.0);
                    robot.loadGlyph();
                    timeStamp = System.currentTimeMillis();

                    // lower glyph bars
                    VortexUtils.moveMotorByEncoder(robot.liftMotor, 0, liftMotorHolderPower);

                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    state = 7;

                }
                break;
            case 7:
                // push glyph into place
                if (0 == moveByDistance(move2GlyphBoxPower, pushDistance + 200)) {
                    moveAtSpeed(0.0);
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                    state = 8;
                }
                break;
            case 8:
                // move forward from the crypto box
                if (0 == moveByDistance(0.8, 500)) {

                    moveAtPower(0.0);
                    robot.loadGlyph();
                    timeStamp = System.currentTimeMillis();
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();

                    if (deliverCount >= 3) {
                        state = 20;
                    } else {
                        state = 9;
                    }

                }
                break;
            case 9:
                // use gyro to adjust angle
                robot.retractJewelArm();
                if (0 == navigation.turnByGyroCloseLoop(0.0,
                        (double) robot.imu.getAngularOrientation().firstAngle,
                        fGlyphTurnAngle, leftMotors, rightMotors)) {
                    moveAtPower(0.0);
                    robot.loadGlyph();
                    VortexUtils.moveMotorByEncoder(robot.liftMotor, 0, liftMotorMovePower);
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    wheelDistanceLandMark = getWheelOdometer();
                    timeStamp = System.currentTimeMillis();
                    state = 10;
                }
                break;
            case 10:
                // Go to next column
                robot.retractJewelArm();

                if (0 == sideMoveByDistance(-sideMovePower, 550)) {
                    moveAtPower(0.0);
                    robot.glyphWheelLoad();
                    robot.loadGlyph();
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                    state = 11;
                }
                break;
            case 11:
                // drive forward to center fast
                if (0 == moveByDistance(move2CenterPower, (int)(glyph2CenterDistance*0.75) + backupDistance + 1200)) {
                    moveAtPower(0.0);
                    robot.glyphWheelLoad();
                    robot.loadGlyph();
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                    state = 12;
                }
                break;
            case 12:
                // drive forward to center slower
                if (0 == moveByDistance(collectingGlyphPower, (int)(glyph2CenterDistance*0.25))) {
                    moveAtPower(0.0);
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                    state = 13;
                }

                if (robot.haveGlyph()) {
                    state = 15;
                }

                break;
            case 13:
                // wiggle(Wiggin)
                navigation.turnByGyroCloseLoop(0.0,
                        (double) robot.imu.getAngularOrientation().firstAngle,
                        -45+rand.nextInt(20)-10,
                        leftMotors, rightMotors);

                // move forward more
                if (0 == moveByDistance(collectingGlyphPower, (int)(glyph2CenterDistance*0.2))) {
                    moveAtPower(0.0);
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                    state = 14;
                }

                if (robot.haveGlyph()) {
                    state = 15;
                }
                break;
            case 14:
                // wait 1 seconds for robot to collect glyph
                if (System.currentTimeMillis() - timeStamp > 1000) {
                    state = 15;
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                    timeStamp = System.currentTimeMillis();
                }

                if (robot.haveGlyph()) {
                    state = 15;
                }

                break;
            case 15:
                // use gyro to adjust angle
                robot.retractJewelArm();
                if (0 == navigation.turnByGyroCloseLoop(0.0,
                        (double) robot.imu.getAngularOrientation().firstAngle,
                        fGlyphTurnAngle,leftMotors,rightMotors)) {
                    moveAtPower(0.0);
                    robot.levelGlyph();
                    VortexUtils.moveMotorByEncoder(robot.liftMotor, 0, liftMotorMovePower);
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    wheelDistanceLandMark = getWheelOdometer();
                    timeStamp = System.currentTimeMillis();
                    state = 16;
                }
                break;
            case 16:
                // unload extra glyph
                robot.glyphWheelUnload();


                // back up to crypto box
                if (0 == moveByDistance(center2GlyphBoxPower, center2GlyphDistance/* + 500*/)) {
                    moveAtPower(0.0);
//                    robot.extendRKArm();
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    timeStamp = System.currentTimeMillis();

                    state = 17;
                }
                break;
            case 17:
                //move to 25 cm
                if (0 == moveByBackUltrasonicSensor(0.3, 25)) {
                    moveAtPower(0.0);
//                    robot.extendRKArm();
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    timeStamp = System.currentTimeMillis();
                    state = 18;
                }

                break;
            case 18:
                //move the last distance
                if (0 == moveByDistance(-0.4, robot.CM2Encoder(5))) {
                    moveAtPower(0.0);
//                    robot.extendRKArm();
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    timeStamp = System.currentTimeMillis();
                    state = 4;
                }
                break;
            case 19:
                // move left to ensure robot is inside parking zone
                if (0 == sideMoveByDistance(-0.8, 650)) {
                    moveAtPower(0.0);
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    timeStamp = System.currentTimeMillis();

                    state = 20;
                }
                break;
            default:
                robot.loadGlyph();
                VortexUtils.moveMotorByEncoder(robot.liftMotor, 0, liftMotorMovePower);
                // stop
//                vuforia.relicTrackables.deactivate();
                robot.stop();
                break;
        }

        telemetry.addData("teamColor", teamColor);
//        telemetry.addData("vumark", vuforia.vumarkImage);
        telemetry.addData("state", state);
        telemetry.addData("backupDistance", backupDistance);
        telemetry.addData("devilver index", deliverIndex);
        telemetry.addData("devilver count", deliverCount);
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

    public void rightDiagonalMoveAtPower(double p) {
        robot.motorRightFrontWheel.setPower(p);
        robot.motorLeftBackWheel.setPower(p);
    }

    public void leftDiagonalMoveAtPower(double p) {
        robot.motorLeftFrontWheel.setPower(p);
        robot.motorRightBackWheel.setPower(p);
    }

    public void moveAtSpeed(double p){
        robot.motorLeftFrontWheel.setPower(p);
        robot.motorRightBackWheel.setPower(p);
        robot.motorRightFrontWheel.setPower(p);
        robot.motorLeftBackWheel.setPower(p);
    }

    // positive power moves left
    public int sideMoveByDistance (double power, int d) {
        int distance = Math.abs(d)*2;
        if (power == 0) {
            return 0; // zero power do nothing
        }
        if (Math.abs(robot.motorRightBackWheel.getCurrentPosition() - rightBackStamp) + Math.abs(robot.motorLeftFrontWheel.getCurrentPosition() - leftFrontStamp) +
                 Math.abs(robot.motorLeftBackWheel.getCurrentPosition() - leftBackStamp) + Math.abs(robot.motorRightFrontWheel.getCurrentPosition() - rightFrontStamp) < distance) {
            sideMoveAtPower(power);
        } else {
            sideMoveAtPower(0.0);
            return 0;
        }
        return 1;
    }


    public int leftDiagonalMoveByDistance(double power, int d) {
        int distance = Math.abs(d);
        if (power == 0) {
            return 0; // zero power do nothing
        }

        if (Math.abs(robot.motorRightBackWheel.getCurrentPosition() - rightBackStamp) +
                Math.abs(robot.motorLeftFrontWheel.getCurrentPosition() - leftFrontStamp) < distance) {
            leftDiagonalMoveAtPower(power);
        } else {
            leftDiagonalMoveAtPower(0.0);
            return 0;
        }
        return 1;
    }

    public int rightDiagonalMoveByDistance(double power, int d) {
        int distance = Math.abs(d);
        if (power == 0) {
            return 0; // zero power do nothing
        }
        if (Math.abs(robot.motorLeftBackWheel.getCurrentPosition() - leftBackStamp) +
                Math.abs(robot.motorRightFrontWheel.getCurrentPosition() - rightFrontStamp) < distance*2) {
            rightDiagonalMoveAtPower(power);
        } else {
            rightDiagonalMoveAtPower(0.0);
            return 0;
        }
        return 1;
    }

    public int moveByBackUltrasonicSensor(double power, int d) {
        int distance = Math.abs(d);
        if (robot.backDistanceSensor.getDistance(DistanceUnit.CM) > distance + 3) {
            moveAtPower(-power);
        } else if (robot.backDistanceSensor.getDistance(DistanceUnit.CM) < distance - 3) {
            moveAtPower(power);
        } else {
            moveAtPower(0.0);
            return 0;
        }

        return 1;
    }

    public int fitColumn(double power) {
        double ultraReading = robot.backDistanceSensor.getDistance(DistanceUnit.CM);
        double lastUltraReading = ultraReading;

        if (ultraReading - lastUltraReading > 6) {
            return 0;
        } else {
            sideMoveAtPower(power);
        }

        return 1;
    }

    public void stopGlyphWheels(){
        robot.leftLiftWheel.setPower(0.0);
        robot.rightLiftWheel.setPower(0.0);
    }

}
