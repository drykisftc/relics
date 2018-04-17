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
@Autonomous(name = "Harvester_PlanA_Red_VF", group = "A_Harvester_red")

public class AutoHarvesterPlanARedVF extends AutoRelic {


    protected BNO055IMU imuSensor = null;

    protected HardwareHarvester robot= null;

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

    public AutoHarvesterPlanARedVF() {
        teamColor = "red";

        glyphLiftPosition = 1500;
        centerGlyphAngleOffset = 0;
        vuforiaDetectingPower = -0.15;
        move2GlyphBoxPower = -0.6;
        move2CenterPower = 0.8;
        fGlyphTurnAngle = -90;
        center2GlyphBoxPower = -0.8;
        glyTurnPower = -0.4;
        glyphOffAngle = 45;

        cryptoBoxDistance = 400;
        center2GlyphDistance = 3500;

//        leftColumnDistance = 3860;
//        centerColumnDistance = 3150;
//        rightColumnDistance = 2500;
        leftColumnDistance = 3500;
        centerColumnDistance = 2850;
        rightColumnDistance = 2200;

        glyph2CenterDistance = 1800;

        sideMovePower = 0.65;

    }

    @Override
    public void init() {
        robot = new HardwareHarvester();
        robot.init(hardwareMap);

        robot.defaultGlyphWheelPower = 1.0;

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
        robot.relicMotor.setPower(0.005);
        robot.relicFlipper.setPosition(0.5);
    }

    @Override
    public void loop() {
        switch (state) {
            case 0:
                cryptoBoxDistance = 250;
                pushDistance = 550;
                robot.defaultGlyphWheelPower = 0.5;
                vuforiaMissCount = 0;
                vuforiaHitCount = 0;
                collectionDistance = 0;
                resetDeliverHistory2(2);

                robot.retractGlyphBlocker();

                // jewel handling
                state = jewelKicker.loop(0, 1, teamColor);

                // move jewel arm to avoid jewel holes
                jewelKicker.jewelArmActionPosition = jewelArmPos + 0.08*rand.nextDouble()-0.04;
                jewelKicker.jewelHitterRestPosition = jewelHitterPos + 0.02*rand.nextDouble()-0.01;

                robot.levelGlyph();

                //read vumark
                computeGlyphColumnDistance();

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
                    double movePower = vuforiaDetectingPower;
                    if ("unknown" == vuforia.vumarkImage.toLowerCase()) {
                        computeGlyphColumnDistance();
                    } else {
                        movePower = Range.clip(vuforiaDetectingPower * 5.0, -1.0, 1.0);
                    }

                    if (0 == moveByDistance(movePower, columnDistance)) {
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
                if (robot.backDistanceSensor.getDistance(DistanceUnit.INCH) < 12
                || 0 == moveByDistance(move2GlyphBoxPower, cryptoBoxDistance)) {
                    moveAtPower(0.0);
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    state = 4;
                }

                break;
            case 4:
                updateDeliverHistory();
                // release the glyph
                time = System.currentTimeMillis();

                if (time - timeStamp < 1000) {
                    robot.dumpGlyph();
                } else {

                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    state = 6;
                }

                break;

            case 5:
                // back up from the crypto box
                if (0 == moveByDistance(-move2GlyphBoxPower, pushDistance)) {

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
                if (0 == moveByDistance(move2GlyphBoxPower, pushDistance)) {
                    moveAtSpeed(0.0);
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                    state = 7;
                }
                break;
            case 7:
                // back up from the crypto box
                if (0 == moveByDistance(0.8, 500)) {

                    moveAtPower(0.0);
                    robot.loadGlyph();
                    timeStamp = System.currentTimeMillis();

                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    state = 9;

                }
                break;
            case 8:
                // use gyro to adjust angle
                robot.retractJewelArm();
                if (0 == navigation.turnByGyroCloseLoop(0.0,
                        (double) robot.imu.getAngularOrientation().firstAngle,
                        fGlyphTurnAngle,leftMotors,rightMotors)) {
                    moveAtPower(0.0);
                    robot.loadGlyph();
                    VortexUtils.moveMotorByEncoder(robot.liftMotor, 0, liftMotorMovePower);
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    wheelDistanceLandMark = getWheelOdometer();
                    timeStamp = System.currentTimeMillis();
                    state = 9;
                }
                break;
            case 9:
                robot.retractJewelArm();
                if ("right" == vuforia.vumarkImage.toLowerCase()
                        || "unknown" == vuforia.vumarkImage.toLowerCase()) {
                    if (0 == sideMoveByDistance(-sideMovePower, 800)) {
                        wheelDistanceLandMark = getWheelOdometer();
                        getWheelLandmarks();
                        timeStamp = System.currentTimeMillis();
                        vuforiaMissCount = 0;
                        vuforiaHitCount =0;
                        robot.retractJewelArm();
                        robot.retractGlyphBlocker();
                        navigation.resetTurn(leftMotors, rightMotors);
                        state = 11;
                    }
                }
                // side move to the right column
                if (0 == sideMoveByDistance(sideMovePower, (columnDistance - rightColumnDistance)*2)) {
                    wheelDistanceLandMark = getWheelOdometer();
                    getWheelLandmarks();
                    robot.extendGlyphBlocker();
                    navigation.resetTurn(leftMotors, rightMotors);
                    state = 11;
                }

                break;
            case 10:
                // use gyro to adjust angle
                robot.retractJewelArm();
                if (0 == navigation.turnByGyroCloseLoop(0.0,
                        (double) robot.imu.getAngularOrientation().firstAngle,
                        fGlyphTurnAngle,leftMotors,rightMotors)) {
                    moveAtPower(0.0);
                    robot.loadGlyph();
                    robot.glyphWheelLoad();
                    VortexUtils.moveMotorByEncoder(robot.liftMotor, 0, liftMotorMovePower);
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    wheelDistanceLandMark = getWheelOdometer();
                    timeStamp = System.currentTimeMillis();
                    state = 11;
                }
                break;
            case 11:
                robot.retractJewelArm();
                 if (System.currentTimeMillis() - timeStamp > 800) {
                     robot.glyphWheelLoad();
                 }
                // move to center
                if (0 == moveByDistance(move2CenterPower, (int)(glyph2CenterDistance))) {
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                    state = 12;
                }
                break;
            case 12:
                // turn 45 degree
                 robot.glyphWheelLoad();
                if (0 == navigation.turnByEncoderOpenLoop(glyTurnPower, 45,
                        robot.axleDistance, leftMotors, rightMotors)) {
                    getWheelLandmarks();
                    robot.glyphWheelLoad();
                    navigation.resetTurn(leftMotors, rightMotors);
                    state = 13;
                }
                break;
            case 13:
                // back up
                if (0 == moveByDistance(-move2CenterPower, 300)) {
                    moveAtPower(0.0);
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    timeStamp = System.currentTimeMillis();
                    navigation.resetTurn(leftMotors, rightMotors);
                    state = 14;
                }
                break;
            case 14:
                robot.retractJewelArm();
                // If the glyphDistance is not NaN, jump to case 15
                if (Double.isNaN(robot.glyphDistance.getDistance(DistanceUnit.CM)) == false) {
                    moveAtPower(0.0);
                    collectionDistance = (int)(getWheelOdometer() - wheelDistanceLandMark);
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    timeStamp = System.currentTimeMillis();
                    state = 15;
                }
                //wiggle
                navigation.turnByGyroCloseLoop(0.0,
                        (double) robot.imu.getAngularOrientation().firstAngle,
                        -45+rand.nextInt(20)-10,
                        leftMotors, rightMotors);
                // move to center slower to collect glyph
                if (0 == moveByDistance(collectingGlyphPower, 2500)) {
                    moveAtPower(0.0);
                    collectionDistance = (int)(getWheelOdometer() - wheelDistanceLandMark);
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    timeStamp = System.currentTimeMillis();
                    state = 15;
                }
                break;
            case 15:
                // back up
                robot.retractJewelArm();

                // back up from glyph
                if (0 == moveByDistance(-rushPower, Math.min(0,1100+collectionDistance))) {
                    moveAtPower(0.0);
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    wheelDistanceLandMark = getWheelOdometer();
                    timeStamp = System.currentTimeMillis();
                    robot.levelGlyph();
                    state = 16;
                }
                break;
            case 16:
                // back up
                robot.retractJewelArm();
                if (System.currentTimeMillis() - timeStamp > 900) {
                    robot.glyphWheelUnload();
                    robot.retractGlyphBlocker();
                    VortexUtils.moveMotorByEncoder(robot.liftMotor, glyphLiftPosition, liftMotorMovePower);
                }

                // back up from glyph
                if (0 == leftDiagonalMoveByDistance(-rushPower, 7000)) {

                    moveAtPower(0.0);
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    timeStamp = System.currentTimeMillis();
                    robot.glyphWheelUnload();
                    robot.retractGlyphBlocker();
                    VortexUtils.moveMotorByEncoder(robot.liftMotor, glyphLiftPosition, liftMotorMovePower);

                    state = 17;
                }
                break;
            case 17:
                vuforiaMissCount = 0;
                // find vuforia mark
                if ("unknown" == vuforia.vumarkImage.toLowerCase()) {
                    vuforia.identifyGlyphCrypto();
                }
                OpenGLMatrix pose1 = vuforia.getGlyphCryptoPosition();

                // turn 45 degrees
                if (0 == navigation.turnByGyroCloseLoop(0.0,
                        (double) robot.imu.getAngularOrientation().firstAngle,
                        -45,leftMotors,rightMotors)
                        || pose1 != null) {
                    moveAtPower(0.0);
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    timeStamp = System.currentTimeMillis();
                    state = 18;
                }
                break;
            case 18:
                // move to target image distance
                OpenGLMatrix pose = vuforia.getGlyphCryptoPosition();
                telemetry.addData("vuforiaMissCount   =", vuforiaMissCount);
                telemetry.addData("vuforiaHitCount", vuforiaHitCount);
                // if too many errors, move on
                if (vuforiaMissCount > 200) {
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                    cryptoBoxDistance = 0;
                    backupDistance = 200;
                    state = 19;
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
                    double errorY = cryptoBoxTargetDistance - tG;
                    telemetry.addData("vuforia Z error", errorZ);
                    telemetry.addData("vuforia Y error", errorY);
                    if ( Math.abs(rot.secondAngle) > 80) {
                        vuforiaMissCount++;
                    } else {
                        if (Math.abs(errorY) < 15) {
                            vuforiaHitCount++;
                        } else {
                            vuforiaHitCount = 0;
                        }

                        if ( vuforiaHitCount > 30) {
                            state = 19;
                            getWheelLandmarks();
                            timeStamp = System.currentTimeMillis();
                            wheelDistanceLandMark = getWheelOdometer();
                            navigation.resetTurn(leftMotors, rightMotors);
                            sideMoveAtPower(0);
                            // set glyph box distance. image to glyph box distance is 34.5 inch , camera to flipper distance is 4
                            //cryptoBoxDistance = robot.imageDistance2GlyphBoxADistance(tG, columnDistance);
                            backupDistance = robot.robotToCryptoBoxADistance(tD);
                        } else {
                            rightDiagonalMoveAtPower(Range.clip(errorY * 0.015, -0.35, 0.35));
                        }
                        vuforiaMissCount = 0;
                    }
                }
                break;
            case 19:
                // turn 45 degree
                if (0 == navigation.turnByEncoderOpenLoop(glyTurnPower, -45,
                        robot.axleDistance, leftMotors, rightMotors)) {
                    getWheelLandmarks();
                    robot.glyphWheelLoad();
                    navigation.resetTurn(leftMotors, rightMotors);
                    state = 20;
                }
                break;
            case 20:
                // turn to 90 degree
                if (0 == navigation.turnByGyroCloseLoop(0.0,
                        (double) robot.imu.getAngularOrientation().firstAngle,
                        fGlyphTurnAngle,leftMotors,rightMotors)) {
                    turnAtPower(0.0);
                     getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                    state = 21;
                }
                break;
            case 21:
                // side move to the correct column
                vuforia.relicTrackables.deactivate();

                int  deltaDistance =  deliverDis[getNextDeliverIndex(-1)]*2;
//                if (vuforia.vumarkImage == "right") {
//                    getWheelLandmarks();
//                    timeStamp = System.currentTimeMillis();
//                    vuforiaMissCount = 0;
//                    vuforiaHitCount = 0;
//                    robot.retractJewelArm();
//                    robot.retractGlyphBlocker();
//                    navigation.resetTurn(leftMotors, rightMotors);
//                    state = 21;
//                } else if (vuforia.vumarkImage == "center") {
//                    deltaDistance = rightColumnDistance - centerColumnDistance;
//                } else {
//                    deltaDistance = rightColumnDistance - leftColumnDistance;
//                }

                if (0 == sideMoveByDistance(-sideMovePower/2, deltaDistance)) {
                    wheelDistanceLandMark = getWheelOdometer();
                    getWheelLandmarks();
                    timeStamp = System.currentTimeMillis();
                    vuforiaMissCount = 0;
                    vuforiaHitCount =0;
                    robot.retractJewelArm();
                    robot.retractGlyphBlocker();
                    navigation.resetTurn(leftMotors, rightMotors);
                    deliverIndex = Math.abs((deliverIndex-1))%deliverDone.length; // move on to the next column
                    state = 22;
                }
                break;
            case 22:
                if (backupDistance > 0) {
                    // continue moving straight
                    if (robot.backDistanceSensor.getDistance(DistanceUnit.INCH) < 12
                    || 0 == moveByDistance(move2GlyphBoxPower, backupDistance)) {
                        moveAtPower(0.0);
                        navigation.resetTurn(leftMotors, rightMotors);
                        getWheelLandmarks();
                        timeStamp = System.currentTimeMillis();
                        robot.levelGlyph();
                        robot.glyphWheelLoad();
                        state = 23;
                    }
                } else {
                    if (robot.backDistanceSensor.getDistance(DistanceUnit.INCH) < 12
                    || 0 == moveByDistance(-move2GlyphBoxPower, backupDistance)) {
                        moveAtPower(0.0);
                        navigation.resetTurn(leftMotors, rightMotors);
                        getWheelLandmarks();
                        timeStamp = System.currentTimeMillis();
                        robot.levelGlyph();
                        robot.glyphWheelLoad();
                        state = 23;
                    }
                }
                break;
            case 23:
                // release glyph
                robot.dumpGlyph();

                if (System.currentTimeMillis() - timeStamp > 1500) {
                    getWheelLandmarks();
                    state = 24;
                }
                break;
            case 24:
                // push
                if (0 == moveByDistance(move2GlyphBoxPower*0.6, 700)) {
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    state = 25;
                }
                break;
            case 25:
                // back up
                if (0 == moveByDistance(-move2GlyphBoxPower, 300)) {
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    state = 26;
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

        telemetry.addData("teamColor", teamColor);
        telemetry.addData("vumark", vuforia.vumarkImage);
        telemetry.addData("state", state);
        telemetry.addData("backupDistance", backupDistance);
        telemetry.addData("devilver index", deliverIndex);
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
        robot.motorRightBackWheel.setPower(0.0);
        robot.motorLeftBackWheel.setPower(p);
        robot.motorLeftFrontWheel.setPower(0.0);
    }

    public void leftDiagonalMoveAtPower(double p) {
        robot.motorLeftFrontWheel.setPower(p);
        robot.motorLeftBackWheel.setPower(0.0);
        robot.motorRightBackWheel.setPower(p);
        robot.motorRightFrontWheel.setPower(0.0);
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

    public void stopGlyphWheels(){
        robot.leftLiftWheel.setPower(0.0);
        robot.rightLiftWheel.setPower(0.0);
    }

}
