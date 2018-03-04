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
@Autonomous(name = "Harvester_PlanA_Red_VF", group = "A_Harvester")

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
    int vuforiaTargetDistance = leftVuDis;
    int cryptoBoxTargetDistance = 1000; // 40 inches?


    protected double distanceToVumark;

    public AutoHarvesterPlanARedVF() {
        teamColor = "red";

        glyphLiftPosition = 1500;
        centerGlyphAngleOffset = 0;
        vuforiaDetectingPower = -0.2;
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
                if (0 == moveByDistance(-move2GlyphBoxPower*2, 400)) {

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
                if (0 == moveByDistance(move2GlyphBoxPower*2.0, 400)) {
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
                    state = 8;

                }
                break;
            case 8:
                // correct angle just in case it got knocked out the cource
                if (0 == navigation.turnByGyroCloseLoop(0.0,
                        (double) robot.imu.getAngularOrientation().firstAngle,
                        -45,leftMotors,rightMotors)) {
                    getWheelLandmarks();
                    robot.glyphWheelLoad();
                    navigation.resetTurn(leftMotors, rightMotors);
                    state = 9;
                }
                break;
            case 9:
                // move to center diagonally
                if (0 == leftDiagonalMoveByDistance(move2CenterPower, 4000)) {
                    moveAtPower(0.0);
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    timeStamp = System.currentTimeMillis();
                    robot.loadGlyph();
                    robot.glyphWheelLoad();
                    state = 10;
                }
                telemetry.addData("turning = ", true);
                break;
            case 10:
                //wiggle
                navigation.turnByGyroCloseLoop(0.0,
                        (double) robot.imu.getAngularOrientation().firstAngle,
                        -45+rand.nextInt(20)-10,
                        leftMotors, rightMotors);
                // move to center slower to collect glyph
                if (0 == moveByDistance(collectingGlyphPower, 1000)) {
                    moveAtPower(0.0);
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    timeStamp = System.currentTimeMillis();
                    state = 11;
                }
                break;
            case 11:
                // wiggle to improve intake
                navigation.turnByGyroCloseLoop(0.0,
                        (double) robot.imu.getAngularOrientation().firstAngle,
                        -45+rand.nextInt(30)-15,
                        leftMotors, rightMotors);

                // collect glyph
                if (System.currentTimeMillis() - timeStamp > 2500) {
                    state = 12;
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                }
                break;
            case 12:
                // back up from glyph
                if (0 == moveByDistance(-collectingGlyphPower, 2500)) {
                    moveAtPower(0.0);
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    timeStamp = System.currentTimeMillis();
                    state = 13;
                }
                break;
            case 13:
                vuforia.identifyGlyphCrypto();
                // turn 45 degrees
                if (0 == navigation.turnByGyroCloseLoop(0.0,
                        (double) robot.imu.getAngularOrientation().firstAngle,
                        - 45,leftMotors,rightMotors)) {
                    moveAtPower(0.0);
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    timeStamp = System.currentTimeMillis();
                    state = 14;
                }
            case 14:
                // look for vuforia
                if ("unknown" == vuforia.vumarkImage.toLowerCase()) {
                    vuforia.identifyGlyphCrypto();
                    // move back
                    if (0 == moveByDistance(vuforiaDetectingPower, 1700)) {
                        moveAtPower(0.0);
                        navigation.resetTurn(leftMotors, rightMotors);
                        getWheelLandmarks();
                        timeStamp = System.currentTimeMillis();
                        state = 25;
                    }
                } else {
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    timeStamp = System.currentTimeMillis();
                    robot.loadGlyph();
                    robot.glyphWheelLoad();
                    state = 15;
                }
                break;
            case 15:
                // calculate location
                OpenGLMatrix pose = vuforia.getGlyphCryptoPosition();

                // if too many errors, move on
                if (vuforiaMissCount > 18) {
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    state = 25;
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
                    telemetry.addData("vuforia distance error", errorZ);
                    if ( Math.abs(errorZ) > 200 || Math.abs(rot.secondAngle) > 30 || Math.abs(errorY) > 400) {
                        vuforiaMissCount++;
                    } else {
                        if (Math.abs(errorZ) < 15 && Math.abs(errorY) < 200 ) {
                            vuforiaHitCount ++;
                        } else {
                            vuforiaHitCount = 0;
                        }

                        if ( vuforiaHitCount > 10) {
                            state = 15;
                            getWheelLandmarks();
                            timeStamp = System.currentTimeMillis();
                            wheelDistanceLandMark = getWheelOdometer();
                            sideMoveAtPower(0);
                            // set glyph box distance. image to glyph box distance is 34.5 inch , camera to flipper distance is 4
                            cryptoBoxDistance = robot.imageDistance2GlyphBoxADistance(tG, columnDistance);
                            backupDistance = robot.robotToCryptoBoxADistance(tD);
                        } else {
                            //leftDiagonalMoveAtPower(Range.clip(errorZ * -0.0015, -0.35, 0.35));
                            //sideMoveAtPower(Range.clip(errorZ * 0.015, -0.2, 0.2));
                            //sideMoveAtPower(Range.clip(navigation.getMaintainDistancePower(targetVuDis,tZ), -0.65, 0.65));
                        }
                        int deltaDistance = 0;
                        if (vuforia.vumarkImage == "left") {
                            //state = 19;
                            getWheelLandmarks();
                            wheelDistanceLandMark = getWheelOdometer();
                            navigation.resetTurn(leftMotors, rightMotors);
                        } else if (vuforia.vumarkImage == "center") {
                            deltaDistance = leftColumnDistance - centerColumnDistance;
                        } else {
                            deltaDistance = leftColumnDistance - rightColumnDistance;
                        }
                        vuforiaMissCount = 0;
                    }
                }
                break;
            case 16:
                // turn 45 degrees back
                if (0 == navigation.turnByGyroCloseLoop(0.0, (double) robot.imu.getAngularOrientation().firstAngle,fGlyphTurnAngle,leftMotors,rightMotors)) {
                    getWheelLandmarks();
                    robot.glyphWheelLoad();
                    navigation.resetTurn(leftMotors, rightMotors);
                    state = 17;
                }
                break;
            case 17:
                // side move to corresponding position
                if (cryptoBoxDistance >= 0) {
                    if (0 == sideMoveByDistance(-sideMovePower/2, cryptoBoxDistance)) {
                        wheelDistanceLandMark = getWheelOdometer();
                        getWheelLandmarks();
                        timeStamp = System.currentTimeMillis();
                        vuforiaMissCount = 0;
                        vuforiaHitCount = 0;
                        robot.retractJewelArm();
                        state = 18;
                    }
                } else {
                    if (0 == sideMoveByDistance(sideMovePower/2, cryptoBoxDistance)) {
                        wheelDistanceLandMark = getWheelOdometer();
                        getWheelLandmarks();
                        timeStamp = System.currentTimeMillis();
                        vuforiaMissCount = 0;
                        vuforiaHitCount =0;
                        robot.retractJewelArm();
                        state = 18;
                    }
                }
                break;
            case 18:
                // continue moving strait
                if (0 == leftDiagonalMoveByDistance(-move2CenterPower, backupDistance)) {
                    moveAtPower(0.0);
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    timeStamp = System.currentTimeMillis();
                    robot.loadGlyph();
                    robot.glyphWheelLoad();
                    state = 19;
                }
                break;
                // calculate location
                /*// move to center fast
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

                break;*/
            case 25:
                // turn back
                if (0 == navigation.turnByGyroCloseLoop(0.0, (double) robot.imu.getAngularOrientation().firstAngle, fGlyphTurnAngle, leftMotors, rightMotors)) {
                    moveAtPower(0.0);
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    timeStamp = System.currentTimeMillis();
                    state = 26;
                }
                break;
            case 26:
                // back up
                if (0 == moveByDistance(-0.5, cryptoBoxDistance + 1000)) {
                    moveAtPower(0.0);
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    timeStamp = System.currentTimeMillis();
                    state = 19;
                }
            case 19:
                // release glyph
                robot.dumpGlyph();

                if (System.currentTimeMillis() - timeStamp > 1000) {
                    getWheelLandmarks();
                    state = 20;
                }
                break;
            case 20:
                // backup
                if (0 == moveByDistance(-move2GlyphBoxPower , 400)) {
                    moveAtPower(0.0);
                    state = 21;
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

    public void stopGlyphWheels(){
        robot.leftLiftWheel.setPower(0.0);
        robot.rightLiftWheel.setPower(0.0);
    }

}
