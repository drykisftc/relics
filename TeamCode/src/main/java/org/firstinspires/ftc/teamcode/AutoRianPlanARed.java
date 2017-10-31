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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/*
 * This is an example LinearOpMode that shows how to use
 * the REV Robotics Color-Distance Sensor.
 *
 * It assumes the sensor is configured with the name "sensor_color_distance".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 */
@Autonomous(name = "Rian_PlanA_Red", group = "Rian")

public class AutoRianPlanARed extends OpMode {

    /**
     * Note that the REV Robotics Color-Distance incorporates two sensors into one device.
     * It has a light/distance (range) sensor.  It also has an RGB color sensor.
     * The light/distance sensor saturates at around 2" (5cm).  This means that targets that are 2"
     * or closer will display the same value for distance/light detected.
     *
     * Although you configure a single REV Robotics Color-Distance sensor in your configuration file,
     * you can treat the sensor as two separate sensors that share the same name in your op mode.
     *
     * In this example, we represent the detected color by a hue, saturation, and value color
     * model (see https://en.wikipedia.org/wiki/HSL_and_HSV).  We change the background
     * color of the screen to match the detected color.
     *
     * In this example, we  also use the distance sensor to display the distance
     * to the target object.  Note that the distance sensor saturates at around 2" (5 cm).
     *
     */

    VuforiaLocalizer vuforia;

    protected int cameraMonitorViewId;
    protected VuforiaLocalizer.Parameters parameters;
    protected VuforiaTrackables relicTrackables;
    protected VuforiaTrackable relicTemplate;
    protected RelicRecoveryVuMark vuMark;
    protected String vumarkImage = "unknown";

    protected ColorSensor jewelSensor = null;
    protected DistanceSensor jewelSensorDistance= null;
    protected BNO055IMU imuSensor = null;

    protected HardwareRian robot= null;

    protected Servo jewelArm= null;
    protected Servo jewelHitter= null;

    protected float axleDiagonalDistance;

    protected int state;
    protected long timeStamp;
    protected float wheelDistanceAverageStamp;
    protected float wheelDistanceAverage;
    protected int columnDistance;
    protected int leftBackStamp;
    protected int leftFrontStamp;
    protected int rightBackStamp;
    protected int rightFrontStamp;

    protected float fGlyphTurnAngle = 90;
    protected int cryptoBoxStopDistance = 20;
    protected double vuforiaDetectingSpeed = 0.2;
    protected double adjustingSpeed;
    protected int rightColumnDistance = 2600;
    protected int centerColumnDistance = 3350;
    protected int leftColumnDistance = 3950;
    protected int cryptoBoxDistance = 500;
    protected float axleDistance = 18.1f;

    protected JewelKicker jewelKicker= null;

    protected Navigation navigation =null;

    @Override
    public void init() {
        robot = new HardwareRian();
        robot.init(hardwareMap);

        jewelArm = robot.jewelArm;
        jewelHitter = robot.jewelHitter;

        jewelSensor = robot.jewelSensor;
        jewelSensorDistance = robot.jewelSensorDistance;
        imuSensor = robot.imu;

        jewelKicker = new JewelKicker(jewelSensor,jewelArm,jewelHitter,telemetry);
        jewelKicker.init();

        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AVw9AA7/////AAAAGR2dOk5hfEdLl+V9Doao7C5Xp0Wvb2cien7ybTAhAyUTB2iZRO/CMyxlXakNnP3+HqLEMe7nzV+fllHLVQLuSwWmLdDErkjexTZKcgCGQUIZ+Ts6O2m7l+zwVVBH5V5Ah5SJP3jd/P6lvuKJY+DUY0pThAitsP59uD6wkcukMQQXNN+xBPzEBEx/0kt7hS5GJ+qCYDLD1qgCO5KrDuWzYtWjZi3LaGHsO9msvrGiCXYaP9PDRX9ZoWB1tJiHky5HyG/p+ndycmiK6sY9lRymaaJ5fX556ZUKtQX2dOAF7tHgVqsPOhqCV3E3qN6kXnwEqy9KgZ1QQjKJnCR5eLRXmSOqAbKi8ArzrRc3737EpSzK";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        navigation = new Navigation(telemetry);

        telemetry.addData("jewelArm", jewelArm.getPosition());
        telemetry.addData("jewelHitter", jewelHitter.getPosition());
        telemetry.update();
    }


    @Override
    public void start() {
        robot.start();
        state = 0;
        timeStamp = System.currentTimeMillis();
        vumarkImage = "Unknown";
        jewelKicker.start();
        relicTrackables.activate();
    }

    @Override
    public void loop() {
        switch (state) {
            case 0:

                // jewel handling
                state = jewelKicker.loop(0, 1, "red");

                vuMark = RelicRecoveryVuMark.from(relicTemplate);

                if (vuMark == RelicRecoveryVuMark.LEFT) {
                    vumarkImage = "left";
                } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    vumarkImage = "right";
                } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                    vumarkImage = "center";
                }

                break;
            case 1:

                //read vumark
                vuMark = RelicRecoveryVuMark.from(relicTemplate);

                if (vuMark == RelicRecoveryVuMark.LEFT) {
                    vumarkImage = "left";
                } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    vumarkImage = "right";
                } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                    vumarkImage = "center";
                }
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                //move forward with encoder
                wheelDistanceAverage = (robot.motorLeftBackWheel.getCurrentPosition() +
                                            robot.motorLeftFrontWheel.getCurrentPosition() +
                                            robot.motorRightBackWheel.getCurrentPosition() +
                                            robot.motorRightFrontWheel.getCurrentPosition())/4;

                if (vumarkImage == "left") {
                    columnDistance = leftColumnDistance;
                } else if (vumarkImage == "center") {
                    columnDistance = centerColumnDistance;
                } else if (vumarkImage == "right") {
                    columnDistance = rightColumnDistance;
                } else {
                    columnDistance = rightColumnDistance;
                }

                if (wheelDistanceAverage < columnDistance) {

                    moveAtSpeed(vuforiaDetectingSpeed);

                } else {

                    moveAtSpeed(0.0);
                    leftBackStamp = robot.motorLeftBackWheel.getCurrentPosition();
                    leftFrontStamp = robot.motorLeftFrontWheel.getCurrentPosition();
                    rightBackStamp = robot.motorRightBackWheel.getCurrentPosition();
                    rightFrontStamp = robot.motorRightFrontWheel.getCurrentPosition();
                    state = 4;

                }

                //move forward
                /*if (!(robot.jewelSensorDistance.getDistance(DistanceUnit.CM) < cryptoBoxStopDistance)) {

                    moveAtSpeed(vuforiaDetectingSpeed);

                } else {

                    moveAtSpeed(0);
                    state = 2;
                }*/

                break;
            case 4:
                /*// get heading
                Orientation angles = imuSensor.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                robot.navigation.heading = angles.firstAngle;
                //gravity  = robot.imu.getGravity();

                // turn right 90 degree
                float turnPower = robot.navigation.getMaintainHeadingPower(fGlyphTurnAngle);
                if (Math.abs(turnPower) < 0.01) {
                    state = 5;
                }

                robot.motorLeftFrontWheel.setPower(turnPower);
                robot.motorLeftBackWheel.setPower(turnPower);
                robot.motorRightFrontWheel.setPower(-turnPower);
                robot.motorRightBackWheel.setPower(-turnPower);*/

                if ((robot.motorLeftFrontWheel.getCurrentPosition() - leftFrontStamp + robot.motorLeftBackWheel.getCurrentPosition() - leftBackStamp > 3575) && (robot.motorRightBackWheel.getCurrentPosition() - rightBackStamp + robot.motorRightFrontWheel.getCurrentPosition() - rightFrontStamp < -3575)) {

                    turnAtSpeed(0.0);
                    wheelDistanceAverageStamp = (robot.motorLeftBackWheel.getCurrentPosition() +
                            robot.motorLeftFrontWheel.getCurrentPosition() +
                            robot.motorRightBackWheel.getCurrentPosition() +
                            robot.motorRightFrontWheel.getCurrentPosition())/4;
                    telemetry.addData("left", robot.motorLeftFrontWheel.getCurrentPosition() - leftFrontStamp + robot.motorLeftBackWheel.getCurrentPosition() - leftBackStamp);
                    telemetry.addData("left", robot.motorRightBackWheel.getCurrentPosition() - rightBackStamp + robot.motorRightFrontWheel.getCurrentPosition() - rightFrontStamp);
                    state = 5;

                } else {

                    turnAtSpeed(0.5);
                }

                break;
            case 5:
                // move straight

                wheelDistanceAverage = (robot.motorLeftBackWheel.getCurrentPosition() +
                        robot.motorLeftFrontWheel.getCurrentPosition() +
                        robot.motorRightBackWheel.getCurrentPosition() +
                        robot.motorRightFrontWheel.getCurrentPosition())/4;

                if (wheelDistanceAverage - wheelDistanceAverageStamp < cryptoBoxDistance) {

                    moveAtSpeed(0.5);

                } else {

                    moveAtSpeed(0.0);
                    timeStamp = System.currentTimeMillis();
                    state = 6;

                }

                break;
            case 6:
                // release the glyph

                time = System.currentTimeMillis();

                if (time - timeStamp < 2000) {

                    robot.leftLiftWheel1.setPower(1.0);
                    robot.leftLiftWheel2.setPower(1.0);
                    robot.leftLiftWheel3.setPower(1.0);
                    robot.rightLiftWheel1.setPower(-1.0);
                    robot.rightLiftWheel2.setPower(-1.0);
                    robot.rightLiftWheel3.setPower(-1.0);

                } else {

                    robot.leftLiftWheel1.setPower(0.0);
                    robot.leftLiftWheel2.setPower(0.0);
                    robot.leftLiftWheel3.setPower(0.0);
                    robot.rightLiftWheel1.setPower(0.0);
                    robot.rightLiftWheel2.setPower(0.0);
                    robot.rightLiftWheel3.setPower(0.0);

                }

                break;
            case 7:

                robot.stop();

                // stop
                break;
            default:
        }

        telemetry.addData("state", state);
        telemetry.addData("vumark", vumarkImage);
        telemetry.update();
    }

    public float getTurnDistance (float angle, float axleDistance){
        // if the res is positive, set left Motor move forward (right turn),
        // other wise (left turn)
        return (float)(axleDistance * angle * 3.14) / 360;
    }

    public void moveAtSpeed (double speed) {

        robot.motorLeftBackWheel.setPower(speed);
        robot.motorLeftFrontWheel.setPower(speed);
        robot.motorRightBackWheel.setPower(speed);
        robot.motorRightFrontWheel.setPower(speed);

    }

    public void turnAtSpeed (double speed) {

        robot.motorLeftBackWheel.setPower(speed);
        robot.motorLeftFrontWheel.setPower(speed);
        robot.motorRightBackWheel.setPower(-speed);
        robot.motorRightFrontWheel.setPower(-speed);

    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

}
