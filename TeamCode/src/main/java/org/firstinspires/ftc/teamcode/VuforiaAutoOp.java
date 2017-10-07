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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;

/*
 * This is an example LinearOpMode that shows how to use
 * the REV Robotics Color-Distance Sensor.
 *
 * It assumes the sensor is configured with the name "sensor_color_distance".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 */
@Autonomous(name = "VeforiaAutoOp", group = "Sensor")

public class VuforiaAutoOp extends OpMode {

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

    protected HardwareRian robot = new HardwareRian();

    //private ColorSensor jewelSensor;
    //private DistanceSensor jewelSensorDistance;

    //private Servo jewelArm;
    //private Servo jewelHitter;

    protected int cameraMonitorViewId;
    protected VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables relicTrackables;
    private VuforiaTrackable relicTemplate;

    private int state;
    //private long timeStamp;
    //private double jewelArmPosition;
    //private double jewelHitterPosition;

    //protected long jewelWaitTime = 1000;

    @Override
    public void init() {
        robot.init(hardwareMap);

//        jewelArm = robot.jewelArm;
//        jewelHitter = robot.jewelHitter;
//
//        jewelSensor = robot.jewelSensor;
//        jewelSensorDistance = robot.jewelSensorDistance;
//
//        jewelArm.setPosition(0.25);
//        jewelHitter.setPosition(0.48);

        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AVw9AA7/////AAAAGR2dOk5hfEdLl+V9Doao7C5Xp0Wvb2cien7ybTAhAyUTB2iZRO/CMyxlXakNnP3+HqLEMe7nzV+fllHLVQLuSwWmLdDErkjexTZKcgCGQUIZ+Ts6O2m7l+zwVVBH5V5Ah5SJP3jd/P6lvuKJY+DUY0pThAitsP59uD6wkcukMQQXNN+xBPzEBEx/0kt7hS5GJ+qCYDLD1qgCO5KrDuWzYtWjZi3LaGHsO9msvrGiCXYaP9PDRX9ZoWB1tJiHky5HyG/p+ndycmiK6sY9lRymaaJ5fX556ZUKtQX2dOAF7tHgVqsPOhqCV3E3qN6kXnwEqy9KgZ1QQjKJnCR5eLRXmSOqAbKi8ArzrRc3737EpSzK";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);

        relicTemplate.setName("relicVuMarkTemplate");


//        telemetry.addData("jewelArm", jewelArm.getPosition());
//        telemetry.addData("jewelHitter", jewelHitter.getPosition());
        telemetry.update();
    }


    @Override
    public void start() {
        state = 2;
        //timeStamp = System.currentTimeMillis();

        relicTrackables.activate();
    }

    @Override
    public void loop() {
        switch (state) {
            case 0:

//                jewelArm.setPosition(0.88);
//                jewelArmPosition = jewelArm.getPosition();
//                jewelHitterPosition = jewelHitter.getPosition();
//
//                if(jewelHitter.getPosition() > 0.40 && jewelHitter.getPosition() < 0.60) {
//                    if (System.currentTimeMillis() - timeStamp > jewelWaitTime && jewelSensor.blue() > jewelSensor.red() && jewelSensor.blue() > jewelSensor.green()) {
//                        jewelHitter.setPosition(1.00);
//                    } else if (System.currentTimeMillis() - timeStamp > jewelWaitTime) {
//                        jewelHitter.setPosition(0.00);
//                    }
//                }
//
//
//                if (System.currentTimeMillis() - timeStamp > 1500) {
//                    state++;
//                }
//
//                telemetry.addData("Distance (cm)", String.format(Locale.US, "%.02f", jewelSensorDistance.getDistance(DistanceUnit.CM)));
//                telemetry.addData("Red  ", jewelSensor.red());
//                telemetry.addData("Green", jewelSensor.green());
//                telemetry.addData("Blue ", jewelSensor.blue());
//                telemetry.addData("ArmPosition", jewelArm.getPosition());
//                telemetry.addData("HitterPosition", jewelHitter.getPosition());
//                telemetry.addData("State", state);

                break;
            case 1:

//                jewelArm.setPosition(0.75);
//
//                if(System.currentTimeMillis() - timeStamp > 1800) {
//                    jewelHitter.setPosition(0.48);
//                }

                break;
            case 2:

                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

                if (vuMark == RelicRecoveryVuMark.LEFT) {
                    telemetry.addData("vuMark", "left");
                } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    telemetry.addData("vuMark", "right");
                } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                    telemetry.addData("vuMark", "center");
                } else {
                    telemetry.addData("vuMark", "Unknown");
                }

            default:
        }
        /*int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);*/

        telemetry.update();

    }
}
