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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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
@Autonomous(name = "Rian_PlanA_Blue", group = "Rian")

public class AutoRianPlanABlue extends OpMode {

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

    protected ColorSensor jewelSensor = null;
    protected DistanceSensor jewelSensorDistance= null;

    protected HardwareRian robot= null;

    protected Servo jewelArm= null;
    protected Servo jewelHitter= null;

    protected int state;
    protected long timeStamp;

    protected float fGlyphTurnAngle = 90;

    protected JewelKicker jewelKicker= null;

    @Override
    public void init() {
        robot = new HardwareRian();
        robot.init(hardwareMap);

        jewelArm = robot.jewelArm;
        jewelHitter = robot.jewelHitter;

        jewelSensor = robot.jewelSensor;
        jewelSensorDistance = robot.jewelSensorDistance;

        jewelKicker = new JewelKicker(jewelSensor,jewelSensorDistance,jewelArm,jewelHitter,telemetry);
        jewelKicker.init();

        telemetry.addData("jewelArm", jewelArm.getPosition());
        telemetry.addData("jewelHitter", jewelHitter.getPosition());
        telemetry.update();
    }


    @Override
    public void start() {
        state = 0;
        timeStamp = System.currentTimeMillis();
        jewelKicker.start();
    }

    @Override
    public void loop() {
        switch (state) {
            case 0:
                // jewel handling
                state = jewelKicker.loop(0,1);
            case 1:
                 //detect crypto

                break;
            case 2:
                // back up, stop at the correct glyph row (left, center, right)
                break;

            case 3:
                // get heading
                Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                robot.navigation.heading = angles.firstAngle;
                //gravity  = robot.imu.getGravity();

                // turn right 90 degree
                float turnPower = robot.navigation.getMaintainHeadingPower(fGlyphTurnAngle);
                if (Math.abs(turnPower) < 0.01) {
                    state = 4;
                }

                break;
            case 4:
                // move straight
                break;
            case 5:
                // release the glyph
                break;
            case 6:
                // stop
                break;
            default:
        }

        telemetry.update();
    }
}
