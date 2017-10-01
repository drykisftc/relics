package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This is NOT an OpMode
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot, using Matrix Hardware.
 * See PushbotTeleopTank_Iterative for a usage examples.
 *
 * This is coded as an Extension of HardwarePushbot to illustrate that the only additional
 * action REQUIRED for a MATRIX controller is enabling the Servos.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Matrix Controller has been assigned the name:  "matrix controller"
 *
 * Motor channel:  Left  drive motor:        "left motor"
 * Motor channel:  Right drive motor:        "right motor"
 * Motor channel:  Manipulator drive motor:  "arm motor"
 * Servo channel:  Servo to open left claw:  "left claw"
 * Servo channel:  Servo to open right claw: "right claw"
 *
 * In addition, the Matrix Controller has been assigned the name:  "matrix controller"
 */
public class HardwareNathen extends HardwareBase
{
    // DC Motors
    public DcMotor motorLeftWheel = null;
    public DcMotor motorRightWheel =null;

    //Servos
    public Servo jewelArm = null;
    public Servo jewelHitter = null;

    //Sensors
    public ColorSensor jewelSensor;
    public DistanceSensor jewelSensorDistance;

    /* Constructor */
    public HardwareNathen(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        super.init(ahwMap);

        motorLeftWheel = hwMap.dcMotor.get("leftWheel");
        motorRightWheel = hwMap.dcMotor.get("rightWheel");
        motorLeftWheel.setDirection(DcMotor.Direction.FORWARD);  // 40 to 1 andymark motor
        motorRightWheel.setDirection(DcMotor.Direction.REVERSE); // 40 to 1 andymark motor
        motorLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        jewelArm = hwMap.servo.get("jewelArm");
        jewelHitter = hwMap.servo.get("jewelHitter");

        //jewelSensor = hwMap.colorSensor.get("jewelSensor");
        jewelSensor = hwMap.get(ColorSensor.class, "jewelSensor");
        jewelSensorDistance = hwMap.get(DistanceSensor.class, "jewelSensor");
    }

    public void stop() {
        motorLeftWheel.setPower(0.0);
        motorRightWheel.setPower(0.0);


        motorLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}