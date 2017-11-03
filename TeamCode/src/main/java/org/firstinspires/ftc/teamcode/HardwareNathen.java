package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.DelayQueue;

/**

 24 inches have 1939 encoder counts that is 80.79encoder counts per inch
 Nathan has wheel base of 15.25 inches.
 */
public class HardwareNathen extends HardwareBase
{
    // DC Motors
    public DcMotor motorLeftWheel = null;
    public DcMotor motorRightWheel = null;
    public DcMotor liftMotor = null;

    //Servos
    public Servo leftHand = null;
    public Servo rightHand = null;

    public ColorSensor jewelSensor = null;
    public DistanceSensor jewelSensorDistance = null;

    public Servo jewelArm = null;
    public Servo jewelHitter = null;

    ModernRoboticsI2cGyro gyro = null;

    //Sensors
    //public ColorSensor jewelSensor;
    //public DistanceSensor jewelSensorDistance;

    // limits
    int liftHeightLimit = 4000;
    int liftMotorPosition = 0;
    double liftMotorHolderPower = 0.3;

    double leftHandOpenPosition = 1.0;
    double leftHandClosePosition = 0.4;
    double leftHandChargePosition = 0.8;
    double rightHandOpenPosition = 0.0;
    double rightHandClosePosition = 0.6;
    double rightHandChargePosition = 0.2;

    protected float axleDistance = 1232; //1248f;//1254

    /* Constructor */
    public HardwareNathen(){

    }

    /* Initialize standard Hardware interfaces */
    @Override
    public void init(HardwareMap ahwMap) {

        super.init(ahwMap);

        motorLeftWheel = hwMap.dcMotor.get("leftWheel");
        motorRightWheel = hwMap.dcMotor.get("rightWheel");
        motorLeftWheel.setDirection(DcMotor.Direction.REVERSE);  // 40 to 1 andymark motor
        motorRightWheel.setDirection(DcMotor.Direction.FORWARD); // 40 to 1 andymark motor
        motorLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotor = hwMap.dcMotor.get("liftMotor");
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftHand = hwMap.servo.get("leftHand");
        rightHand = hwMap.servo.get("rightHand");

        jewelHitter = hwMap.servo.get("jewelHitter");
        jewelArm = hwMap.servo.get("jewelArm");

        jewelSensor = hwMap.colorSensor.get("jewelSensor");
        jewelSensor.enableLed(true);

        gyro = (ModernRoboticsI2cGyro)hwMap.gyroSensor.get("gyro");

    }

    @Override
    public void start() {
        // wheels
        motorLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftWheel.setPower(0.0);
        motorRightWheel.setPower(0.0);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setPower(0.0);

        // init positions
        jewelArm.setPosition(0.8);
        jewelHitter.setPosition(0.5);
        leftHand.setPosition(leftHandOpenPosition);
        rightHand.setPosition(rightHandOpenPosition);
        jewelSensor.enableLed(true);
    }

    @Override
    public void stop() {
        motorLeftWheel.setPower(0.0);
        motorRightWheel.setPower(0.0);
        liftMotor.setPower(0.0);

        motorLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public float getGyroHeading () {
        return -gyro.getHeading();
    }
}