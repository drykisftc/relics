package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

/**
 24 inches have 1939 encoder counts that is 80.79encoder counts per inch
 Nathan has wheel base of 14.4 inches.
 */

public class HardwareHarvester extends HardwareBase
{
    // DC Motors
    public DcMotor motorLeftBackWheel = null;
    public DcMotor motorRightBackWheel = null;
    public DcMotor motorLeftFrontWheel = null;
    public DcMotor motorRightFrontWheel = null;

    public DcMotor liftMotor = null;

    public DcMotor relicMotor = null;

    public DcMotor leftLiftWheel = null;
    public DcMotor rightLiftWheel = null;

    //servos
    public Servo jewelArm = null;
    public Servo jewelHitter = null;

    public Servo leftFlipper = null;
    public Servo rightFlipper = null;

    public Servo relicClaw = null;
    public Servo relicFlipper = null; // winch servo

    public Servo glyphBlocker = null;

    //public Servo RKArm = null; // roboknight arm for auto

    // Orientation sensor
    BNO055IMU imu = null;
    Orientation angles = null;

    //sensors
    public ColorSensor jewelSensor = null; // jewel arm
    public DistanceSensor jewelSensorDistance = null; //jewel arm

    public DistanceSensor glyphDistance = null;

    //public DistanceSensor RKSensor = null; // roboknight arm for auto

    public ModernRoboticsI2cRangeSensor backDistanceSensor = null; // sensor in the back

    protected float axleDistance = 2200; //80.79 * 14;

    double leftFlipperLoadPosition = 0.19;    // a
    double leftFlipperLevelPosition = 0.28;   // b
    double leftFlipperDumpPosition = 0.70;    // y

    double rightFlipperLoadPosition = 0.91;   // a
    double rightFlipperLevelPosition = 0.82;  // b
    double rightFlipperDumpPosition = 0.39;   // y

    double defaultGlyphWheelPower = 1.0;
    double defaultGlyphLiftPower = 0.5;

    double relicClawOpenPosition = 0.0;
    double relicClawClosePosition = 0.55;

    double RKArmRetractPosition = 0.6;
    double RKArmExtendPosition = 0.2;

    double encoderStepsPerInch = 89;

    /* Constructor */
    public HardwareHarvester(){

    }

    /* Initialize standard Hardware interfaces */
    @Override
    public void init(HardwareMap ahwMap) {

        super.init(ahwMap);

        motorLeftBackWheel = hwMap.dcMotor.get("leftBackWheel");
        motorRightBackWheel = hwMap.dcMotor.get("rightBackWheel");
        motorLeftBackWheel.setDirection(DcMotor.Direction.REVERSE);  // 40 to 1 andymark motor
        motorRightBackWheel.setDirection(DcMotor.Direction.FORWARD); // 40 to 1 andymark motor
        motorLeftBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLeftFrontWheel = hwMap.dcMotor.get("leftFrontWheel");
        motorRightFrontWheel = hwMap.dcMotor.get("rightFrontWheel");
        motorLeftFrontWheel.setDirection(DcMotor.Direction.REVERSE);  // 40 to 1 andymark motor
        motorRightFrontWheel.setDirection(DcMotor.Direction.FORWARD); // 40 to 1 andymark motor
        motorLeftFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotor = hwMap.dcMotor.get("liftMotor");
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        relicMotor = hwMap.dcMotor.get("relicMotor");
        relicMotor.setDirection(DcMotor.Direction.REVERSE);
        relicMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        relicMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        jewelHitter = hwMap.servo.get("jewelHitter");
        jewelArm = hwMap.servo.get("jewelArm");

        relicFlipper = hwMap.servo.get("relicFlipper");
        relicClaw = hwMap.servo.get("relicClaw");

//        RKArm = hwMap.servo.get("RKArm");

        jewelSensor = hwMap.get(ColorSensor.class, "jewelSensor");
        jewelSensorDistance = hwMap.get(DistanceSensor.class, "jewelSensor");

        glyphDistance = hwMap.get(DistanceSensor.class, "glyphDistance");

        //RKSensor = hwMap.get(DistanceSensor.class, "RKSensor");

        backDistanceSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "backDistanceSensor");

        leftLiftWheel = hwMap.dcMotor.get("leftLiftWheel");
        leftLiftWheel.setDirection(DcMotor.Direction.FORWARD);
        leftLiftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLiftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightLiftWheel = hwMap.dcMotor.get("rightLiftWheel");
        rightLiftWheel.setDirection(DcMotor.Direction.REVERSE);
        rightLiftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFlipper = hwMap.servo.get("leftFlipper");
        rightFlipper = hwMap.servo.get("rightFlipper");

        glyphBlocker = hwMap.servo.get("glyphBlocker");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    @Override
    public void start () {
        // wheels
        motorLeftBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftBackWheel.setPower(0.0);
        motorRightBackWheel.setPower(0.0);

        motorLeftFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftFrontWheel.setPower(0.0);
        motorRightFrontWheel.setPower(0.0);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setPower(0.0);

        relicMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        relicMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        relicMotor.setPower(0.0);

        leftLiftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLiftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLiftWheel.setPower(0.0);

        rightLiftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLiftWheel.setPower(0.0);
        //jewelArm.setPosition(0.8);
    }

    @Override
    public void stop() {

        motorLeftBackWheel.setPower(0.0);
        motorRightBackWheel.setPower(0.0);
        motorLeftFrontWheel.setPower(0.0);
        motorRightFrontWheel.setPower(0.0);

        liftMotor.setPower(0.0);

        relicMotor.setPower(0.0);

        leftLiftWheel.setPower(0.0);
        rightLiftWheel.setPower(0.0);

        motorLeftBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        relicMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftLiftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLiftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    void initAllDevices() {
        retractJewelArm();
        retractGlyphBlocker();
    }

    void retractJewelArm() {
        jewelArm.setPosition(0.85);
        jewelHitter.setPosition(0.0);
    }

    void retractGlyphBlocker () {
        glyphBlocker.setPosition(1.0);
    }

    void extendGlyphBlocker() { glyphBlocker.setPosition(0.6); }

    void loadGlyph() {
        leftFlipper.setPosition(leftFlipperLoadPosition);
        rightFlipper.setPosition(rightFlipperLoadPosition);
    }

    void dumpGlyph() {
        leftFlipper.setPosition(leftFlipperDumpPosition);
        rightFlipper.setPosition(rightFlipperDumpPosition);
    }

    void levelGlyph() {
        leftFlipper.setPosition(leftFlipperLevelPosition);
        rightFlipper.setPosition(rightFlipperLevelPosition);
    }
    void glyphWheelLoad(){

        leftLiftWheel.setPower(defaultGlyphWheelPower);
        rightLiftWheel.setPower(defaultGlyphWheelPower);
    }

    void glyphWheelUnload() {
        leftLiftWheel.setPower(-defaultGlyphWheelPower);
        rightLiftWheel.setPower(-defaultGlyphWheelPower);
    }

//    void extendRKArm() {
//        RKArm.setPosition(RKArmExtendPosition);
//    }
//
//    void retractRKArm() {
//        RKArm.setPosition(RKArmRetractPosition);
//    }

    public void stopGlyphWheels() {
        leftLiftWheel.setPower(0.0);
        rightLiftWheel.setPower(0.0);
    }

    public boolean haveGlyph() {
        return (glyphDistance.getDistance(DistanceUnit.CM) < 8);
    }

    public static double getVuforiaLeftRightDistance(OpenGLMatrix pose ) {
        if (pose != null) {
            VectorF trans = pose.getTranslation();
            Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);

            // Extract the X, Y, and Z components of the offset of the target relative to the robot
            //double tX = trans.get(0);
            double tY = trans.get(1);
            double tZ = trans.get(2);
            double rDegree = rot.secondAngle;

            return tZ * Math.sin(rDegree) + tY * Math.cos(rDegree);
        } else {
            return 0.0;
        }
    }

    public static double getVuforiaFrontBackDistance ( OpenGLMatrix pose ) {
        if (pose != null) {
            VectorF trans = pose.getTranslation();
            Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);

            // Extract the X, Y, and Z components of the offset of the target relative to the robot
            // double tX = trans.get(0);
            double tY = trans.get(1);
            double tZ = trans.get(2);
            double rDegree = rot.secondAngle;

            return tZ * Math.cos(rDegree) - tY * Math.sin(rDegree);
        } else {
            return 0.0;
        }
    }

    public int CM2Encoder (double cm) {
        return (int)(cm / 2.54 * 89);
    }

    public static int imageDistance2GlyphBoxBDistance (double tG) {
        return (int)(((34.5/0.0393701 - Math.abs(tG)) * 0.0393701) * 89);
    }

//    public static int imageDistance2GlyphBoxADistance (double tG, int columnDistance) {
//        return (int)((((columnDistance/89)/0.0393701 - Math.abs(tG)) * 0.0393701 - 4.5) * 89);
//    }
//
//    public static int robotToCryptoBoxADistance (double tD) {
//        return (int)(((Math.abs(tD) - 7/0.0393701) * 0.0393701 - 11) * 89);
//    }

    public static int robotToCryptoBoxADistance (double tD) {
        return (int)((Math.abs(tD) * 0.0393701 - 12) * 89);
    }
}
