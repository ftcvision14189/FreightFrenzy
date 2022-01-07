package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

import static java.lang.Math.abs;

@Autonomous(name="Auto Functions", group="Auto")

public class FreightFrenzy_AutoFunctions extends LinearOpMode{
    BNO055IMU imu;

    Orientation angles;

    static DcMotor backLeftDrive = null;
    static DcMotor backRightDrive = null;
    static DcMotor frontLeftDrive = null;
    static DcMotor frontRightDrive = null;
    static DcMotor Feeder = null;
    static DcMotor Carousel = null;

    public void robotSetup(){
        DcMotor backLeftDrive = null;
        DcMotor backRightDrive = null;
        DcMotor frontLeftDrive = null;
        DcMotor frontRightDrive = null;
        DcMotor Feeder = null;
        DcMotor Carousel = null;

        backLeftDrive = hardwareMap.get(DcMotor.class, "dtBL");
        backRightDrive = hardwareMap.get(DcMotor.class, "dtBR");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "dtFL");
        frontRightDrive = hardwareMap.get(DcMotor.class, "dtFR");
        Feeder = hardwareMap.get(DcMotor.class, "Feeder");
        Carousel = hardwareMap.get(DcMotor.class, "Color Wheel");

        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /*
    Background Methods
    */
    // Inches to encoder counts
    static int inchesToEncoder(float inches){
        final double     COUNTS_PER_MOTOR_REV    = 384.5 ;    // eg: TETRIX Motor Encoder
        final double     DRIVE_GEAR_REDUCTION    = 2 ;     // This is < 1.0 if geared UP
        final double     WHEEL_DIAMETER_INCHES   = 6 ;     // For figuring circumference
        final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
        int counts = (int) (COUNTS_PER_INCH * inches);
        return(counts);
    }
    //resets the encoders
    static void resetMotors() {
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    //tells the motors to move and waits for the robot to stop moving
    public static void motorMove(){
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(frontLeftDrive.isBusy() || frontRightDrive.isBusy() || backLeftDrive.isBusy() || backRightDrive.isBusy()){
            // Do Nothing
        }
    }
    //runs the robot without encoders
    static void noEncoder(){
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    //sets the motor power
    static void motorPower(float power){
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);
    }
    static void motorPowerLeft(float power){
        frontLeftDrive.setPower(power);
        backLeftDrive.setPower(power);
    }
    static void motorPowerRight(float power){
        frontRightDrive.setPower(power);
        backRightDrive.setPower(power);
    }
    public double roll(){
        Quaternion quatangles = imu.getQuaternionOrientation();

        double w =quatangles.w;
        double x = quatangles.x;
        double y = quatangles.y;
        double z = quatangles.z;

        double yaw = Math.atan2(2*(w*z+x*y), 1 - (2*(y*y+z*z)))*180.0/Math.PI;

        return(yaw);
    }


    /*
    Movement
    */
    //drives forwards using encoders
    static void driveF(float power, int inches){ // Drive Forward
        resetMotors();
        motorPower(power);
        int distance = inchesToEncoder(inches);
        frontLeftDrive.setTargetPosition(distance);
        frontRightDrive.setTargetPosition(distance);
        backLeftDrive.setTargetPosition(distance);
        backRightDrive.setTargetPosition(distance);
        motorMove();
        noEncoder();
    }
    void driveRR(float power, float degrees){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addLine()
                .addData("Turn: ", new Func<String>() {
                    @Override public String value() {
                        return(toString().valueOf(angles.firstAngle));
                    }
                });
        float start=angles.firstAngle;
        do{
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.update();
            motorPowerRight(-power);
            motorPowerLeft(power);
        }while((angles.firstAngle-start)>=degrees && opModeIsActive());
        motorPower(0);
    }
    /*static void driveTimeTurnLeft(float power, float timeInSeconds) {
        motorPowerLeft(-power);
        motorPowerRight(power);
        sleep((int) (timeInSeconds * 1000));
        if (power > 0) {
            motorPowerLeft(-0.01f);
            motorPowerRight(0.01f);
        } else if (power < 0) {
            motorPowerLeft(0.01f);
            motorPowerRight(-0.01f);
        } else {
            // do nothing because power is already 0
        }
        sleep(500);
        motorPowerLeft(0);
        motorPowerRight(0);
    }
    static void driveTimeTurnRight(float power, float timeInSeconds) {
        motorPowerLeft(power);
        motorPowerRight(-power);
        sleep((int)(timeInSeconds * 1000));
        if (power > 0) {
            motorPowerLeft(0.01f);
            motorPowerRight(-0.01f);
        } else if (power < 0) {
            motorPowerLeft(-0.01f);
            motorPowerRight(0.01f);
        } else {
            // do nothing because power is already 0
        }
        sleep(500);
        motorPowerLeft(0);
        motorPowerRight(0);
    }
    static void driveTime(float power, float timeInSeconds) {
        FreightFrenzy_AutoFunctions.motorPower(power);
        sleep((int)(timeInSeconds * 1000));
        if (power > 0) {
            FreightFrenzy_AutoFunctions.motorPower(-0.01f);
        } else if (power < 0) {
            FreightFrenzy_AutoFunctions.motorPower(0.01f);
        } else {
            // do nothing because power is already 0
        }
        sleep(500);
        FreightFrenzy_AutoFunctions.motorPower(0);
    }*/

    @Override
    public void runOpMode() throws InterruptedException {


        backLeftDrive = hardwareMap.get(DcMotor.class, "dtBL");
        backRightDrive = hardwareMap.get(DcMotor.class, "dtBR");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "dtFL");
        frontRightDrive = hardwareMap.get(DcMotor.class, "dtFR");
        Feeder = hardwareMap.get(DcMotor.class, "Feeder");
        Carousel = hardwareMap.get(DcMotor.class, "Color Wheel");

        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);

        waitForStart();

        //FreightFrenzy_AutoFunctions.noEncoder();
        //FreightFrenzy_AutoFunctions.driveF(-.2f, -6);
        /*driveRR(0.25f, -90);
        sleep(4000);
        driveRR(0.25f, -160);
        sleep(4000);
        driveRR(0.25f, -120);
        sleep(2000);*/
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addLine()
                .addData("Turn: ", new Func<String>() {
                    @Override public String value() {
                        return(toString().valueOf(roll()));
                    }
                });
        while(opModeIsActive()){
            roll();
            telemetry.update();
        }
    }
}

