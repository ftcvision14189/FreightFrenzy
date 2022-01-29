package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

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

@Autonomous(name="RedWarehouse", group="Auto")

public class FreightFrenzy_RedWarehouse extends LinearOpMode{
    BNO055IMU imu;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor Feeder = null;
    private DcMotor Carousel = null;

    // Outside variables
    private float PI = 3.14159265359f;
    private float DIAMETER_IN_INCHES = 6.0f;
    private float WHEEL_CIRCUMFERENCE = 2.0f * PI * (DIAMETER_IN_INCHES / 2.0f);

    Orientation angles;

    int inchesToEncoder(float inches){
        final double     COUNTS_PER_MOTOR_REV    = 384.5 ;    // eg: TETRIX Motor Encoder
        final double     DRIVE_GEAR_REDUCTION    = 2 ;     // This is < 1.0 if geared UP
        final double     WHEEL_DIAMETER_INCHES   = 6 ;     // For figuring circumference
        final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
        int counts = (int) (COUNTS_PER_INCH * inches);
        return(counts);
    }

    void resetMotors() {
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    void motorMove(){
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(frontLeftDrive.isBusy() || frontRightDrive.isBusy() || backLeftDrive.isBusy() || backRightDrive.isBusy()){
            // Do Nothing
        }
    }

    void noEncoder(){
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void motorPower(float power){
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);
    }
    void motorPower_Left(float power) {
        frontLeftDrive.setPower(power);
        backLeftDrive.setPower(power);
    }
    void motorPower_Right(float power) {
        frontRightDrive.setPower(power);
        backRightDrive.setPower(power);
    }

    void tankDrive(float power, float rotL, float rotR) {
        float timeL = abs(rotL) * WHEEL_CIRCUMFERENCE / (64.84f * power) * 1000.0f;
        float timeR = abs(rotR) * WHEEL_CIRCUMFERENCE / (64.84f * power) * 1000.0f;
        float avgTime = (timeL + timeR) / 2.0f;
        float speedL = (rotL * WHEEL_CIRCUMFERENCE) / avgTime;
        float speedR = (rotR * WHEEL_CIRCUMFERENCE) / avgTime;
        float powerL = speedL / 64.84f * 1000;
        float powerR = speedR / 64.84f * 1000;
        motorPower_Left(powerL);
        motorPower_Right(powerR);
        sleep((int)avgTime);
        motorPower(0.0f);
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

    public double roll2() {
        Quaternion quatangles = imu.getQuaternionOrientation();

        double w = quatangles.w;
        double x = quatangles.x;
        double y = quatangles.y;
        double z = quatangles.z;

        double yaw = Math.atan2(2 * (w * z + x * y), 1 - (2 * (y * y + z * z))) * 180.0 / Math.PI;
        if (yaw < 0) {
            return (yaw + 360);
        }
        return (yaw);
    }

    public double roll3() {
        Quaternion quatangles = imu.getQuaternionOrientation();

        double w = quatangles.w;
        double x = quatangles.x;
        double y = quatangles.y;
        double z = quatangles.z;

        double yaw = Math.atan2(2 * (w * z + x * y), 1 - (2 * (y * y + z * z))) * 180.0 / Math.PI;
        if (yaw > 0) {
            return (yaw - 360);
        }
        return (yaw);
    }


    /*
    Movement
    */
    //drives forwards using encoders
    void driveF(float power, int inches){ // Drive Forward
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
                        return(toString().valueOf(roll()));
                    }
                });
        double start= roll();
        do{
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.update();
            motorPower_Right(-power);
            motorPower_Left(power);
        }while((roll()-start)>=degrees && opModeIsActive());
        motorPower(0);
    }

    void driveRL(float power, float degrees){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addLine()
                .addData("Turn: ", new Func<String>() {
                    @Override public String value() {
                        return(toString().valueOf(roll()));
                    }
                });
        double start= roll();
        do{
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.update();
            motorPower_Right(power);
            motorPower_Left(-power);
        }while((roll()-start)<=degrees && opModeIsActive());
        motorPower(0);
    }

    void driveRR2(float power, float degrees){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addLine()
                .addData("Turn: ", new Func<String>() {
                    @Override public String value() {
                        return(toString().valueOf(roll3()));
                    }
                });
        double start= roll3();
        do{
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.update();
            motorPower_Right(-power);
            motorPower_Left(power);
        }while((roll3()-start)>=degrees && opModeIsActive());
        motorPower(0);
    }

    void driveRL2(float power, float degrees){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addLine()
                .addData("Turn: ", new Func<String>() {
                    @Override public String value() {
                        return(toString().valueOf(roll2()));
                    }
                });
        double start= roll2();
        do{
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.update();
            motorPower_Right(power);
            motorPower_Left(-power);
        }while((roll2()-start)<=degrees && opModeIsActive());
        motorPower(0);
    }


    void powerTurn () {} // Complete later

    @Override
    public void runOpMode() throws InterruptedException {
        backLeftDrive = hardwareMap.get(DcMotor.class, "dtBL");
        backRightDrive = hardwareMap.get(DcMotor.class, "dtBR");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "dtFL");
        frontRightDrive = hardwareMap.get(DcMotor.class, "dtFR");
        Feeder = hardwareMap.get(DcMotor.class,"Feeder");
        Carousel = hardwareMap.get(DcMotor.class, "Color Wheel");
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        float motorPower = 0.5f;
        float secOffset = 0.031f;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);

        waitForStart();

        // FORMULA for timInSeconds to Inches :
        // inches / 32.42 = timeInSeconds
        // Round to the nearest thousandth

        driveF(-0.5f, -24);
        resetMotors();
        Feeder.setPower(-0.5f);
        sleep(2000);
        Feeder.setPower(0);
        driveF(0.5f, 4);
        driveRR(0.5f, -85);
        driveF(-0.5f, -55);

    }
}

