package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.abs;

@Autonomous(name="Test Auto", group="Auto")

public class FreightFrenzy_AutoTemplate extends LinearOpMode{
    BNO055IMU imu;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor Feeder = null;
    private DcMotor Carousel = null;

    Orientation angles;

    int inchesToEncoder(float inches){
        final double     COUNTS_PER_MOTOR_REV    = 751.8 ;    // eg: TETRIX Motor Encoder
        final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
        final double     WHEEL_DIAMETER_INCHES   = 3.93701 ;     // For figuring circumference
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

    void driveF(float power, int inches){ // Drive Forward
        resetMotors();
        motorPower(power);
        int distance = inchesToEncoder(inches);
        frontLeftDrive.setTargetPosition(distance);
        frontRightDrive.setTargetPosition(distance);
        backLeftDrive.setTargetPosition(distance);
        backRightDrive.setTargetPosition(distance);
        motorMove();
    }

    void gyroTurnRight (float power, float endAngle) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(-power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(-power);

        sleep((long)(500-(power*50)));

        telemetry.addLine()
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return(toString().valueOf(angles.thirdAngle));
                    }
                });
        telemetry.addLine()
            .addData("Pitch", new Func<String>() {
                @Override public String value() {
                    return(toString().valueOf(angles.secondAngle));
            }
        });
        telemetry.addLine()
            .addData("heading", new Func<String>() {
                @Override public String value() {
                    return(toString().valueOf(angles.firstAngle));
            }
        });

        do{
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.update();
        }while(angles.secondAngle >= (endAngle) && opModeIsActive());

        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    void gyroTurnLeft (float power, float endAngle) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        frontLeftDrive.setPower(-power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(-power);
        backRightDrive.setPower(power);

        sleep((long)(500-(power*50)));

        telemetry.addLine()
                .addData("Roll", new Func<String>() {
                    @Override public String value() {
                        return(toString().valueOf(angles.secondAngle));
                    }
                });

        do{
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.update();
        }while(angles.secondAngle <= (endAngle) && opModeIsActive());

        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    void gyroTurn(float power, float degrees, String direction) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float startAngle = angles.secondAngle;
        float travelAngle = degrees;
        if(direction.equals("Right")){
            float endAngle = startAngle - travelAngle;
            if (abs(endAngle) >= 180){
                gyroTurnRight(power,-178);
                gyroTurnRight(power, (endAngle+360));
            }
            else if (0 < abs(endAngle)) {
                gyroTurnRight(power, endAngle);
            }

        }else if(direction.equals("Left")){
            float endAngle = angles.secondAngle + degrees;
            if (abs(endAngle) >= 180){
                gyroTurnLeft(power,178);
                gyroTurnLeft(power, (endAngle - 360));
            }
            else if (0 < abs(endAngle)) {
                gyroTurnLeft(power, (endAngle));
            }
        }
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

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);

        waitForStart();

        // Turn right
        /*driveR(0.5f, 45);
        sleep(1000);
        driveR(0.5f, 90);
        sleep(1000);
        driveR(0.5f, 360);
        sleep(1000);
        driveR(1.0f, 90);*/

        //gyroTurn(0.5f, 90, "Right");
        //sleep(1000);
        //gyroTurn(0.5f, 90, "Left");
        //sleep(1000);
        gyroTurn(0.5f, 90, "Left");
        sleep(1000);
        //gyroTurn(0.5f, 35, "Left");
        //sleep(1000);
        gyroTurn(0.5f, 180, "Left");
        //sleep(1000);
        //gyroTurn(0.5f, 100, "Left");
        /*gyroTurnLeft(.3f, 90);
        sleep(1000);
        gyroTurnLeft(.3f, 178);
        sleep(1000);
        gyroTurnLeft(.3f, -90);
        sleep(1000);*/


    }
}

