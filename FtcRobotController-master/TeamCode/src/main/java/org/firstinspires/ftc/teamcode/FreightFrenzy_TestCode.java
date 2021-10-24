package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="Test Auto", group="Auto")

public class FreightFrenzy_TestCode extends LinearOpMode{

    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor Shooter = null;
    private DcMotor Feeder = null;
    private DcMotor Intake = null;

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

    void driveR(float power, int degrees) { // Drive Rotate
        resetMotors();
        motorPower(power);
        int inches = (int) (degrees * 0.2);
        int distance = inchesToEncoder(inches);
        frontLeftDrive.setTargetPosition(distance);
        frontRightDrive.setTargetPosition(-distance);
        backLeftDrive.setTargetPosition(distance);
        backRightDrive.setTargetPosition(-distance);
        motorMove();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        backLeftDrive = hardwareMap.get(DcMotor.class, "dtBL");
        backRightDrive = hardwareMap.get(DcMotor.class, "dtBR");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "dtFL");
        frontRightDrive = hardwareMap.get(DcMotor.class, "dtFR");
        Shooter = hardwareMap.get(DcMotor.class,"Shooter" );
        Feeder = hardwareMap.get(DcMotor.class,"Feeder");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        // Set power for motors



        // Go forwards
       //driveF(0.25f, 12);

        // Turn right
        driveR(0.5f, 45);
        sleep(1000);
        driveR(0.5f, 90);
        sleep(1000);
        driveR(0.5f, 360);
        sleep(1000);
        driveR(1.0f, 90);
    }
}

