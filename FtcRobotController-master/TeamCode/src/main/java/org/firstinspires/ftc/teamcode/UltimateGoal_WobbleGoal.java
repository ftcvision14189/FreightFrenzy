package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Drop Wobble Goal", group="Auto")

public class UltimateGoal_WobbleGoal extends LinearOpMode{

    DcMotor FLMotor;
    DcMotor FRMotor;
    DcMotor BLMotor;
    DcMotor BRMotor;
    DcMotor FeMotor;
    DcMotor StMotor;
    CRServo WGServo;

    int distance;
    double power;

    @Override
    public void runOpMode() throws InterruptedException {
        FLMotor = hardwareMap.dcMotor.get("dtFL");
        FRMotor = hardwareMap.dcMotor.get("dtFR");
        BLMotor = hardwareMap.dcMotor.get("dtBL");
        BRMotor = hardwareMap.dcMotor.get("dtBR");
        FeMotor = hardwareMap.dcMotor.get("Feeder");
        StMotor = hardwareMap.dcMotor.get("Shooter");
        WGServo = hardwareMap.crservo.get("Flip");
        FRMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BRMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        distance = 0;
        waitForStart();

        // Set power for motors
        power = 0.75;
        FLMotor.setPower(power);
        FRMotor.setPower(power);
        BLMotor.setPower(power);
        BRMotor.setPower(power);
        FeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        StMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Go forwards
        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        distance = 3800;
        FLMotor.setTargetPosition(distance);
        FRMotor.setTargetPosition(distance);
        BLMotor.setTargetPosition(distance);
        BRMotor.setTargetPosition(distance);
        FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(FLMotor.isBusy() && FRMotor.isBusy() && BLMotor.isBusy() && BRMotor.isBusy()){

        }

        //Drop wobble goal
        WGServo.setPower(1);
        sleep(4000);
        WGServo.setPower(0);

        // Go Backwards
        power = 0.40;
        FLMotor.setPower(power);
        FRMotor.setPower(power);
        BLMotor.setPower(power);
        BRMotor.setPower(power);
        FeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        StMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        distance = -1600;
        FLMotor.setTargetPosition(distance);
        FRMotor.setTargetPosition(distance);
        BLMotor.setTargetPosition(distance);
        BRMotor.setTargetPosition(distance);
        FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(FLMotor.isBusy() && FRMotor.isBusy() && BLMotor.isBusy() && BRMotor.isBusy()){

        }
        sleep(500);

        // Turn right
        power = 0.40;
        FLMotor.setPower(power);
        FRMotor.setPower(power);
        BLMotor.setPower(power);
        BRMotor.setPower(power);
        FeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        StMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        distance = 633;
        FRMotor.setTargetPosition(distance);
        BRMotor.setTargetPosition(distance);
        FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(FLMotor.isBusy() && FRMotor.isBusy() && BLMotor.isBusy() && BRMotor.isBusy()){

        }
        power = 0.40;
        FLMotor.setPower(power);
        FRMotor.setPower(power);
        BLMotor.setPower(power);
        BRMotor.setPower(power);
        FeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        StMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        distance = 1150;
        FLMotor.setTargetPosition(distance);
        FRMotor.setTargetPosition(distance);
        BLMotor.setTargetPosition(distance);
        BRMotor.setTargetPosition(distance);
        FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(FLMotor.isBusy() && FRMotor.isBusy() && BLMotor.isBusy() && BRMotor.isBusy()){

        }

        // Charge shooter
        StMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        StMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        StMotor.setPower(power);
        sleep(5000);

        // First shot
        FeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FeMotor.setPower(-power);
        sleep(610);
       //StMotor.setPower(0);
        FeMotor.setPower(0);

        // Turn right
        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        distance = 69;
        FRMotor.setTargetPosition(distance);
        BRMotor.setTargetPosition(distance);
        FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(FLMotor.isBusy() && FRMotor.isBusy() && BLMotor.isBusy() && BRMotor.isBusy()){

        }
        // Charge shooter
        StMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        StMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        StMotor.setPower(power);
        sleep(4000);

        // Second shot
        FeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FeMotor.setPower(-power);
        sleep(1000);
        //StMotor.setPower(0)
        FeMotor.setPower(0);;

        // Turn right
        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        distance = 170;
        FRMotor.setTargetPosition(distance);
        BRMotor.setTargetPosition(distance);
        FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(FLMotor.isBusy() && FRMotor.isBusy() && BLMotor.isBusy() && BRMotor.isBusy()){

        }
        // Charge shooter
        StMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        StMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        StMotor.setPower(power);
        sleep(4000);

        // Third shot
        FeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FeMotor.setPower(-power);
        sleep(1100);
        //StMotor.setPower(0);
        FeMotor.setPower(0);

        //go forward
        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        distance = 500;
        FLMotor.setTargetPosition(distance);
        FRMotor.setTargetPosition(distance);
        BLMotor.setTargetPosition(distance);
        BRMotor.setTargetPosition(distance);
        FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(FLMotor.isBusy() && FRMotor.isBusy() && BLMotor.isBusy() && BRMotor.isBusy()){

        }

    }
}

