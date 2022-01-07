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

import static java.lang.Math.abs;

@Autonomous(name="BlueWarehouse", group="Auto")

public class FreightFrenzy_BlueWarehouse extends LinearOpMode{
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;

    @Override
    public void runOpMode() throws InterruptedException {
        backLeftDrive = hardwareMap.get(DcMotor.class, "dtBL");
        backRightDrive = hardwareMap.get(DcMotor.class, "dtBR");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "dtFL");
        frontRightDrive = hardwareMap.get(DcMotor.class, "dtFR");

        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        //FreightFrenzy_AutoFunctions.noEncoder();
        //FreightFrenzy_AutoFunctions.driveF(-.2f, -6);
        //FreightFrenzy_AutoFunctions.driveRR(0.3f, 90);
    }
}

