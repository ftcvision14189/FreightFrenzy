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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="UltimateGoal_LinearOpMode", group="Linear Opmode")
@Disabled
public class UltimateGoal_LinearOpMode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // Declare motors
    DcMotor FLMotor;
    DcMotor FRMotor;
    DcMotor BLMotor;
    DcMotor BRMotor;
    DcMotor ItMotor;
    DcMotor FeMotor;
    DcMotor StMotor;
    CRServo WGServo;
    CRServo ItServo;

    // Declare motor powers
    double FLPower;
    double FRPower;
    double BLPower;
    double BRPower;
    double ItPower;
    double FePower;
    double StPower;
    double WGPower;

    // Declare constants
    boolean DtDirection = true;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Map motors
        FLMotor = hardwareMap.dcMotor.get("dtFL");
        FRMotor = hardwareMap.dcMotor.get("dtFR");
        BLMotor = hardwareMap.dcMotor.get("dtBL");
        BRMotor = hardwareMap.dcMotor.get("dtBR");
        ItMotor = hardwareMap.dcMotor.get("Intake");
        FeMotor = hardwareMap.dcMotor.get("Feeder");
        StMotor = hardwareMap.dcMotor.get("Shooter");
        WGServo = hardwareMap.crservo.get("Flip");
        ItServo = hardwareMap.crservo.get("393");


        // Reverse right side
        FRMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BRMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Shooter encoder setup
        StMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        StMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Dt power input
            if(gamepad1.right_bumper) {
                DtDirection = !DtDirection;
            }

            if(DtDirection) {
                FLPower = gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x / 1.75;
                FRPower = gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x / 1.75;
                BLPower = gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x / 1.75;
                BRPower = gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x / 1.75;
            }

            if(!DtDirection) {
                FLPower = -gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x / 1.75;
                FRPower = -gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x / 1.75;
                BLPower = -gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x / 1.75;
                BRPower = -gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x / 1.75;
            }

            // Dt power set
            FLMotor.setPower(FLPower);
            FRMotor.setPower(FRPower);
            BLMotor.setPower(BLPower);
            BRMotor.setPower(BRPower);

            // Intake power set
            if(gamepad2.a){
                ItPower = 1;
            }else if(gamepad2.b){
                ItPower = -1;
            }else{
                ItPower = 0;
            }
            ItMotor.setPower(ItPower);
            //ItServo.setPower(ItPower * 200);
            //WGServo.setPower(ItPower);

            // Feeder power set
            if(gamepad1.x){
                FePower = -1;
            }else if (gamepad1.y){
                FePower = 1;
            }else{
                FePower = 0;
            }
            FeMotor.setPower(FePower);

            // Shooter power set
            if(gamepad1.a){
                StMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                StPower = 0.4;
            }else if(gamepad1.b){
                StMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                StPower = 1;
            }else{
                StMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            StMotor.setPower(StPower);

            //test
            if(gamepad2.x){
                WGPower = 1;
            }else if(gamepad2.y){
                WGPower = -1;
            }else{
                WGPower = 0;
            }
            WGServo.setPower(WGPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}