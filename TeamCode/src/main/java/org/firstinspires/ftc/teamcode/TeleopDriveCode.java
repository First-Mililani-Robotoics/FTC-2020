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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//import java.awt.image.DirectColorModel;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOp", group="Iterative Opmode")
//@Disabled
public class TeleopDriveCode extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftFrontDrive = null;
    public DcMotor leftRearDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightRearDrive = null;
    public DcMotor intake = null;
    public DcMotor pivot = null;
    public DcMotor flywheelOne = null;
    public DcMotor flywheelTwo = null;

    public Servo feeder = null;

    static final double     COUNTS_PER_MOTOR_REV    = 134.4 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.2 ;     // This is < 1.0 if geared UP
    static final double     PIVOT_ARM    = 0.24 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (PIVOT_ARM * 3.1415);
    int safety = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "left_rear_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "right_rear_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        pivot = hardwareMap.get(DcMotor.class, "pivot");
        flywheelOne = hardwareMap.get(DcMotor.class, "fwOne");
        flywheelTwo = hardwareMap.get(DcMotor.class, "fwTwo");


        feeder = hardwareMap.get(Servo.class, "feeder");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        pivot.setDirection(DcMotor.Direction.FORWARD);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelOne.setDirection(DcMotor.Direction.FORWARD);
        flywheelTwo.setDirection(DcMotor.Direction.FORWARD);
        feeder.setPosition(0.6);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;
        double intakePower = gamepad2.right_trigger - gamepad2.left_trigger;
        boolean shoot = gamepad1.right_bumper;
        boolean drop = gamepad1.left_bumper;
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;

        leftPower    = Range.clip(drive + turn, -0.8, 0.8) ;
        rightPower   = Range.clip(drive - turn, -0.8, 0.8) ;

        leftFrontDrive.setPower(leftPower);
        leftRearDrive.setPower(leftPower);
        rightFrontDrive.setPower(rightPower);
        rightRearDrive.setPower(rightPower);
        intake.setPower(intakePower);
        if(shoot){
            if(safety == 0) {
                pivotTo(2.2);
                //pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                flywheelOne.setPower(0.7);
                flywheelTwo.setPower(0.7);
            }
            safety = 1;
        }

        if(drop){
            if(safety == 1) {
                pivotTo(-2.2);
                flywheelOne.setPower(0.0);
                flywheelTwo.setPower(0.0);
            }
            safety = 0;
        }

        if(gamepad1.y){
            flywheelOne.setPower(0.6);
            flywheelTwo.setPower(0.6);
        }

        if(gamepad1.x){
            flywheelOne.setPower(0);
            flywheelTwo.setPower(0);
        }

        if(gamepad2.a){
            double startTime = runtime.time();
            while(startTime != -1){
                if(runtime.time()-startTime>0.5){
                    feeder.setPosition(0.6);
                    startTime = -1;
                }
                else{
                    feeder.setPosition(0.1);
                }
            }

        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "leftPower (%.2f), rightPower (%.2f), intake (%.2f)", leftPower, rightPower, intakePower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public void pivotTo(double angle){
        double distanceToTravel = PIVOT_ARM*angle;
        int angleTarget = pivot.getCurrentPosition()+(int)(distanceToTravel*COUNTS_PER_INCH);
        pivot.setTargetPosition(angleTarget);
        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivot.setPower(0.35);
        if(pivot.getCurrentPosition()==distanceToTravel){
            pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
}