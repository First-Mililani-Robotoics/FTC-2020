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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "teleop", group = "Iterative Opmode")
//@Disabled
public class teleop extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFoward = null;
    private DcMotor rightReverse = null;
    private DcMotor leftReverse = null;
    private DcMotor rightFoward = null;
    private DcMotor intakeOne = null;
    private DcMotor shooterOne = null;
    private DcMotor shooterTwo = null;


    /*
     * Code to run ONCE when the driver hits INIT
     */
        public void init() {
            telemetry.addData("Status", "Initialized");

            // Initialize the hardware variables. Note that the strings used here as parameters
            // to 'get' must correspond to the names assigned during the robot configuration
            // step (using the FTC Robot Controller app on the phone).
            //Name motors

            // Initialize the hardware variables. Note that the strings used here as parameters
            // to 'get' must correspond to the names assigned during the robot configuration
            // step (using the FTC Robot Controller app on the phone).
            leftFoward = hardwareMap.get(DcMotor.class, "leftFront");
            rightReverse = hardwareMap.get(DcMotor.class, "rightBack");
            leftReverse = hardwareMap.get(DcMotor.class, "leftBack");
            rightFoward = hardwareMap.get(DcMotor.class, "rightFront");
            intakeOne = hardwareMap.get(DcMotor.class, "intake");
            shooterOne = hardwareMap.get(DcMotor.class, "shooter1");
            shooterTwo = hardwareMap.get(DcMotor.class, "shooter2");
            // Most robots need the motor on one side to be reversed to drive forward
            // Reverse the motor that runs backwards when connected directly to the battery
            leftFoward.setDirection(DcMotor.Direction.FORWARD);
            rightReverse.setDirection(DcMotor.Direction.REVERSE);
            leftReverse.setDirection(DcMotor.Direction.FORWARD);
            rightFoward.setDirection(DcMotor.Direction.REVERSE);
            intakeOne.setDirection(DcMotor.Direction.FORWARD);
            shooterOne = setDirection(DcMotor.Direction.FORWARD);
            shooterTwo = setDirection(DcMotor.Direction.FORWARD);
            //Reset encoders
            leftFoward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightReverse.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftReverse.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFoward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //Reset power
            leftFoward.setPower(0);
            rightReverse.setPower(0);
            leftReverse.setPower(0);
            rightFoward.setPower(0);
            intakeOne.setPower(0);
            shooterOne.setPower(0);
            shooterTwo.setPower(0);

            //use encoders
            leftFoward.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightReverse.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftReverse.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFoward.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftFowardPower;
        double rightReversePower;
        double leftReversePower;
        double rightFowardPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        //double drive = -gamepad1.left_stick_y;
        //double turn  =  gamepad1.right_stick_x;
        // leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        // rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.

        //when button pushed, power on
        if (gamepad2.a == true) {
            intakeOne.setPower(.5);
        }
        if (gamepad2.a == false) {
            intakeOne.setPower(0);
        }
        if (gamepad2.b == true){
            intakeOne.setPower(-.5);
        }
        if (gamepad2.b == false){
            intakeOne.setPower(0);
        }

        if (gamepad2.right_bumper == true) {
            shooterOne.setPower(1);
            shooterTwo.setPower(1);
        }
        if (gamepad2.left_bumper == false) {
            shooterOne.setPower(0);
            shooterTwo.setPower(0);
        }

        //Left side: If the stick is held left or right, then set powers to strafe.
        // Else the stick will be held vertically, so the power is just how much you push the stick
        if (Math.abs(gamepad1.left_stick_x) > Math.abs(gamepad1.left_stick_y)){
            leftFowardPower = -gamepad1.left_stick_x;
            leftReversePower = gamepad1.left_stick_x;
        } else {
            leftReversePower = gamepad1.left_stick_y;
            leftFowardPower = gamepad1.left_stick_y;
        }
        //right side: If the stick is held left or right, then set powers to strafe.
        // Else the stick will be held vertically, so the power is just how much you push the stick
        if (Math.abs(gamepad1.right_stick_x) > Math.abs(gamepad1.right_stick_y)){
            rightFowardPower = -gamepad1.right_stick_x;
            rightReversePower = gamepad1.right_stick_x;
        } else {
            rightReversePower = gamepad1.right_stick_y;
            rightFowardPower = gamepad1.right_stick_y;
        }
        /*
        Tank drive set up
        leftReversePower = gamepad1.left_stick_y;
        leftFowardPower = gamepad1.left_stick_y;
        rightReversePower = gamepad1.left_stick_y;
        rightFowardPower = gamepad1.left_stick_y;
         */
        //intakePower = -gamepad2.left_stick_y ;

        // Send calculated power to wheels
        leftFoward.setPower(leftFowardPower);
        rightReverse.setPower(rightReversePower);
        leftReverse.setPower(leftReversePower);
        rightFoward.setPower(rightFowardPower);
        intakeOne.setPower(intakeOnePower);


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors",
                "leftForward (%.2f), rightReverse (%.2f), leftReverse (%.2f), rightForward (%.2f), intakeOnePower (%.2f)",
                leftFowardPower, rightReversePower, leftReversePower, rightFowardPower, intakeOnePower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
