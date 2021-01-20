package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name ="arcadeDrive", group = "Iterative Opmode")

public class arcadeDrive extends OpMode {
        private ElapsedTime runtime = new ElapsedTime();
        public DcMotor leftFoward = null;
        public DcMotor rightReverse = null;
        private DcMotor leftReverse = null;
        public DcMotor rightFoward = null;
        private DcMotor intake = null;
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        leftFoward = hardwareMap.get(DcMotor.class, "left_foward_drive");
        rightReverse = hardwareMap.get(DcMotor.class, "right_reverse_drive");
        leftReverse = hardwareMap.get(DcMotor.class, "left_reverse_drive");
        rightFoward = hardwareMap.get(DcMotor.class, "right_foward_drive");
        intake = hardwareMap.get(DcMotor.class, "intake_intial");

        leftFoward.setDirection(DcMotor.Direction.FORWARD);
        rightReverse.setDirection(DcMotor.Direction.REVERSE);
        leftReverse.setDirection(DcMotor.Direction.FORWARD);
        rightFoward.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);

        leftFoward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightReverse.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftReverse.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFoward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFoward.setPower(0);
        rightReverse.setPower(0);
        leftReverse.setPower(0);
        rightFoward.setPower(0);
        intake.setPower(0);


        leftFoward.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightReverse.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftReverse.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFoward.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
    }
    @Override
    public void start() {
        runtime.reset();
    }


    public double driveDegree(double y, double x) {
        double degreeRadians = (Math.atan2(y, x));
        double degree = degreeRadians * (180 / Math.PI) + 90;

        return degree >= 0? degree : degree + 360;
    }













    public void motorPower(double degrees, double forwardPower, double sidePower, double turnPower) {

        if (turnPower >= 0) {
            leftFoward.setPower(forwardPower);
            leftReverse.setPower(forwardPower); 


        }



    }


    @Override
    public void loop() {
        double forwardPower;
        double sidePower;
        double turnPower;

        double yPower = gamepad1.left_stick_y;
        double xPower = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        forwardPower = Range.clip(yPower, -1.0, 1.0);
        sidePower = Range.clip(xPower, -1.0, 1.0);
        turnPower = Range.clip(turn, -1.0, 1.0);


        double degrees = driveDegree(yPower, xPower);
        double magnitude = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double rotationSpeed =
    }



}


