package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
public class robotDeclarations extends LinearOpMode {

public void runOpMode(){

    }
    //determines variables for  turning//
    final static int DIAMETER = 4; // This is the diameter of the wheels of the robot in inches

    final static double CIRCUMFRENCE = Math.PI * DIAMETER; // calculates the circumference of the wheels in inches
    final static double GEARREDUCTION = 1; // If we were to have gears the gear reduction would go up or down depending
    final static int    TICKSPERROTATION = 560; // there is 1680 ticks per a revolution of a motor
    public final static double ROBOTRADIOUS = 8.5; // the radious of the robot is 8.5 inches
    public final static double TILELENGTH = 22.75; // the length of a tile is 22.75 inches
    public final static double TICKSPERANINCHSTRAFING = 37;


    //determine variables for driving//
    static final double COUNTS_PER_MOTOR_REV = 1440;
    static final double DRIVE_GEAR_REDUCTION = 2.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 1.0;
    static final double TURN_SPEED = 0.5;


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor intakeDrive = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    public void init(HardwareMap hardwareMap) {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        intakeDrive = hardwareMap.get(DcMotor.class, "intake_drive");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        intakeDrive.setDirection(DcMotor.Direction.REVERSE);
        // Tell the driver that initialization is complete.

        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

    //make encoder drive function
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {

        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftFrontDrive.setTargetPosition((int)(leftInches * COUNTS_PER_INCH));
            rightFrontDrive.setTargetPosition((int)(rightInches * COUNTS_PER_INCH));
            leftBackDrive.setTargetPosition((int)(leftInches * COUNTS_PER_INCH));
            rightBackDrive.setTargetPosition((int)(rightInches * COUNTS_PER_INCH));

            // Turn On RUN_TO_POSITION
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftBackDrive.isBusy() && rightBackDrive.isBusy() && leftFrontDrive.isBusy() && rightFrontDrive.isBusy())) {

                if(leftInches < 0 && rightInches < 0)
                {
                    leftBackDrive.setPower(-speed);
                    leftFrontDrive.setPower(-speed);
                    rightBackDrive.setPower(-speed);
                    rightFrontDrive.setPower(-speed);
                }
                else if(leftInches > 0 && rightInches > 0)
                {
                    leftBackDrive.setPower(speed);
                    leftFrontDrive.setPower(speed);
                    rightBackDrive.setPower(speed);
                    rightFrontDrive.setPower(speed);
                }
                // Display it for the driver.
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftFrontDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition());
                telemetry.update();
            }

        }
    }
}