package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class robotDeclarations {
}

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
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
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


    private ElapsedTime     runtime = new ElapsedTime(); // For the use of time
//determines variables for  turning//
    final static int DIAMETER                = 4; // This is the diameter of the wheels of the robot in inches

    final static double CIRCUMFRENCE         = Math.PI * DIAMETER; // calculates the circumference of the wheels in inches
    final static double GEARREDUCTION        = 1; // If we were to have gears the gear reduction would go up or down depending
    final static int TICKSPERROTATION        = 560; // there is 1680 ticks per a revolution of a motor
    public final static double ROBOTRADIOUS  = 8.5; // the radious of the robot is 8.5 inches
    public final static double TILELENGTH    = 22.75; // the length of a tile is 22.75 inches
    public final static double TICKSPERANINCHSTRAFING = 37;

    //determins the amount of ticks per an inch
    static final double COUNTS_PER_INCH = (TICKSPERROTATION * GEARREDUCTION) / CIRCUMFRENCE;
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS)


//determine variables for driving//
    static final double COUNTS_PER_MOTOR_REV    = 1440 ;
    static final double DRIVE_GEAR_REDUCTION    = 2.0 ;
    static final double WHEEL_DIAMETER_INCHES   = 4.0 ;
    static final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double  DRIVE_SPEED            = 1.0;
    static final double  TURN_SPEED             = 0.5;

    @Override public void runOpMode() {

        robot.init(hardwareMap);


        telemetry.addData("Status", "Resetting Encoders");   //
        telemetry.update();

        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        // Define and Initialize Motors

    }