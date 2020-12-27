package org.firstinspires.ftc.teamcode;

public class halfFieldRedAuto extends LinearOpMode {

    robotDeclarations robot   = new robotDeclarations();



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

    telemetry.addData("Path0", "Starting at %7d :%7d",
            robot.leftFrontDrive.getCurrentPosition(),
            robot.rightFrontDrive.getCurrentPosition());
    robot.leftBackDrive.getCurrentPosition(),
            robot.rightBackDrive.getCurrentPosition();
    telemetry.update();

//wait for game to start (press PLAY)
    waitForStart();

    encoderDrive(DRIVE_SPEED, 44, 44, 5.0);  // S1: Forward 44 Inches with 5 Sec timeout
    encoderDrive(TURN_SPEED, 12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
    encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

//wait for game to start (press PLAY)
    waitForStart();

    encoderDrive(DRIVE_SPEED, 44, 44, 5.0);  // S1: Forward 44 Inches with 5 Sec timeout
    encoderDrive(TURN_SPEED, 12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
    encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout







}