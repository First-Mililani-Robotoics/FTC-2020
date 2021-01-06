package org.firstinspires.ftc.teamcode;

public class halfFieldRedAuto extends robotDeclarations {

    robotDeclarations robot   = new robotDeclarations();


@Override public void runOpMode() {

    robot.init(hardwareMap);


    // Send telemetry message to signify robot waiting;
    telemetry.addData("Status", "Resetting Encoders");    //
    telemetry.update();

    telemetry.addData("Path0", "Starting at %7d :%7d",
            robot.leftFrontDrive.getCurrentPosition(),
            robot.rightFrontDrive.getCurrentPosition());
            robot.leftBackDrive.getCurrentPosition(),
            robot.rightBackDrive.getCurrentPosition();
    telemetry.update();

    // This is getting the amount of inches the wheel has to turn to reach 90 degrees
    double inchesMoved = degreesToInches(45);
    robot.foundationOne.setPosition(0);
    robot.foundationTwo.setPosition(1);




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