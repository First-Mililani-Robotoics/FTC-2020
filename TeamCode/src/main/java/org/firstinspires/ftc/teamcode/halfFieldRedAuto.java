package org.firstinspires.ftc.teamcode;

public class halfFieldRedAuto extends robotDeclarations {

    robotDeclarations robot = new robotDeclarations();


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftFrontDrive.getCurrentPosition(),
                robot.rightFrontDrive.getCurrentPosition());
        telemetry.update();

        //Wait for game to start
        waitForStart();

        //movements
        robot.encoderDrive(1.0, 44.0, 44.0, 5.0); //Forward 44 inches with a 5 second timeout
        robot.turnDrive( 0.5, 45.0, 5.0 ); // Turn right 45 degree
        robot.encoderDrive(1.0, 22.6, 22.6, 5.0); //Forward 22.6 inches
        robot.turnDrive(1.0, -45.0, 5.0); // Turn right 45 degrees
        //you still need to shoot-- make a pivot and shooter motor
        robot.encoderDrive();
    }
}

