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
        robot.leftBackDrive.getCurrentPosition(),
                robot.rightBackDrive.getCurrentPosition();
        telemetry.update();

        // This is getting the amount of inches the wheel has to turn to reach 45 degrees
        double inchesMoved = degreesToInches(45);
        robot.foundationOne.setPosition(0);
        robot.foundationTwo.setPosition(1);

//create a new function to make degrees. Degrees to ticks,
        //Turning
        public void turnDrive ( double speed, double degrees, double timeoutS){
            int turnLeftTarget;
            int turnRightTarget;
            double rightPower;
            double leftPower;

            //OpMode
            if (opModeIsActive()) {
                //Find the new position)
                double degreeInches = (degrees * (Math.PI / 180) * ROBOT_RADIUS);
                if (0 < degrees) {
                    turnLeftTarget = leftFrontDrive.getCurrentPosition() + (int) (degreeInches * COUNTS_PER_INCH);
                    turnRightTarget = rightBackDrive.getCurrentPosition() + (int) (-degreeInches * COUNTS_PER_INCH);
                    rightPower = -speed;
                    leftPower = speed;
                } else {
                    turnLeftTarget = leftFrontDrive.getCurrentPosition() + (int) (-degreeInches * COUNTS_PER_INCH);
                    turnRightTarget = rightBackDrive.getCurrentPosition() + (int) (degreeInches * COUNTS_PER_INCH);
                    rightPower = speed;
                    leftPower = -speed;
                }

                //wait for game to start (press PLAY)
                waitForStart();

                encoderDrive(DRIVE_SPEED, 44, 44, 5.0);  // S1: Forward 44 Inches with 5 Sec timeout
                encoderDrive(TURN_SPEED, 45, -45, 4.0);  // S2: Turn Right 44 Inches with 4 Sec timeout
                encoderDrive(DRIVE_SPEED, 22.6, 22.6, 4.0);  // S3: Forward 22.6 Inches with 4 Sec timeout
                encoderDrive(TURN_SPEED, -45, 45, 4.0);   // S2: Turn left 44 Inches with 4 Sec timeout



                //*wait for game to start (press PLAY)

                //waitForStart();
                //encoderDrive(DRIVE_SPEED, 44, 44, 5.0);
                //encoderDrive(TURN_SPEED, 12, -12, 4.0);
                //encoderDrive(DRIVE_SPEED, -24, -24, 4.0);


            }
        }
    }
}