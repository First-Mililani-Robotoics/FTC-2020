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



            }
        }

