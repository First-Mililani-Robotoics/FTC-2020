package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
@Autonomous (name = "Vuforia", group = "Pushbot")

public class Vuforia extends LinearOpMode{
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String QUAD_ELEMENT = "Quad";
    private static final String SINGLE_ELEMENT = "Single";
    private static final String vuforiaKey = ""; //add key later
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    @Override
    public void runOpMode(){
        initVuforia();
        initTfod();
        tfod.activate();
        waitForStart();
        //tfod instance motor activate method, things that are run
    }
    private void initVuforia() {}
    private void initTfod() {}


}
