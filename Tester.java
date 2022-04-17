package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.robot.Robot;


@Autonomous(name = "Tester", group = "Autonomous")
public class Tester extends LinearOpMode
{
    //vuforia initialisation
    public VuforiaSkystoneDetector vuforiaDetector;
    int a;
    VuforiaSkystoneDetector.skystonePos pos;
    private VuforiaLocalizer vuforia;
    private static final String VUFORIA_KEY = "AWaEPBn/////AAAAGWa1VK57tkUipP01PNk9ghlRuxjK1Oh1pmbHuRnpaJI0vi57dpbnIkpee7J1pQ2RIivfEFrobqblxS3dKUjRo52NMJab6Me2Yhz7ejs5SDn4G5dheW5enRNWmRBsL1n+9ica/nVjG8xvGc1bOBRsIeZyL3EZ2tKSJ407BRgMwNOmaLPBle1jxqAE+eLSoYsz/FuC1GD8c4S3luDm9Utsy/dM1W4dw0hDJFc+lve9tBKGBX0ggj6lpo9GUrTC8t19YJg58jsIXO/DiF09a5jlrTeB2LK+GndUDEGyZA1mS3yAR6aIBeDYnFw+79mVFIkTPk8wv3HIQfzoggCu0AwWJBVUVjkDxJOWfzCGjaHylZlo";
    
    //hardware components
    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;
    //servo for autonomous grabber
    Servo S1;
    Servo S2;
    //servo for movement of foundation
    Servo F1;
    Servo F2;
    //TeleOp Components for configuration
    DcMotor In1,In2, Rot;
    Servo del;
    DcMotor pull;
    //capstone mechanism
    Servo Capstone ;
    
    @Override
    public void runOpMode() throws InterruptedException
    {   
      //hardwareMap
        FL = hardwareMap.dcMotor.get("FL");
        BR = hardwareMap.dcMotor.get("BR");
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        //servo for grabbing
        // S1 = hardwareMap.servo.get("S1");
        // S2 = hardwareMap.servo.get("S2"); 
        // //foundation movement 
        // F1 = hardwareMap.servo.get("F1");
        // F2 = hardwareMap.servo.get("FOUND2");
        // //teleop components 
        // In1= hardwareMap.dcMotor.get("In1");
        // In2= hardwareMap.dcMotor.get("In2");
        // del= hardwareMap.servo.get("Drop"); 
        // Rot = hardwareMap.dcMotor.get("Rotation");
        // //Pulley Mechanism
        // pull= hardwareMap.dcMotor.get("Pulley");
        // //capstone mechanism
        // Capstone = hardwareMap.servo.get("Cap"); 
            
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforiaDetector = new VuforiaSkystoneDetector(vuforia, telemetry);
        telemetry.addData("VUFORIA: ", "Scanning ...");
        telemetry.update();

        pos = vuforiaDetector.detectSkystone();

        waitForStart();
        
        switch(pos)
        {
            case LEFT:
               telemetry.addData("VUFORIA: ", "LEFT");
               break;
            case RIGHT:
               telemetry.addData("VUFORIA: ", "RIGHT");
               
               break;
              
            case CENTER:
               telemetry.addData("VUFORIA: ", "CENTER");
               break;
            default:
               telemetry.addData("VUFORIA: ", "** UNKNOWN **");
               break;
        }
        
        telemetry.addLine("Sleeping for 2000 ms");
        telemetry.update();
        sleep(2000);
    }
}
