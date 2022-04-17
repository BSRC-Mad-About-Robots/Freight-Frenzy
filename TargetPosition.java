package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Point;

import java.io.File;

@Autonomous(name="TargetPosition")
public class TargetPosition extends LinearOpMode {
    // Handle hardware stuff...

    int width = 960;
    int height = 720;//changed this
    // store as variable here so we can access the location
    
    TestOpencv detector = new TestOpencv(telemetry);
    OpenCvCamera phoneCam;
    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;
    double first_valY=0;
    double second_valY=0;
    double third_valY=0;
    double fourth_valY=0;
    double fifth_valY=0;
    double final_valY=0;
    double beforeY;
    double afterY;
    double differnceY;
    double cmY =60;
    double CmPerInchY;
    double after_Area=0;
    double before_Area=0;
    double first_Area=0;
    double second_Area=0;
    double third_Area=0;
    double fourth_Area=0;
    double fifth_Area=0;
    double final_Area=0;
    double first_valX=0;
    double second_valX=0;
    double third_valX=0;
    double fourth_valX=0;
    double fifth_valX=0;
    double final_valX=0;
    double beforeX;
    double afterX;
    double differnceX;
    double cmX =60;
    double CmPerInchX;
    private ElapsedTime runtime = new ElapsedTime();
    
    //while motors are busy dont perform next action
    private void waitUntilMotorsBusy()
    {
        while (opModeIsActive() && (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy())) 
            {
                // Display it for the driver.
                telemetry.addData("Autonomous",  "Motors busy");
                telemetry.update();
            }
    }
    
    
    //basic methods 
    public void setWheelbasePower(double pow) 
    {
        FL.setPower(pow);
        FR.setPower(pow);
        BL.setPower(pow);
        BR.setPower(pow);
    }
 
    public void  directionForward()
    {
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void  directionBack()
    {
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void  directionRight()
    {
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public  void turnAnticlockwise() 
    {
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    //go clockwise
    public void turnClockwise()
    {
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    //go left
    public void  directionLeft()
    {
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    //set target position
    public  void  setTargetPosition(int pos)
    {
        FL.setTargetPosition(pos);
        FR.setTargetPosition(pos);
        BL.setTargetPosition(pos);
        BR.setTargetPosition(pos);
    }
    //to minimize repitition
    public void Premov()
    {
        //to stop and reset 
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       
        //run using encoder
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        //run to position
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    File CmperInchY = AppUtil.getInstance().getSettingsFile("Pixels/Inch.txt");
    File CmperInchX = AppUtil.getInstance().getSettingsFile("Cm/InchX.txt");
    File star_val=AppUtil.getInstance().getSettingsFile("Start_val");
    File TargetPositionY=AppUtil.getInstance().getSettingsFile("TargetPositionY.txt");
    File TargetPositionArea=AppUtil.getInstance().getSettingsFile("TargetPositionArea.txt");
    @Override
    public void runOpMode()
    {
        // robot logic...

        // https://github.com/OpenFTC/EasyOpenCV/blob/master/examples/src/main/java/org/openftc/easyopencv/examples/InternalCameraExample.java
        // Initialize the back-facing camera
    
      FL = hardwareMap.dcMotor.get("FL");
      BR = hardwareMap.dcMotor.get("BR");
      FR = hardwareMap.dcMotor.get("FR");
      BL = hardwareMap.dcMotor.get("BL");
      int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
      
        // Connect to the camera
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.setPipeline(detector);

                
                phoneCam.startStreaming(width, height, OpenCvCameraRotation.SIDEWAYS_LEFT);

                // phoneCam.setFlashlightEnabled(true);
                
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });


        // Use the SkystoneDetector pipeline
        // processFrame() will be called to process the frame
        
        // Remember to change the camera rotation

        
        waitForStart();
        sleep(1500);
        telemetry.addData("before Y",detector.getPostitionY());
        telemetry.update();
        first_valY=detector.getPostitionY();
        sleep(100);
        second_valY=detector.getPostitionY();
        sleep(100);
        third_valY=detector.getPostitionY();
        sleep(100);
        fourth_valY=detector.getPostitionY();
        sleep(100);
        fifth_valY=detector.getPostitionY();
        sleep(1000);
        final_valY=((first_valY+second_valY+third_valY+fourth_valY+fifth_valY)/5);
        telemetry.addData("FinalY:",final_valY);
        telemetry.update();
        beforeY=final_valY;
        ReadWriteFile.writeFile(TargetPositionY,String.valueOf(final_valY));
        sleep(4000);
        first_Area=detector.getArea();
        sleep(100);
        second_Area=detector.getArea();
        sleep(100);
        third_Area=detector.getArea();
        sleep(100);
        fourth_Area=detector.getArea();
        sleep(100);
        fifth_Area=detector.getArea();
        sleep(1000);
        final_Area=((first_Area+second_Area+third_Area+fourth_Area+fifth_Area)/5);
        telemetry.addData("Final Area:",final_Area);
        telemetry.update();
        after_Area=final_Area;
        ReadWriteFile.writeFile(TargetPositionY,String.valueOf(final_valY));
        sleep(4000);
        ReadWriteFile.writeFile(TargetPositionArea, String.valueOf(final_Area));
        sleep(3000);
        //X
        // sleep(24000);
        // telemetry.addData("before X",detector.getPostitionX());
        // telemetry.update();
        // first_valX=detector.getPostitionX();
        // sleep(100);
        // second_valX=detector.getPostitionX();
        // sleep(100);
        // third_valX=detector.getPostitionX();
        // sleep(100);
        // fourth_valX=detector.getPostitionX();
        // sleep(100);
        // fifth_valX=detector.getPostitionX();
        // sleep(1000);
        // final_valX=((first_valX+second_valX+third_valX+fourth_valX+fifth_valX)/5);
        // telemetry.addData("FinalX:",final_valX);
        // telemetry.update();
        // beforeX=final_valX;
        // sleep(7800);
        // telemetry.addData("Go:","24 inches");
        // telemetry.addData("after X",detector.getPostitionX());
        // telemetry.update();
        // sleep(18599);
        // telemetry.addData("After X",detector.getPostitionX());
        // telemetry.update();
        // first_valX=detector.getPostitionX();
        // sleep(100);
        // second_valX=detector.getPostitionX();
        // sleep(100);
        // third_valX=detector.getPostitionX();
        // sleep(100);
        // fourth_valX=detector.getPostitionX();
        // sleep(100);
        // fifth_valX=detector.getPostitionX();
        // sleep(1000);
        // final_valY=((first_valX+second_valX+third_valX+fourth_valX+fifth_valX)/5);
        // telemetry.addData("FinalX:",final_valX);
        // telemetry.update();
        // afterX=final_valX;
        // sleep(3000);
        // differnceX=afterX-beforeX;
        // CmPerInchX =differnceX/ cmX;
        // telemetry.addData("Cm/InchX:", CmPerInchX);
        // telemetry.update();
        // ReadWriteFile.writeFile(CmperInchX, String.valueOf(CmPerInchX));
        // sleep(3000);

//        telemetry.readMessage
//        sleep(500);
//        phoneCam.stopStreaming();
//        beforeX=detector.PostitionX();
//        beforeY=detector.PostitionY();
//        telemetry.addData("X",beforeX);
//        telemetry.addData("Y",beforeY);
//        telemetry.update();
//        sleep(1000);
//        telemetry.addData("Go: ","24 inches");
//        telemetry.update();
//        sleep(3000);
//        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
//        phoneCam.openCameraDevice();
//        // Use the SkystoneDetector pipeline
//        // processFrame() will be called to process the frame
//        phoneCam.setPipeline(detector);
//        // Remember to change the camera rotation
//
//        phoneCam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);
//        sleep(4000);
//        phoneCam.stopStreaming();
//        afterY=detector.PostitionY();
//        afterX=detector.PostitionX();


        //...

//        SkystoneDetector.SkystoneLocation location = detector.getLocation();
//        if (location != SkystoneDetector.SkystoneLocation.NONE) {
//            // Move to the left / right
//        } else {
//            // Grab the skystone
//        }

        // more robot logic...
    }

}
