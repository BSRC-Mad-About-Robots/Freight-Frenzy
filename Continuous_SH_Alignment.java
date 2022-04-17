package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;
import android.os.Environment;
import java.io.FileOutputStream;
import java.io.IOException;
import static android.graphics.Bitmap.createBitmap;
import static android.graphics.Bitmap.createScaledBitmap;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Point;

import java.io.File;

@Autonomous(name="Contniuos_SH_Alignment")
public class Continuous_SH_Alignment extends LinearOpMode {
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
    DcMotor Parl;
    DcMotor Bask;
    BNO055IMU imu;
    NormalizedColorSensor colorSensor;
    double TargetPositionY=0;
    double TargetPositionArea=0;
    double Pixelspeed=0;
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
    double Inchestomove=0;
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
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
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
    private void nudgeToAngle(double target_angle)
        {
            Orientation angles;
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double current_angle = angles.firstAngle; //Because REV Hub is upside down

            FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            
            
            if (target_angle < current_angle)
            {
                FL.setDirection(DcMotorSimple.Direction.REVERSE);
                FR.setDirection(DcMotorSimple.Direction.REVERSE);
                BL.setDirection(DcMotorSimple.Direction.REVERSE);
                BR.setDirection(DcMotorSimple.Direction.REVERSE);
              
            }
            else
            {
                FL.setDirection(DcMotorSimple.Direction.FORWARD);
                FR.setDirection(DcMotorSimple.Direction.FORWARD);
                BL.setDirection(DcMotorSimple.Direction.FORWARD);
                BR.setDirection(DcMotorSimple.Direction.FORWARD);
            }

            setWheelbasePower(0.2);
            while (Math.abs(target_angle - current_angle) > 0.5)
            {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                current_angle = angles.firstAngle; //Because REV Hub is upside down
            }
            setWheelbasePower(0);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            current_angle = angles.firstAngle; //Because REV Hub is upside down
            telemetry.addData("IMU: ", "FINAL HEADING %f", current_angle);
            telemetry.update();
            
            sleep(100);
        }
    // File CmperInchY = AppUtil.getInstance().getSettingsFile("Pixels/Inch.txt");
    // File CmperInchX = AppUtil.getInstance().getSettingsFile("Cm/InchX.txt");
    // File star_val=AppUtil.getInstance().getSettingsFile("Start_val");
    // File TargetPositionYFile=AppUtil.getInstance().getSettingsFile("TargetPositionY.txt");
    // File TargetPositionAreaFile=AppUtil.getInstance().getSettingsFile("TargetPositionArea.txt");
    // File Areabyinch = AppUtil.getInstance().getSettingsFile("Areabyinch.txt");
    // double TargetPositionArea1=Double.parseDouble(ReadWriteFile.readFile(TargetPositionAreaFile).trim());
    // double AreabyInch=Double.parseDouble(ReadWriteFile.readFile(Areabyinch).trim());
    double currentarea=0;
    double inchestogo=0;
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
      Parl = hardwareMap.dcMotor.get("Parl");
      Bask = hardwareMap.dcMotor.get("Baskq");
    //   colorSensor = hardwareMap.dcMotor.get("sensor_color");
      BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        parameters.loggingTag          = "IMU";
        // parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = (BNO055IMU) hardwareMap.get("imu");
        imu.initialize(parameters);
      int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
      
        // Connect to the camera
        
        
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.setPipeline(detector);

                
                phoneCam.startStreaming(1280, 960, OpenCvCameraRotation.SIDEWAYS_LEFT);
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

        
        // waitForStart();
        // sleep(1000);
        // FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // FL.setDirection(DcMotorSimple.Direction.REVERSE);
        // FR.setDirection(DcMotorSimple.Direction.REVERSE);
        // BL.setDirection(DcMotorSimple.Direction.REVERSE);
        // BR.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // FL.setPower(0.15);
        // FR.setPower(0.15);
        // BL.setPower(0.15);
        // BR.setPower(0.15);
        // while(detector.getPostitionY()>=887||detector.getPostitionY()==0){
        //     telemetry.addData("FinalY:",detector.getPostitionY());
        //     telemetry.update();
        // }
        // setWheelbasePower(0.0);
                Parl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            waitForStart();
                Parl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Parl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Parl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Parl.setMode(DcMotor.RunMode.RUN_TO_POSITION); 
                Parl.setDirection(DcMotorSimple.Direction.REVERSE);
                Parl.setTargetPosition((int)3390);
                Parl.setPower(1.0);
          while(Parl.isBusy())
     {}
                            Parl.setPower(0.0);


            // sleep(100);
            int iteration = 0;
            boolean check = false;
    while(true){
        if (check) sleep(5100);
        first_valY=detector.getArea();
        // sleep(67);
        second_valY=detector.getArea();
        // sleep(67);
        third_valY=detector.getArea();
        // sleep(67);
        fourth_valY=detector.getArea();
        // sleep(67);
        fifth_valY=detector.getArea();
        final_valY=((first_valY+second_valY+third_valY+fourth_valY+fifth_valY)/5.0);
        double currentarea=final_valY;
        first_valY=detector.getPostitionY();
        // sleep(67);
        second_valY=detector.getPostitionY();
        // sleep(67);
        third_valY=detector.getPostitionY();
        // sleep(67);
        fourth_valY=detector.getPostitionY();
        // sleep(67);
        fifth_valY=detector.getPostitionY();
        final_valY=((first_valY+second_valY+third_valY+fourth_valY+fifth_valY)/5);
        telemetry.addData("FinalY:",final_valY);
        // TargetPositionY = Double.parseDouble(ReadWriteFile.readFile(TargetPositionYFile).trim());
        
        // Pixelspeed=Double.parseDouble(ReadWriteFile.readFile(CmperInchY).trim());
        
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       
        //run using encoder
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        if (iteration == 2)
        {
                    FL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        

        }
        FL.setPower(0.14);
        FR.setPower(0.14);
        BL.setPower(0.14);
        BR.setPower(0.14);
        while((iteration < 2 && detector.getPostitionY()>=1000) || (iteration == 2 && detector.getPostitionY() <= 887) ||detector.getPostitionY()==0)
        {
            telemetry.addData("FinalY:",detector.getPostitionY());
            telemetry.update();
            currentarea=detector.getArea();
        }
        setWheelbasePower(0.0);
        sleep(1200);
        currentarea=detector.getArea();
            inchestogo= Math.abs((397-currentarea)/13.69);
        telemetry.addData("AreabyInch",inchestogo);
        telemetry.addData("AreabyInch",currentarea);
        telemetry.update();
        
        sleep(400);
        
        if(currentarea<400){
        
        // nudgeToAngle(0.0);
        // sleep(10000);
        Premov();
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE );
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        directionBack();

                FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);

        setTargetPosition((int)(inchestogo*79));
        // setTargetPosition((int)(adjacent*79));
       while(FL.isBusy())
       {
            FL.setPower(1.0/4);
            FR.setPower(1.0/4);
            BL.setPower(1.0/4);
            BR.setPower(1.0/4);
        }
           
        // telemetry.addData("angle", angle);
        // telemetry.addData("newPow", newPow);
        // telemetry.update();
                // setTargetPosition((int)((Math.sqrt(inchestogo*inchestogo+adjacent*adjacent))*79));

       }
        
        
        
        setWheelbasePower(0.0);
        sleep(270);
        check = true;
        // nudgeToAngle(0.0);
        iteration++;
        }
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
        // TargetPositionY = Double.parseDouble(ReadWriteFile.readFile(TargetPositionYFile).trim());
        
        // Pixelspeed=Double.parseDouble(ReadWriteFile.readFile(CmperInchY).trim());
        
        

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
