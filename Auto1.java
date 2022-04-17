// package org.firstinspires.ftc.teamcode.Autonomous;

// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
// import java.lang.annotation.Target;
// import com.qualcomm.robotcore.util.ElapsedTime;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.util.ReadWriteFile;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
// import com.qualcomm.robotcore.util.ElapsedTime;
// import com.qualcomm.robotcore.hardware.CRServo;
// import com.qualcomm.robotcore.hardware.Servo;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.eventloop.opmode.OpMode;
// import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// import org.openftc.easyopencv.OpenCvCamera;
// import org.openftc.easyopencv.OpenCvCameraFactory;
// import org.openftc.easyopencv.OpenCvCameraRotation;
// import org.openftc.easyopencv.OpenCvInternalCamera;
// import org.openftc.easyopencv.OpenCvPipeline;

// import java.io.File;

// @Autonomous(name="Auto1",group="Autonomous")

// public class Auto1 extends LinearOpMode
// {
//     DcMotor FL;
//     DcMotor FR;
//     DcMotor BL;
//     DcMotor BR;
//     DcMotor CarL;
//     DcMotor CarR;
//     SkystoneDetector detector = new SkystoneDetector(telemetry);
//     OpenCvCamera phoneCam;

    
//     private void waitUntilMotorsBusy()
//     {
//         while (opModeIsActive() && (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy())) 
//             {
//                 // Display it for the driver.
//                 // telemetry.addData("Autonomous",  "Motors busy");
//                 // telemetry.update();
//             }
//     }
    
    
//     //basic methods 
//     public void setWheelbasePower(double pow) 
//     {
//         FL.setPower(pow);
//         FR.setPower(pow);
//         BL.setPower(pow);
//         BR.setPower(pow);
//     }
 
//     public void  MoveForward(double inches,double pow,double difference)
//     {
//       FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//       FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//       BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//       BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//       Premov();
//       FL.setDirection(DcMotorSimple.Direction.REVERSE);
//       FR.setDirection(DcMotorSimple.Direction.FORWARD);
//       BL.setDirection(DcMotorSimple.Direction.REVERSE);
//       BR.setDirection(DcMotorSimple.Direction.FORWARD);
//       setTargetPosition((int)inches*79);
//       FL.setPower(pow-difference);
//       FR.setPower(pow);
//       BL.setPower(pow);
//       BR.setPower(pow-difference);
//       waitUntilMotorsBusy();
//       setWheelbasePower(0.0);
//     }
//     public void  MoveBack(double inches,double pow)
//     {
//       FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//       FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//       BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//       BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//       Premov();
//       FL.setDirection(DcMotor.Direction.FORWARD);
//       FR.setDirection(DcMotor.Direction.REVERSE);
//       BL.setDirection(DcMotor.Direction.FORWARD);
//       BR.setDirection(DcMotor.Direction.REVERSE);
//       setTargetPosition((int)inches*79);
//       FL.setPower(pow);
//       FR.setPower(pow);
//       BL.setPower(pow);
//       BR.setPower(pow);
//       waitUntilMotorsBusy();
//       setWheelbasePower(0.0);
//     }
//     public void  MoveLeft(double inches,double pow, double difference)
//     {
//       FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//       FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//       BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//       BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//       Premov();
//       FL.setDirection(DcMotorSimple.Direction.FORWARD);
//       FR.setDirection(DcMotorSimple.Direction.FORWARD);
//       BL.setDirection(DcMotorSimple.Direction.REVERSE);
//       BR.setDirection(DcMotorSimple.Direction.REVERSE);
//       setTargetPosition((int)inches*79);
//       FL.setPower(pow-difference);
//       FR.setPower(pow);
//       BL.setPower(pow);
//       BR.setPower(pow-difference); 
//       waitUntilMotorsBusy();
//       setWheelbasePower(0.0);
      
//     }
//     public  void MoveClockwise(double inches,double pow) 
//     {
//         FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         Premov();
//         FL.setDirection(DcMotorSimple.Direction.REVERSE);
//         FR.setDirection(DcMotorSimple.Direction.REVERSE);
//         BL.setDirection(DcMotorSimple.Direction.REVERSE);
//         BR.setDirection(DcMotorSimple.Direction.REVERSE);
//         setTargetPosition((int)inches*79);
//         FL.setPower(pow);
//         FR.setPower(pow);
//         BL.setPower(pow);
//         BR.setPower(pow);
//         waitUntilMotorsBusy();
//         setWheelbasePower(0.0);
//     }
//     //go clockwise
//     public void MoveAnticlockwise(double inches,double pow)
//     {
//         FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         Premov();
//         FL.setDirection(DcMotorSimple.Direction.FORWARD);
//         FR.setDirection(DcMotorSimple.Direction.FORWARD);
//         BL.setDirection(DcMotorSimple.Direction.FORWARD);
//         BR.setDirection(DcMotorSimple.Direction.FORWARD);
//         setTargetPosition((int)inches*79);
//         FL.setPower(pow);
//         FR.setPower(pow);
//         BL.setPower(pow);
//         BR.setPower(pow);
//         waitUntilMotorsBusy();
//         setWheelbasePower(0.0);
//     }
//     //go left
//     public void  MoveRight(double inches,double pow, double difference)
//     {
//         FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         Premov();
//         FL.setDirection(DcMotorSimple.Direction.REVERSE);
//         FR.setDirection(DcMotorSimple.Direction.REVERSE);
//         BL.setDirection(DcMotorSimple.Direction.FORWARD);
//         BR.setDirection(DcMotorSimple.Direction.FORWARD);
//         setTargetPosition((int)inches*79);
//         FR.setPower(pow);
//         FL.setPower(pow-difference);
//         BR.setPower(pow-difference);
//         BL.setPower(pow); 
//         waitUntilMotorsBusy();
//         setWheelbasePower(0.0);
        
//     }
//     //set target position
//     public  void  setTargetPosition(int pos)
//     {
//         FL.setTargetPosition(pos);
//         FR.setTargetPosition(pos);
//         BL.setTargetPosition(pos);
//         BR.setTargetPosition(pos);
//     }
//     public void MoveCarL(double inches)
//     {
//         CarL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         CarL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         CarL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//         CarL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//         CarL.setDirection(DcMotorSimple.Direction.FORWARD);
//         CarL.setTargetPosition((int)(inches*79));
//         CarL.setPower(0.5);
//         while (opModeIsActive() && (CarL.isBusy()))
//         {
                
//         }
//         CarL.setPower(0.0);
        
        
        
//     }
//     public void MoveCarR(double inches)
//     {
//         CarR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         CarR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         CarR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//         CarR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//         CarR.setDirection(DcMotorSimple.Direction.REVERSE);
//         CarR.setTargetPosition((int)inches*79);
//         CarR.setPower(0.7);
//         while (opModeIsActive() && (CarR.isBusy()))
//         {
                
//         }
//         CarR.setPower(0.0);
        
        
        
//     }
    
//     //to minimize repitition
//     public void Premov()
//     {
//         //to stop and reset 
//         FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       
//         //run using encoder
//         FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//         FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//         BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//         BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
//         //run to position
//         FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//         FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//         BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//         BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//     }
//       @Override
//     public void runOpMode()
//     {
//       int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//       phoneCam =  OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//       phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//         {
//             @Override
//             public void onOpened()
//             {
                
//                 phoneCam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
//             }

//             @Override
//             public void onError(int errorCode)
//             {
//                 /*
//                  * This will be called if the camera could not be opened
//                  */
//             }
//         });
//         // Remember to change the camera rotation
//         // phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener());

//         //...
//         sleep(2000);
//         String location=detector.getPostitionCapstone();
//         telemetry.addData("Location",location);
//         telemetry.update();
        
//       FL = hardwareMap.dcMotor.get("FL");
//       BR = hardwareMap.dcMotor.get("BR");
//       FR = hardwareMap.dcMotor.get("FR");
//       BL = hardwareMap.dcMotor.get("BL");
//       CarL = hardwareMap.dcMotor.get("CarL");
//       CarR = hardwareMap.dcMotor.get("CarR");
      
//       FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//       FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//       BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//       BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//       waitForStart();
//       //MoveBack(1,0.15);
//     //   MoveCarR(40.0);
//       MoveLeft(29.1,1.0,0.75);
//       sleep(3000);
//       MoveBack(5.2,0.6);
//       sleep(300);
//       MoveRight(30.6,0.5,0.022);
//       sleep(200);
//     //   MoveBack(1.5,0.09);
//       MoveCarR(24.0);
//       sleep(200);
      
//     //   CarL=
//       MoveForward(10,0.4,-0.3);
  
//     }
    
// }

//     // todo: write your code here
