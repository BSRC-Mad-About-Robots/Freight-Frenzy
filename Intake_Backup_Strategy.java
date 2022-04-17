package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous

public class Intake_Backup_Strategy extends LinearOpMode
{
    String location;
    SkystoneDetector detector = new SkystoneDetector(telemetry);
    OpenCvCamera phoneCam;
    BNO055IMU imu;
    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;
    DcMotor CarL;
    DcMotor CarR;
    DcMotor Parl;
    DcMotor Bask;
    NormalizedColorSensor colorSensor;
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
                telemetry.addData("IMU: ", "current %f, target %f", current_angle, target_angle);
                telemetry.update();
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
        private void turnToAngle2(double target_angle)
    {
        Orientation angles;
        double angle_to_turn = target_angle;
        double current_angle;// = imu.readCurrentHeading();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        current_angle = angles.firstAngle; //Because REV Hub is upside down
        
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        setWheelbasePower(0.5);
    
        while (Math.abs(target_angle - current_angle) > 0.5)
        {
            telemetry.addData("IMU: ", "current %f, target %f", current_angle, target_angle);
            telemetry.update();
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            current_angle = angles.firstAngle; //Because REV Hub is upside down
            if (Math.abs(target_angle - current_angle) < 45)
                setWheelbasePower(0.2);
            else if (Math.abs(target_angle - current_angle) < 30)
                setWheelbasePower(0.1);
        }
        setWheelbasePower(0);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        current_angle = angles.firstAngle; //Because REV Hub is upside down
        telemetry.addData("IMU: ", "FINAL HEADING %f", current_angle);
        telemetry.update();
    }
    private void waitUntilMotorsBusy()
    {
        while ((FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy())) 
            {
                // telemetry.addData("Autonomous",  "Motors busy");
                // telemetry.update();
                // Display it for the driver.
                // 
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
 
    public void  MoveForward(double inches,double pow,double difference)
    {
      FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      Premov();
      FL.setDirection(DcMotorSimple.Direction.REVERSE);
      FR.setDirection(DcMotorSimple.Direction.FORWARD);
      BL.setDirection(DcMotorSimple.Direction.REVERSE);
      BR.setDirection(DcMotorSimple.Direction.FORWARD);
      setTargetPosition((int)inches*79);
      FL.setPower(pow-difference);
      FR.setPower(pow);
      BL.setPower(pow);
      BR.setPower(pow-difference);
      telemetry.addData("how?","idk5.0");
        telemetry.update();
      waitUntilMotorsBusy();
      setWheelbasePower(0.0);
    }
    public void  MoveBack(double inches,double pow)
    {
      FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      Premov();
      FL.setDirection(DcMotor.Direction.FORWARD);
      FR.setDirection(DcMotor.Direction.REVERSE);
      BL.setDirection(DcMotor.Direction.FORWARD);
      BR.setDirection(DcMotor.Direction.REVERSE);
      setTargetPosition((int)inches*79);
      FL.setPower(pow);
      FR.setPower(pow);
      BL.setPower(pow);
      BR.setPower(pow);
      waitUntilMotorsBusy();
      setWheelbasePower(0.0);
    }
    public void  MoveLeft(double inches,double pow, double difference, int parl_var)
    {
      FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      Premov();
      FL.setDirection(DcMotorSimple.Direction.FORWARD);
      FR.setDirection(DcMotorSimple.Direction.FORWARD);
      BL.setDirection(DcMotorSimple.Direction.REVERSE);
      BR.setDirection(DcMotorSimple.Direction.REVERSE);
      setTargetPosition((int)inches*79);
      FR.setPower(pow);
        FL.setPower(pow-difference);
        BR.setPower(pow-difference);
        BL.setPower(pow); 
      while((FR.isBusy()&&BL.isBusy()))
      {
          telemetry.addData("FR position",FR.getCurrentPosition());
        telemetry.addData("BR position",BR.getCurrentPosition());
                  telemetry.addData("FL position",FL.getCurrentPosition());
          telemetry.addData("BL position",BL.getCurrentPosition());

          telemetry.update();
          if(parl_var>0)
          {
              if(parl_var<=Parl.getCurrentPosition())
              {
                  Parl.setPower(0.0);
                  
              }
          }
      }
            setWheelbasePower(0.0);

    //   sleep(200);
    
    
    }
    public  void MoveClockwise(double inches,double pow) 
    {
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Premov();
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        setTargetPosition((int)inches*79);
        FL.setPower(pow);
        FR.setPower(pow);
        BL.setPower(pow);
        BR.setPower(pow);
        waitUntilMotorsBusy();
        setWheelbasePower(0.0);
    }
    //go clockwise
    public void MoveAnticlockwise(double inches,double pow)
    {
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Premov();
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        setTargetPosition((int)inches*79);
        FL.setPower(pow);
        FR.setPower(pow);
        BL.setPower(pow);
        BR.setPower(pow);
        waitUntilMotorsBusy();
        setWheelbasePower(0.0);
    }
    //go left
    public void  MoveRight(double inches,double pow, double difference,int parl_var)
    {
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Premov();
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        setTargetPosition((int)inches*79);
        FR.setPower(pow-difference);
        FL.setPower(pow);
        BR.setPower(pow);
        BL.setPower(pow-difference); 
        while((BR.isBusy()&&FL.isBusy()))
      {
          telemetry.addData("FR position",FR.getCurrentPosition());
        telemetry.addData("BR position",BR.getCurrentPosition());
                  telemetry.addData("FL position",FL.getCurrentPosition());
          telemetry.addData("BL position",BL.getCurrentPosition());

          telemetry.update();
          if(parl_var>0)
          {
              if(parl_var<=Parl.getCurrentPosition())
              {
                  Parl.setPower(0.0);
                  
              }
          }
      }
        // waitUntilMotorsBusy();
        setWheelbasePower(0.0);
        
    }
    //set target position
    public  void  setTargetPosition(int pos)
    {
        FL.setTargetPosition(pos);
        FR.setTargetPosition(pos);
        BL.setTargetPosition(pos);
        BR.setTargetPosition(pos);
    }
    public void MoveCarL(double inches)
    {
        CarL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        CarL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        CarL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        CarL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        CarL.setDirection(DcMotorSimple.Direction.FORWARD);
        CarL.setTargetPosition((int)(inches*79));
        CarL.setPower(1.0);
        while (opModeIsActive() && (CarL.isBusy()))
        {
                
        }
        CarL.setPower(0.0);
        
        
        
    }
    public void MoveCarR(double inches)
    {
        CarR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        CarR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        CarR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        CarR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        CarR.setDirection(DcMotorSimple.Direction.REVERSE);
        CarR.setTargetPosition((int)inches*79);
        CarR.setPower(1.0);
        while (opModeIsActive() && (CarR.isBusy()))
        {
                
        }
        CarR.setPower(0.0);
        
        
        
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
    public void  MoveForwardACDC(double inches,double pow,double difference,double divide)
    {
      FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      Premov();
      FL.setDirection(DcMotorSimple.Direction.REVERSE);
      FR.setDirection(DcMotorSimple.Direction.FORWARD);
      BL.setDirection(DcMotorSimple.Direction.REVERSE);
      BR.setDirection(DcMotorSimple.Direction.FORWARD);
      setTargetPosition((int)inches*79);
      FL.setPower(pow-difference);
      FR.setPower(pow);
      BL.setPower(pow);
      BR.setPower(pow-difference);
    while(FR.isBusy() && BL.isBusy()&&FL.isBusy()&&BR.isBusy())
      {
      if(FL.getCurrentPosition()>(inches*79*0.8))
      {
      FL.setPower((pow-difference)/divide);
      FR.setPower(pow/divide);
      BL.setPower(pow/divide);
      BR.setPower((pow-difference)/divide); }}      
      setWheelbasePower(0.0);
    }

    @Override 
    public void runOpMode()
    {
                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        parameters.loggingTag          = "IMU";
        // parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = (BNO055IMU) hardwareMap.get("imu");
        imu.initialize(parameters);
        // https://github.com/OpenFTC/EasyOpenCV/blob/master/examples/src/main/java/org/openftc/easyopencv/examples/InternalCameraExample.java
        // Initialize the back-facing camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam =  OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        // Connect to the camera
        // phoneCam.openCameraDevice();
        // Use the SkystoneDetector pipeline
        // processFrame() will be called to process the frame
        phoneCam.setPipeline(detector);
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                
                phoneCam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        // Remember to change the camera rotation
        // phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener());

        //...
        sleep(5000);
        String location="";

              FL = hardwareMap.dcMotor.get("FL");
      BR = hardwareMap.dcMotor.get("BR");
      FR = hardwareMap.dcMotor.get("FR");
      BL = hardwareMap.dcMotor.get("BL");
      CarL = hardwareMap.dcMotor.get("CarL");
      CarR = hardwareMap.dcMotor.get("CarR");
      Parl=hardwareMap.dcMotor.get("Parl");
      Bask=hardwareMap.dcMotor.get("Baskq");
      Parl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                  colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

      FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      int sleep=3100;
               double diagnal=0.0;
            //   double power=0.79;
              double forward=0.0;
              telemetry.addData("Ready",true);
              telemetry.update();
      waitForStart();
    //   sleep(500)
      location=detector.getPostitionCapstone();
        telemetry.addData("Location",location);
        telemetry.update();
      int left=0;
      int right=0;
      int middle=0;
      int i=0;
      while(i<=5)
      {
          location=detector.getPostitionCapstone();
        telemetry.addData("Location",location);
        telemetry.update();
        if(location=="left")
        {
            left++;
        }
        else if(location=="right")
        {
            right++;
        }
        else if(location=="middle")
        {
            middle++;
        }
        i++;
      }
      location=detector.getPostitionCapstone();
        telemetry.addData("Location final",location);
        location=detector.getPostitionCapstone();
        telemetry.addData("middle",middle);
        telemetry.addData("right",right);
        telemetry.addData("bottom",left);


        telemetry.update();
        telemetry.update();
      if(left>right && left>middle)
      {
          location="left";
      }
      else if(right>left && right>middle)
      {
          location="right";
      }
      else if(middle>left && middle>right){
          location="middle";
      }
      else
      {
          location="right";
      }
    //   location = "middle";
      switch (location)
      {
          case "left":
          {
              sleep=1300;
              forward=0.1;
              break;
          }
          case "middle":
          {
              sleep=2275;
              forward=0.1;
              break;
          } 
          case "right":
          {
              sleep=3210;
              forward=2.55;
          }
          case "None":
             {
              sleep=3210;
              forward=2.55;
          } 
      }      //MoveBack(1,0.15);
    //   MoveCarR(40.0);
      
                
Parl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Parl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Parl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Parl.setDirection(DcMotorSimple.Direction.REVERSE);
                Parl.setTargetPosition((int)sleep);
                Parl.setPower(1.0);
                sleep(120);
    MoveRight(24,1.0,0.89,sleep);
    
    
     sleep(30);
    nudgeToAngle(0.0);
    sleep(10);
      MoveForward(forward,1.0,0.0);
      
      sleep(60);
      Bask.setPower(0.85);
      while (colorSensor instanceof DistanceSensor && ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM)<3.5) 
                {}
                sleep(100);
              Bask.setPower(0.15);  
                
      MoveBack(forward+0.23,1.0);
      Bask.setPower(0.0);
      forward = 2.99;
      sleep(15);
      Parl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Parl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Parl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Parl.setDirection(DcMotorSimple.Direction.REVERSE);
                Parl.setTargetPosition(-(int)sleep);
                Parl.setPower(1.0);
                sleep(70);
      FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Premov();
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("Sweeping", true);
        telemetry.update();
        setTargetPosition(-(int)33*79);
        FL.setPower(1.0);
        FR.setPower(1.0);
        BL.setPower(0.258);
        BR.setPower(0.258);
        while(FL.isBusy()&&BL.isBusy()&&BR.isBusy()&&FR.isBusy())
        {
            if(!(Parl.isBusy()))
            {
                Parl.setPower(0.0);
                
            }
        }
        setWheelbasePower(0.0); 
        
      sleep(20);
            nudgeToAngle(91.8);
            sleep(30);
            Bask.setPower(-1.0);
            MoveForward(22.5,1.0,0.0);
            // nudgeToAngle(-92);
                  FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      Premov();
      FL.setDirection(DcMotorSimple.Direction.REVERSE);
      FR.setDirection(DcMotorSimple.Direction.FORWARD);
      BL.setDirection(DcMotorSimple.Direction.REVERSE);
      BR.setDirection(DcMotorSimple.Direction.FORWARD);
      setTargetPosition(8*79);
      FL.setPower(0.3);
      FR.setPower(0.3);
      BL.setPower(0.3);
      BR.setPower(0.3);
             int ticks = 0;
            while (colorSensor instanceof DistanceSensor && ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM)>2.0) 
            {
                ticks = FL.getCurrentPosition();
            }
            ticks = FL.getCurrentPosition();
            setWheelbasePower(0.0);
            Premov();
            Bask.setPower(-0.2);
            sleep(15);
    // sleep(100);
    // // //   sleep(3000);
                nudgeToAngle(86);

      MoveBack(18.1 + ticks/79,1.0);
      Bask.setPower(0.0);

      sleep(40);
      Parl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Parl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Parl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Parl.setDirection(DcMotorSimple.Direction.REVERSE);
                Parl.setTargetPosition((int)3290);
                Parl.setPower(1.0);
                sleep(70);
      FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Premov();
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        setTargetPosition((int)33.5*79);
        FL.setPower(1.0);
        FR.setPower(1.0);
        BL.setPower(0.258);
        BR.setPower(0.258);
        while(FL.isBusy()&&BL.isBusy()&&BR.isBusy()&&FR.isBusy())
        {
            if(!(Parl.isBusy()))
            {
                Parl.setPower(0.0);
                
            }
        }
        setWheelbasePower(0.0); 
        sleep(15);
        nudgeToAngle(0.0);
        sleep(15);
                

              MoveForward(forward+0.94,1.0,0.0);
              sleep(30);
              Bask.setPower(1.0);
      while (colorSensor instanceof DistanceSensor && ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM)<3.9) 
                {}
                sleep(140);
                

    }
}