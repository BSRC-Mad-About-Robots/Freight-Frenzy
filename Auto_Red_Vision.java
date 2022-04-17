package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="Auto: Red Vision")
public class Auto_Red_Vision extends LinearOpMode 
{
    // Handle hardware stuff...

    int width = 320;
    int height = 240;
    BNO055IMU imu;
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
    double inchestogo=0.0;
    // store as variable here so we can access the location
        FinalOpenCV detector = new FinalOpenCV(telemetry);

    // SkystoneDetector detector = new SkystoneDetector(telemetry);
    OpenCvCamera phoneCam;
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
                // telemetry.addData("IMU: ", "current %f, target %f", current_angle, target_angle);
                // telemetry.update();
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                current_angle = angles.firstAngle; //Because REV Hub is upside down
            }
            setWheelbasePower(0);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            current_angle = angles.firstAngle; //Because REV Hub is upside down
            // telemetry.addData("IMU: ", "FINAL HEADING %f", current_angle);
            // telemetry.update();
            
            sleep(20);
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
            // telemetry.addData("IMU: ", "current %f, target %f", current_angle, target_angle);
            // telemetry.update();
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
        // telemetry.addData("IMU: ", "FINAL HEADING %f", current_angle);
        // telemetry.update();
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
        // telemetry.addData("how?","idk4.0");
        // telemetry.update();
      FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      Premov();
    //   telemetry.addData("how?","idk5.0");
    //     telemetry.update();
      FL.setDirection(DcMotorSimple.Direction.REVERSE);
      FR.setDirection(DcMotorSimple.Direction.FORWARD);
      BL.setDirection(DcMotorSimple.Direction.REVERSE);
      BR.setDirection(DcMotorSimple.Direction.FORWARD);
      setTargetPosition((int)inches*79);
    //   telemetry.addData("how?","idk6.0");
        // telemetry.update();
      FL.setPower(pow-difference);
      FR.setPower(pow);
      BL.setPower(pow);
      BR.setPower(pow-difference);
    //   telemetry.addData("how?","idk5.0");
        // telemetry.update();
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
      while((FL.isBusy()&&BL.isBusy()))
      {
        //   telemetry.addData("FR position",FR.getCurrentPosition());
        // telemetry.addData("BR position",BR.getCurrentPosition());
        //           telemetry.addData("FL position",FL.getCurrentPosition());
        //   telemetry.addData("BL position",BL.getCurrentPosition());

        //   telemetry.update();
          if(parl_var>0)
          {
              if(parl_var<=Parl.getCurrentPosition())
              {
                  Parl.setPower(0.0);
                  
              }
          }
          if(FL.getCurrentPosition()>(inches*79*0.72))
      {
      FR.setPower(pow/4);
        FL.setPower((pow-difference)/4);
        BR.setPower((pow-difference)/4);
        BL.setPower(pow/4);  }
          
      
      
            

    //   sleep(200);
    
    
    }
        setWheelbasePower(0.0);
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
        while(FL.isBusy() && BR.isBusy())
      {if(parl_var>0)
          {
              if(parl_var<=Parl.getCurrentPosition())
              {
                  Parl.setPower(0.0);
                  
              }
          }
    if(FL.getCurrentPosition()>(inches*79*0.75))
      {
      FR.setPower((pow-difference)/3);
      FL.setPower(pow/3);
      BR.setPower(pow/3);
      BL.setPower((pow-difference)/3); }
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
    public void  MoveBackACDC(double inches,double pow,double difference,double divide)
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
      FL.setPower(pow-difference);
      FR.setPower(pow);
      BL.setPower(pow);
      BR.setPower(pow-difference);
    while(FR.isBusy() && BL.isBusy()&&FL.isBusy()&&BR.isBusy())
      {
      if(FL.getCurrentPosition()>(inches*79*0.84))
      {
      FL.setPower((pow-difference)/divide);
      FR.setPower(pow/divide);
      BL.setPower(pow/divide);
      BR.setPower((pow-difference)/divide); }}      
      setWheelbasePower(0.0);
    }
   public void AlignmentToSH(){
        while(detector.getArea()==0){}
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
        
        
        inchestogo=((376-currentarea)/13.69);
        double initialinches=inchestogo;
        telemetry.addData("AreabyInch",inchestogo);
        telemetry.addData("AreabyInch",currentarea);
        double adjacent=(900-final_valY)/49;
        telemetry.addData("adjacent",adjacent);
        double angle=Math.abs(Math.toDegrees(Math.atan(inchestogo/adjacent)));
        double hypo=Math.sqrt(inchestogo*inchestogo+adjacent*adjacent);
        telemetry.addData("hypo", Math.sqrt(inchestogo*inchestogo+adjacent*adjacent));
        double newPow=Math.abs((angle-45))/90;
        telemetry.addData("angle", angle);
        telemetry.addData("newPow", newPow);

        telemetry.update();
        
        // sleep(8000);
        
        
        
        // nudgeToAngle(0.0);
        // sleep(10000);
        Premov();
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE );
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        setTargetPosition((int)((Math.sqrt(inchestogo*inchestogo+adjacent*adjacent))*79));
       while(FL.isBusy()){
           if(fifth_valY>850)
           {
               FL.setPower(1.0);
      FR.setPower(newPow);
      BL.setPower(newPow);
      BR.setPower(1.0);
      
           }
           else{
               FL.setPower(newPow);
      FR.setPower(1.0);
      BL.setPower(1.0);
      BR.setPower(newPow);
      if(FR.getCurrentPosition()>=(hypo*79))
      {
          
          break;
      }
           }
           
           
      final_valY=detector.getPostitionY();
      currentarea=detector.getArea();
        inchestogo=((396-currentarea)/12.82);
        // telemetry.addData("AreabyInch",inchestogo);
        // telemetry.addData("AreabyInch",currentarea);
        adjacent=(900-final_valY)/49;
        // telemetry.addData("adjacent",adjacent);
        angle=Math.abs(Math.toDegrees(Math.atan(inchestogo/adjacent)));
        newPow=Math.abs((angle-45))/90;
        // telemetry.addData("angle", angle);
        // telemetry.addData("newPow", newPow);
        // telemetry.update();
                // setTargetPosition((int)((Math.sqrt(inchestogo*inchestogo+adjacent*adjacent))*79));

       }
        
        
        
        setWheelbasePower(0.0);
        sleep(10);
        

        
    }
    
    
    

    @Override
    public void runOpMode() {
        // robot logic...
        detector.AlighnmentStart = false;
        detector.TSE_detection=true;
        detector.red=true;
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
                
                phoneCam.startStreaming(1280, 960, OpenCvCameraRotation.UPRIGHT);
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
    detector.pixelshift=120;
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
      if(left>right
      && left>middle)
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
              sleep=659;
              forward=3.3;
              break;
          }
          case "middle":
          {
              sleep=1525;
              forward=3.49;
              break;
          } 
          case "right":
          {
              sleep=2300;
              forward=4.5;
              break;
          }
          case "None":
             {
              sleep=2300;
              forward=4.5;
          } 
      }      //MoveBack(1,0.15);
    //   MoveCarR(40.0);
    
        detector.AlighnmentStart = true;
        detector.TSE_detection=false;//MoveBack(1,0.15);
    //   MoveCarR(40.0);
      Parl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Parl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Parl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Parl.setDirection(DcMotorSimple.Direction.REVERSE);
                Parl.setTargetPosition((int)sleep);
                Parl.setPower(1.0);
                sleep(100);
                // forward=3;

        MoveRight(25.2,1.0,0.81,sleep);

     sleep(8);
    nudgeToAngle(0.0);
    sleep(8);
    
      MoveForward(forward,0.6,0.0);
      sleep(10);
      Bask.setPower(0.85);
      while (colorSensor instanceof DistanceSensor && ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM)<3.9) 
                {}
      sleep(30);
      
      Bask.setPower(0.0);
      sleep(12);
            // nudgeToAngle(0.5);
    // sleep(8);
    // //   sleep(3000);
    
      MoveBack(forward+3.4,0.8);
      forward=3.3;
      sleep(10);
        // nudgeToAngle(0.5);
    //   sleep(30);
         

    //   sleep(30);
      Parl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Parl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Parl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Parl.setDirection(DcMotorSimple.Direction.REVERSE);
                Parl.setTargetPosition(-(int)sleep);
                Parl.setPower(1.0);
                sleep(100);
      MoveLeft(36,0.7,-0.3,sleep);
            forward = 2.5;
nudgeToAngle(0.0);
    //   waitUntilMotorsBusy();
    //   setWheelbasePower(0.0);    // //   nudgeToAngle(0.0);
    //   sleep(200);
    // //   MoveBack(1.5,0.09);
    MoveCarL(25);
          

      
      sleep(7);
      
      FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Premov();
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        setTargetPosition((int)25*79);
        FL.setPower(0.9);
        FR.setPower(0.9);
        BL.setPower(0.0);
        BR.setPower(0.0);
        waitUntilMotorsBusy();
        setWheelbasePower(0.0);
            nudgeToAngle(-94.5);    
              Bask.setPower(-1.0);

        MoveForwardACDC(62,0.8,-0.2,4);
         
                sleep(20);
            nudgeToAngle(-88);
            // sleep(900);

            // nudgeToAngle(-92);
sleep(20);
                
                
             int ticks = 0;
             
            while (colorSensor instanceof DistanceSensor && ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM)>2.8) 
            {

             
                
            }
            // ticks = FL.getCurrentPosition();
            
            setWheelbasePower(0.0);
            Premov();
            // Bask.setPower(-0.2);
            // sleep(15);
                // Bask.setPower(-0.2);
            // sleep(15);
    sleep(10);
    //   sleep(1000);
             
      MoveBackACDC(22.3+(ticks/79.0),0.88,-0.12,5);
      Parl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Parl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Parl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Parl.setDirection(DcMotorSimple.Direction.REVERSE);
                Parl.setTargetPosition((int)2500);
                Parl.setPower(1.0);
                sleep(100);   
      Bask.setPower(0.0);
    

      sleep(15);
     
      FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Premov();
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        setTargetPosition(-(int)33.5*79);
        FL.setPower(1.0);
        FR.setPower(1.0);
        BL.setPower(0.22);
        BR.setPower(0.22);
        while(FL.isBusy()&&BL.isBusy()&&BR.isBusy()&&FR.isBusy())
        {
            if(!(Parl.isBusy()))
            {
                Parl.setPower(0.0);
                
            }
        }
        setWheelbasePower(0.0); 
        sleep(10);
        nudgeToAngle(0.0);
        // sleep(40);
                

              AlignmentToSH();
              sleep(30);
              Bask.setPower(0.8);  
                      nudgeToAngle(0.0);
Bask.setPower(0.85);
      while (colorSensor instanceof DistanceSensor && ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM)<3.9) 
                {}
      sleep(10);
      
      Bask.setPower(0.0);
MoveBack(forward,1.0);
      
      sleep(10);
      Parl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Parl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Parl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Parl.setDirection(DcMotorSimple.Direction.REVERSE);
                Parl.setTargetPosition(-(int)2500);
                Parl.setPower(1.0);
                // sleep(120);   
      FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Premov();
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        setTargetPosition((int)(33.5*79));
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
            nudgeToAngle(-91.6);
            sleep(20);
            Bask.setPower(-1.0);
            MoveForwardACDC(25.1,0.9,0.1,5);
            sleep(20);
            nudgeToAngle(-87.8);
            
            // nudgeToAngle(-92);

            while (colorSensor instanceof DistanceSensor && ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM)>2.0) 
                {}
                Bask.setPower(-0.2);
            // sleep(15);
    // sleep(100);
    //   sleep(1000);
                
      MoveBackACDC(21.8,0.82,-0.18,5);
      Bask.setPower(0.0);
    

      sleep(30);
      Parl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Parl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Parl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Parl.setDirection(DcMotorSimple.Direction.REVERSE);
                Parl.setTargetPosition((int)2500);
                Parl.setPower(1.0);
                sleep(100);   
      FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Premov();
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        setTargetPosition(-(int)33.5*79);
        FL.setPower(1.0);
        FR.setPower(1.0);
        BL.setPower(0.21);
        BR.setPower(0.21);
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
        // sleep(40);
                

              AlignmentToSH();
              sleep(50);
              Bask.setPower(0.8);  
                      nudgeToAngle(0.0);
      Bask.setPower(0.85);
      while (colorSensor instanceof DistanceSensor && ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM)<3.9) 
                {}
      sleep(50);
      
      Bask.setPower(0.0);
      MoveBack(forward,1.0);
      sleep(15);
      
      FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Premov();
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        setTargetPosition((int)(33.1*79));
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
            nudgeToAngle(-91.6);
            sleep(40);
            Bask.setPower(-1.0);
            MoveForward(21.5,0.9,0.1);
            sleep(20);
            nudgeToAngle(-87.2);
            
    //           diagnal=4.12;
    // //   CarL=                                                  
      
    //   sleep(250);
    //   nudgeToAngle(0.0);
        // waitForStart();22
        

        // more robot logic...
    }

}
