package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.   OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="Auto: Red Shared Vision")
public class Auto_Red_Vision_Shared extends LinearOpMode 
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
            target_angle=-target_angle;
            Orientation angles;
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double current_angle = angles.firstAngle; //Because REV Hub is upside down
            telemetry.addData("currentangle",current_angle);
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
            
            sleep(40);
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
        setWheelbasePower(0.89);
    
        while (Math.abs(target_angle - current_angle) > 0.5)
        {
            telemetry.addData("IMU: ", "current %f, target %f", current_angle, target_angle);
            telemetry.update();
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            current_angle = angles.firstAngle; //Because REV Hub is upside down
            if (Math.abs(target_angle - current_angle) < 65)
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
        telemetry.addData("how?","idk4.0");
        telemetry.update();
      FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      Premov();
      telemetry.addData("how?","idk5.0");
        telemetry.update();
      FL.setDirection(DcMotorSimple.Direction.REVERSE);
      FR.setDirection(DcMotorSimple.Direction.FORWARD);
      BL.setDirection(DcMotorSimple.Direction.REVERSE);
      BR.setDirection(DcMotorSimple.Direction.FORWARD);
      setTargetPosition((int)inches*79);
      telemetry.addData("how?","idk6.0");
        telemetry.update();
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
        sleep(400);
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
        
        // sleep(3000);
        
        
        
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
        inchestogo=((380-currentarea)/13.8);
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
        sleep(30);
        

        
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
              sleep=659;
              forward=0.1;
              break;
          }
          case "middle":
          {
              sleep=1425;
              forward=0.1;
              break;
          } 
          case "right":
          {
              sleep=2500;
              forward=2.55;
          }
          case "None":
             {
              sleep=2500;
              forward=2.55;
          } 
          telemetry.addData("Location final",location);
        location=detector.getPostitionCapstone();
        telemetry.addData("middle",middle);
        telemetry.addData("right",right);
        telemetry.addData("bottom",left);


        telemetry.update();
        telemetry.update();
      }      //MoveBack(1,0.15);
    //   MoveCarR(40.0);
    
        detector.AlighnmentStart = true;
        detector.TSE_detection=false;
                
        Parl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Parl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Parl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Parl.setDirection(DcMotorSimple.Direction.REVERSE);
        Parl.setTargetPosition((int)sleep);
        Parl.setPower(1.0);
        sleep(100);
        
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      Premov();
      FL.setDirection(DcMotorSimple.Direction.REVERSE);
      FR.setDirection(DcMotorSimple.Direction.FORWARD);
      BL.setDirection(DcMotorSimple.Direction.REVERSE);
      BR.setDirection(DcMotorSimple.Direction.FORWARD);
      setTargetPosition((int)27*79);
      FL.setPower(0.6/1.5);
      FR.setPower(1.0/1.5);
      BL.setPower(0.6/1.5);
      BR.setPower(1.0/1.5);
      while(FR.isBusy())
      {
          telemetry.addData("FR", FR.getCurrentPosition());
          telemetry.addData("percent", (double)FR.getCurrentPosition()/(15*79));
        telemetry.addData("target Position", (15*79));
      

      telemetry.update();
          if((double)FR.getCurrentPosition()/(25*79)>=0.45)
          {
          FL.setPower(0.3/2);
      FR.setPower(1.0/2);
      BL.setPower(0.3/2);
      BR.setPower(1.0/2);
      
      }}
      setWheelbasePower(0.0);
      Bask.setPower(1.0);
      sleep(1000);
      
      FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      Premov();
      FL.setDirection(DcMotorSimple.Direction.REVERSE);
      FR.setDirection(DcMotorSimple.Direction.FORWARD);
      BL.setDirection(DcMotorSimple.Direction.REVERSE);
      BR.setDirection(DcMotorSimple.Direction.FORWARD);
      setTargetPosition(-(int)24*79);
      FL.setPower(0.3/2);
      FR.setPower(1.0/2);
      BL.setPower(0.3/2);
      BR.setPower(1.0/2);
      while(FR.isBusy())
      {
          telemetry.addData("FR", FR.getCurrentPosition());
          telemetry.addData("percent", (double)FR.getCurrentPosition()/(15*79));
          telemetry.addData("target Position", (15*79));
          telemetry.update();
          if((double)FR.getCurrentPosition()/(23*79)>=0.55)
          {
          FL.setPower(0.6/2);
      FR.setPower(1.0/2);
      BL.setPower(0.6/2);
      BR.setPower(1.0/2);
      
      }}
      setWheelbasePower(0.0);
      sleep(40);
      Parl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Parl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Parl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Parl.setDirection(DcMotorSimple.Direction.REVERSE);
        Parl.setTargetPosition(-(int)sleep);
        Parl.setPower(1.0);
        sleep(100);
      turnToAngle2(-90);
      sleep(100);
      FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Premov();
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
      FR.setDirection(DcMotorSimple.Direction.FORWARD);
      BL.setDirection(DcMotorSimple.Direction.REVERSE);
      BR.setDirection(DcMotorSimple.Direction.FORWARD);
        setTargetPosition((int)23*79);
        FR.setPower(0.01);
        FL.setPower(1.0);
        BR.setPower(1.0);
        BL.setPower(0.01); 
        Bask.setPower(-1.0);
        while((BR.isBusy()&&FL.isBusy()))
      {
          if(((FL.getCurrentPosition()/79.0))>=11.1)
          {
              FR.setPower(1.0-0.15);
        FL.setPower(1.0);
        BR.setPower(1.0);
        BL.setPower(1.0-0.15); 
          }
          if(sleep>0)
          {
              if(sleep>=Math.abs(Parl.getCurrentPosition()))
              {
                  Parl.setPower(0.0);
                  
              }
          }
      }
      
    //     // waitUntilMotorsBusy();
        setWheelbasePower(0.0);
        sleep(40);
nudgeToAngle(82);
FL.setDirection(DcMotorSimple.Direction.REVERSE);
      FR.setDirection(DcMotorSimple.Direction.FORWARD);
      BL.setDirection(DcMotorSimple.Direction.REVERSE);
      BR.setDirection(DcMotorSimple.Direction.FORWARD);
FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      Premov();
      setTargetPosition(10*79);
      setWheelbasePower(0.5);
    //   MoveForward(10,0.9,0.1);
          int ticks = 0;
        //   while()
            while (colorSensor instanceof DistanceSensor && ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM)>2) 
            {
                ticks = FL.getCurrentPosition()+ticks;
                if(colorSensor instanceof DistanceSensor && ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM)<3.2){
                    Premov();
                    setTargetPosition(5*79);
                    setWheelbasePower(0.3);
                    if(colorSensor instanceof DistanceSensor && ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM)<2.75&&((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM)>2)
                    {
                        Premov();
                        setTargetPosition(-8*79);
                        setWheelbasePower(0.3);
                        
                    }
                    
                    
                }
                if(colorSensor instanceof DistanceSensor && ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM)>3.42){
                    Premov();
                    setTargetPosition(5*79);
                    setWheelbasePower(0.3);
                }
                telemetry.clear();
                telemetry.addData("dist",((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
                telemetry.update();
            }
                      setWheelbasePower(0.0);
                    sleep(100);
                    while(colorSensor instanceof DistanceSensor && ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM)>2.5)
                    {
                    }
                // if(FR.getCurrentPosition()>=(5*79))
                // {
                //     FR.setTargetPosition(-5*79);
                //     FL.setTargetPosition(-5*79);
                //     BL.setTargetPosition(-5*79);
                //     BR.setTargetPosition(-5*79);


                // }
            
            nudgeToAngle(90.0);
            telemetry.addData("ticks",Math.abs(ticks/79.0));
            telemetry.update();
            if(Math.abs((ticks/79.0))>4.9)
            {
            MoveBack(6.5,0.6);
                
            }
            else
            {
                MoveBack(3,0.6);
            }
            // setWheelBasePower(0.0);
            nudgeToAngle(90.0);
        //     ticks = FL.getCurrentPosition();
        //   ticks = FL.getCurrentPosition();
        //     setWheelbasePower(0.0);
        //     Premov();
        //     Bask.setPower(-1);
        //     sleep(15);
        //     sleep(100);
        //     telemetry.addData("inches",(double)ticks/79);
            telemetry.update();
    
                

      
                
        
        sleep(100);
        MoveLeft(15,1.0,0,0);
        sleep(80);
        
              turnToAngle2(0.0);
              sleep(70);
              Parl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Parl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Parl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Parl.setDirection(DcMotorSimple.Direction.REVERSE);
        Parl.setTargetPosition((int)1000);
        Parl.setPower(1.0);
        sleep(100);
              MoveRight(18,0.6,0.0,0);
              sleep(30);
              Bask.setPower(0.0);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Premov();
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
      FR.setDirection(DcMotorSimple.Direction.FORWARD);
      BL.setDirection(DcMotorSimple.Direction.REVERSE);
      BR.setDirection(DcMotorSimple.Direction.FORWARD);
        setTargetPosition((int)21*79);
        FR.setPower(0.8);
        FL.setPower(1.0);
        BR.setPower(1.0);
        BL.setPower(0.8); 
        while((BR.isBusy()&&FL.isBusy()))
      {telemetry.addData("current",((FL.getCurrentPosition()/79)/21));
      telemetry.update();
          if(((FL.getCurrentPosition()/79.0))>=8)
          {
        FR.setPower(1.0);
        FL.setPower(0.05);
        BR.setPower(1.0);
        BL.setPower(0.05); 
          }
          if(sleep>0)
          {
              if(sleep>=Parl.getCurrentPosition())
              {
                  Parl.setPower(0.0);
                  
              }
          }
      }
      
        // waitUntilMotorsBusy();
        setWheelbasePower(0.0);
        sleep(100);
        Bask.setPower(1.0);
        
    //   turn
        while (colorSensor instanceof DistanceSensor && ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM)<3.5) 
                {}
                Bask.setPower(0.0);
                sleep(100);
                FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Premov();
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
      FR.setDirection(DcMotorSimple.Direction.FORWARD);
      BL.setDirection(DcMotorSimple.Direction.REVERSE);
      BR.setDirection(DcMotorSimple.Direction.FORWARD);
        setTargetPosition(-(int)21*79);
        FR.setPower(1.0);
        FL.setPower(0.05);
        BR.setPower(1.0);
        BL.setPower(0.05);
        while((BR.isBusy()&&FL.isBusy()))
      {telemetry.addData("current",((FL.getCurrentPosition()/79)/21));
      telemetry.update();
          if(Math.abs(((FL.getCurrentPosition()/79.0)))>=13)
          {
        FR.setPower(0.8);
        FL.setPower(1.0);
        BR.setPower(1.0);
        BL.setPower(0.8);
          }
          if(sleep>0)
          {
              if(sleep>=Parl.getCurrentPosition())
              {
                  Parl.setPower(0.0);
                  
              }
          }
      }
                
    //             sleep(1020);
    //           Bask.setPower(0.15);  
    //     MoveBack(forward,1.0);
    //     Bask.setPower(0.0);  
              
    // sleep(15);
    //         nudgeToAngle(0.0);

    // // Parl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    // //             Parl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    // //             Parl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    // //             Parl.setDirection(DcMotorSimple.Direction.REVERSE);
    // //             Parl.setTargetPosition(-(int)3290);
    // //             Parl.setPower(1.0);
    // //             sleep(50);
    //   FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    //     FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    //     BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    //     BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    //     Premov();
    //     FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
    //     BL.setDirection(DcMotorSimple.Direction.FORWARD);
    //     BR.setDirection(DcMotorSimple.Direction.FORWARD);
    //     setTargetPosition(-(int)30.7*79);
    //     FL.setPower(1.0);
    //     FR.setPower(1.0);
    //     BL.setPower(0.256);
    //     BR.setPower(0.256);
    //     while(FL.isBusy()&&BL.isBusy()&&BR.isBusy()&&FR.isBusy())
    //     {
    //         if(!(Parl.isBusy()))
    //         {
    //             Parl.setPower(0.0);
                
    //         }
    //     }
    //     setWheelbasePower(0.0); 
    //     telemetry.addData("how?","idk0.0");
    //     telemetry.update();
    //   sleep(15);
    //   telemetry.addData("how?","idk1.0");
    //     telemetry.update();
    //         nudgeToAngle(90.4);

    //         sleep(30);
    //         Bask.setPower(-1.0);
    //         telemetry.addData("how?","idk2.0");
    //     telemetry.update();
    //         MoveForward(23.4,1.0,0.0);
    //         telemetry.addData("how?","idk3.0");
    //     telemetry.update();
    //         // nudgeToAngle(-90.5);

    //       sleep(1000);
    //             Bask.setPower(-0.2);
    //         // sleep(20);
    // // sleep(100);
    // // // //   sleep(3000);
    //             nudgeToAngle(87.3);

    //   MoveBack(20.2,1.0);
    //   Bask.setPower(0.0);
    //   sleep(30);
    // //   Parl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    // //             Parl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    // //             Parl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    // //             Parl.setDirection(DcMotorSimple.Direction.REVERSE);
    // //             Parl.setTargetPosition((int)3290);
    // //             Parl.setPower(1.0);
    // //             sleep(45);
    //   FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    //     FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    //     BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    //     BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    //     Premov();
    //     FL.setDirection(DcMotorSimple.Direction.REVERSE);
    //     FR.setDirection(DcMotorSimple.Direction.REVERSE);
    //     BL.setDirection(DcMotorSimple.Direction.FORWARD);
    //     BR.setDirection(DcMotorSimple.Direction.FORWARD);
    //     setTargetPosition((int)33*79);
    //     FL.setPower(1.0);
    //     FR.setPower(1.0);
    //     BL.setPower(0.258);
    //     BR.setPower(0.258);
    //     while(FL.isBusy()&&BL.isBusy()&&BR.isBusy()&&FR.isBusy())
    //     {
    //         if(!(Parl.isBusy()))
    //         {
    //             Parl.setPower(0.0);
                
    //         }
    //     }
    //     setWheelbasePower(0.0); 
    //     sleep(13);
    //     nudgeToAngle(0.0);
    //     sleep(45);
    //                   AlignmentToSH();
    //                 //   sleep(10);
    //                   Bask.setPower(0.8);  
    //     while (colorSensor instanceof DistanceSensor && ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM)<3.5) 
    //             {}
    //             sleep(1020);
    //           Bask.setPower(0.15);  
    //     MoveBack(forward,1.0);
    //     Bask.setPower(0.0);  
        
    //   sleep(30);
    // //   MoveBack(forward,1.0);
    //   sleep(9);
    //   Parl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    //             Parl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    //             Parl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    //             Parl.setDirection(DcMotorSimple.Direction.REVERSE);
    //             Parl.setTargetPosition(-(int)3290);
    //             Parl.setPower(1.0);
    //             sleep(30);
    //   FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    //     FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    //     BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    //     BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    //     Premov();
    //     FL.setDirection(DcMotorSimple.Direction.REVERSE);
    //     FR.setDirection(DcMotorSimple.Direction.REVERSE);
    //     BL.setDirection(DcMotorSimple.Direction.FORWARD);
    //     BR.setDirection(DcMotorSimple.Direction.FORWARD);
    //     setTargetPosition(-(int)30.7*79);
    //     FL.setPower(1.0);
    //     FR.setPower(1.0);
    //     BL.setPower(0.258);
    //     BR.setPower(0.258);
    //     while(FL.isBusy()&&BL.isBusy()&&BR.isBusy()&&FR.isBusy())
    //     {
    //         if(!(Parl.isBusy()))
    //         {
    //             Parl.setPower(0.0);
                
    //         }
    //     }
    //     setWheelbasePower(0.0); 
        
    //   sleep(7);
    //         nudgeToAngle(91.2);
    //         Bask.setPower(-1.0);
    //         MoveForward(24,1.0,0.0);
    //         // nudgeToAngle(-90.5);

    //         sleep(1000);
    //             Bask.setPower(-0.15);
    //         // sleep(20);
    // // sleep(100);  
    // // // //   sleep(3000);
    //             nudgeToAngle(87);
    //   sleep(7);
    //   MoveBack(21,1.0);
    //   Bask.setPower(0.0);
    //   sleep(50);
    // //   Parl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    // //             Parl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    // //             Parl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    // //             Parl.setDirection(DcMotorSimple.Direction.REVERSE);
    // //             Parl.setTargetPosition((int)3450);
    // //             Parl.setPower(1.0);
    // //             sleep(50);
    //   FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    //     FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    //     BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    //     BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    //     Premov();
    //     FL.setDirection(DcMotorSimple.Direction.REVERSE);
    //     FR.setDirection(DcMotorSimple.Direction.REVERSE);
    //     BL.setDirection(DcMotorSimple.Direction.FORWARD);
    //     BR.setDirection(DcMotorSimple.Direction.FORWARD);
    //     setTargetPosition((int)30.5*79);
    //     FL.setPower(1.0);
    //     FR.setPower(1.0);
    //     BL.setPower(0.258);
    //     BR.setPower(0.258);
    //     while(FL.isBusy()&&BL.isBusy()&&BR.isBusy()&&FR.isBusy())
    //     {
    //         if(!(Parl.isBusy()))
    //         {
    //             Parl.setPower(0.0);
                
    //         }
    //     }
    //     setWheelbasePower(0.0); 
    //     sleep(25);
    //     // nudgeToAngle(0.0);

    //     nudgeToAngle(0.0);
    //   AlignmentToSH();
    //   sleep(100);
    //           MoveBack(forward,1.0);
    //     nudgeToAngle(0.0);

    //   Bask.setPower(1.0);
    //   while (colorSensor instanceof DistanceSensor && ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM)<3.9) 
    //             {}
    //             sleep(50);
    //             Bask.setPower(0.0);
    //             // sleep(10);
                                

    // MoveBack(forward+0.2,1.0);
    // // Parl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    // //             Parl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    // //             Parl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    // //             Parl.setDirection(DcMotorSimple.Direction.REVERSE);
    // //             Parl.setTargetPosition(-(int)3450);
    // //             Parl.setPower(1.0);
    // //             sleep(110);
    //   FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    //     FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    //     BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    //     BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    //     Premov();
    //     FL.setDirection(DcMotorSimple.Direction.REVERSE);
    //     FR.setDirection(DcMotorSimple.Direction.REVERSE);
    //     BL.setDirection(DcMotorSimple.Direction.FORWARD);
    //     BR.setDirection(DcMotorSimple.Direction.FORWARD);
    //     setTargetPosition(-(int)30.2*79);
    //     FL.setPower(1.0);
    //     FR.setPower(1.0);
    //     BL.setPower(0.258);
    //     BR.setPower(0.258);
    //     while(FL.isBusy()&&BL.isBusy()&&BR.isBusy()&&FR.isBusy())
    //     {
    //         if(!(Parl.isBusy()))
    //         {
    //             Parl.setPower(0.0);
                
    //         }
    //     }
    //     setWheelbasePower(0.0);
        

    //     // MoveBack()
    //     // sleep(5);
    //     Bask.setPower(-1.0);
    //     MoveForward(28,1.0,0.0);
    //     while (colorSensor instanceof DistanceSensor && ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM)>2.0) 
    //             {}
    //             Bask.setPower(0.0);
}
}
