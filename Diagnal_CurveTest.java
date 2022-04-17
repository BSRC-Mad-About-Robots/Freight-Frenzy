package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous

public class Diagnal_CurveTest extends LinearOpMode {
    int width = 320;
    int height = 240;
    BNO055IMU imu;

    // store as variable here so we can access the location
    
    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;
    DcMotor CarL;
    DcMotor CarR;
    DcMotor Parl;
    DcMotor Bask;
   

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
                // Display it for the driver.
                // 
            }
            telemetry.addData("Autonomous",  "Motors busy");
                telemetry.update();
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
    public void  MoveLeft(double inches,double pow, double difference)
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
      FL.setTargetPosition((int)(inches*79));
      FR.setTargetPosition((int)inches*79);
      BR.setTargetPosition((int)(inches*79)-79);
      BL.setTargetPosition((int)(inches*79)-79);
      FL.setPower(pow-difference);
      FR.setPower(pow);
      BL.setPower(pow-0.059);
      BR.setPower(pow-difference-0.059); 
      while(FR.isBusy() && BL.isBusy()&&FL.isBusy()&&BR.isBusy())
      {
      }
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
    public void  MoveRight(double inches,double pow, double difference)
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
      {}
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
    public void  MoveForwardACDC(double inches,double pow,double difference,double divide, int percent, int times)
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
      double difference_2=pow-(pow/divide);
      double decrement= difference_2/times;
      double perinchdecrement=(percent/100)*(inches*79);
      double counter=1;
    while(FR.isBusy() && BL.isBusy()&&FL.isBusy()&&BR.isBusy())
      {
      if(FL.getCurrentPosition()>(inches*79*(percent/100))+perinchdecrement)
      {decrement=decrement*counter;
      perinchdecrement=perinchdecrement*counter;
      FL.setPower((pow-difference)-decrement);
      FR.setPower((pow/divide)-decrement);
      BL.setPower((pow/divide)-decrement);
      BR.setPower((pow-difference)-decrement ); }}      
      setWheelbasePower(0.0);
    }
    @Override
    public void runOpMode() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        parameters.loggingTag          = "IMU";
        // parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = (BNO055IMU) hardwareMap.get("imu");
        imu.initialize(parameters);
     FL = hardwareMap.dcMotor.get("FL");
      BR = hardwareMap.dcMotor.get("BR");
      FR = hardwareMap.dcMotor.get("FR");
      BL = hardwareMap.dcMotor.get("BL");
      
      FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            waitForStart();
            
FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      Premov();
      FL.setDirection(DcMotorSimple.Direction.REVERSE);
      FR.setDirection(DcMotorSimple.Direction.FORWARD);
      BL.setDirection(DcMotorSimple.Direction.REVERSE);
      BR.setDirection(DcMotorSimple.Direction.FORWARD);
      setTargetPosition((int)25*79);
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
    //   sleep(1000);
    //   Premov();
      
      
    //   FL.setDirection(DcMotorSimple.Direction.FORWARD);
    //     FR.setDirection(DcMotorSimple.Direction.FORWARD);
    //     BL.setDirection(DcMotorSimple.Direction.FORWARD);
    //     BR.setDirection(DcMotorSimple.Direction.FORWARD);
    //     setTargetPosition((int)12.5*79);
        
    //     FR.setPower(1.0);
    //     FL.setPower(0.0);
    //     BR.setPower(1.0);
    //     BL.setPower(0.0); 
    //     while(FR.isBusy()){}
     setWheelbasePower(0.0);    //   MoveLeft(75,1.0,0.0);
    // todo: write your code here
}}  