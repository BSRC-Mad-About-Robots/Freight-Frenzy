package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
@TeleOp(name="MarsTeleOp",group="Final")
public class Mars_TeleOp extends OpMode
{
    // CREATING HARDWARE COMPONENT OBJECTS
    //WheelBase
    private DcMotor FL;
    private DcMotor BL;
    private DcMotor FR;
    private DcMotor BR;
    private DcMotor EL, ER, EM;
    BNO055IMU imu;
    DcMotor CarR;
    DcMotor CarL;
    DcMotor Parl;
    DcMotor Bask;
    // CREATING TIME COUNTER
        ElapsedTime runtime = new ElapsedTime();
    private void waitUntilMotorsBusy()
    {
        while ((FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy())) 
            {
                // Display it for the driver.
                // telemetry.addData("Autonomous",  "Motors busy");
                // telemetry.update();
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
 
    public void  MoveForward(double inches,double pow)
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
      FL.setPower(pow);
      FR.setPower(pow);
      BL.setPower(pow);
      BR.setPower(pow);
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
      FL.setDirection(DcMotorSimple.Direction.FORWARD);
      FR.setDirection(DcMotorSimple.Direction.REVERSE);
      BL.setDirection(DcMotorSimple.Direction.FORWARD);
      BR.setDirection(DcMotorSimple.Direction.REVERSE);
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
      setTargetPosition((int)inches*79);
      FL.setPower(pow-difference);
      FR.setPower(pow);
      BL.setPower(pow);
      BR.setPower(pow-difference); 
      waitUntilMotorsBusy();
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
        FR.setPower(pow);
        FL.setPower(pow-difference);
        BR.setPower(pow-difference);
        BL.setPower(pow); 
        waitUntilMotorsBusy();
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
    @Override
    public void init() 
    {
      // SHOWING THAT THE HARDWARE IS GETTING INITIALIZED ON DRIVER STATION VIA TELEMETRY
      telemetry.addData("Status", "INITIALIZING");
      telemetry.update();
      
      //Initialize in hardware map to use in configurations
      // DECLARING CONFIGURATION NAMES OF HARDWARE COMPONENTS
        FL = hardwareMap.dcMotor.get("FL");
        BL = hardwareMap.dcMotor.get("BL");
        FR = hardwareMap.dcMotor.get("FR");
        BR = hardwareMap.dcMotor.get("BR");
        ER = hardwareMap.dcMotor.get("FR");
        EL = hardwareMap.dcMotor.get("BL");
        EM = hardwareMap.dcMotor.get("FL");
        CarR=hardwareMap.dcMotor.get("CarR");
        CarL=hardwareMap.dcMotor.get("CarL");
        Parl=hardwareMap.dcMotor.get("Parl");
        Bask=hardwareMap.dcMotor.get("Baskq");


            
        
        //Give initial direction because of bevel gear
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        // BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Parl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //EL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //ER.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //EM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
     @Override
    public void loop() 
    {
            
            
            // telemetry.addData("EL=",EL.getCurrentPosition());
            // telemetry.addData("ER=",ER.getCurrentPosition());
            // telemetry.addData("EM=",EM.getCurrentPosition());
            // telemetry.update();
                
            
            
            //double leftstickx = 0;
            //double leftsticky = 0;
            //double dpadpower = 0.25;
            
            float vertical;
            float horizontal;
            float pivot;
            if(!gamepad2.y)
            {
            double power_factor = -0.6;
        
            vertical = -gamepad1.right_stick_y;
            horizontal = gamepad1.right_stick_x;
            
            if(gamepad1.left_trigger>0){
              pivot = -gamepad1.left_trigger;
            }else if(gamepad1.right_trigger>0){
              pivot = +gamepad1.right_trigger;
            }else{
              pivot = 0;
            }
            if(!gamepad1.right_bumper && !gamepad1.left_bumper)
            {
            FR.setPower((-pivot + (vertical - horizontal)) * power_factor);
            BR.setPower((-pivot + vertical + horizontal) * power_factor);
            FL.setPower((pivot + vertical + horizontal) * power_factor);
            BL.setPower((pivot + (vertical - horizontal)) * power_factor);
            }
            
        
            // Put loop blocks here.
            telemetry.addData("Motor", "power FL %f, FR %f, BL %f, BR %f", FL.getPower(), FR.getPower(), BL.getPower(), BR.getPower());
            telemetry.addData("Motor", "vertical %f, horizontal %f, pivot %f", vertical, horizontal, pivot);
            telemetry.addData("Motor", "gamepad1.left_stick %f %f", gamepad1.left_stick_x, gamepad1.left_stick_y);
            telemetry.addData("Motor", "gamepad1.right_stick %f %f", gamepad1.right_stick_x, gamepad1.right_stick_y);
            telemetry.update(); 
            if(gamepad1.dpad_up){
                Parl.setPower(-0.8);
            }
            else if(gamepad1.dpad_down)
            {
                Parl.setPower(0.5);

            }
            else{
                Parl.setPower(0.0);
            }
            if(gamepad2.left_trigger>0)
            {
                Bask.setPower(-(gamepad2.left_trigger));
                
            }
            else if(gamepad2.right_trigger>0){
                Bask.setPower(gamepad2.right_trigger);
            }
            else 
            {
                Bask.setPower(0.0);
            }
            if(gamepad2.right_bumper)
            {
                CarR.setPower(-0.6);
            }
            else if(gamepad2.left_bumper)
            {
                CarL.setPower(-0.6);
            }
            else
            {
                CarR.setPower(0.0);
                CarL.setPower(0.0);
            }
            
            if(gamepad2.y){
                FL.setPower(0.0);
                BL.setPower(0.0);
                FR.setPower(0.0);
                BR.setPower(0.0);
                CarR.setPower(0.0);
                CarL.setPower(0.0);
                Parl.setPower(0.0);
                Bask.setPower(0.0);
                
            }
            if(gamepad1.left_bumper)
            {
                
                // Premov();
                
                // setTargetPosition((int)31*79);
                FL.setPower(1.0);
                FR.setPower(-1.0);
                BL.setPower(-0.32);
                BR.setPower(0.32);
                // waitUntilMotorsBusy();
                // setWheelbasePower(0.0);
                
                
            }
            else if(gamepad1.right_bumper)
            {
                
                // Premov();
                
                // setTargetPosition(-(int)31*79);
                FL.setPower(-1.0);
                FR.setPower(1.0);
                BL.setPower(0.32);
                BR.setPower(-0.32);
                // waitUntilMotorsBusy();
                // setWheelbasePower(0.0);
                
                
            }
            
          /*if(gamepad1.dpad_up)
            {
                leftsticky = dpadpower;
            }
            else if(gamepad1.dpad_right)
            {
                leftstickx = dpadpower;
            }
            else if(gamepad1.dpad_down)
            {
                leftsticky = -dpadpower;
            }
             else if(gamepad1.dpad_left)
            {
                leftstickx = -dpadpower;
            }
            else
            {
              leftstickx = gamepad1.left_stick_x;
              leftsticky = -gamepad1.left_stick_y;
              
               
            }*/
            
           
       
      }
}
}
 //   public void sleepTeleOp(double runTime, double reqTime) {
 //    while (runtime.seconds() < (reqTime + runTime)) {}
    
       


