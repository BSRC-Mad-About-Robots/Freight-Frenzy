package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp(name="MARSTeleOpFinal__1_",group="Final")
public class MARSTeleOpFinal__1_ extends OpMode
{
    // CREATING HARDWARE COMPONENT OBJECTS
    //WheelBase
    private DcMotor FL;
    private DcMotor BL;
    private DcMotor FR;
    private DcMotor BR;
    CRServo Parkings;

   
    //In1 and In2 are compliant wheels
    //Rot is for rotating the delivery mechanism of TeleOp
    private DcMotor In1,In2, Rot;
    
    //del is the delivery mechanism of picking and dropping
    private Servo del;
    
    // CRServo used to move foundation
    //check how to use a crservo
    //change to servo
    Servo F1;
    Servo F2;
   
    //Used for pully to lift the stone(misoumi)  
    DcMotor pull;
    //servo for capstone 
    Servo Capstone ;
    
    // CREATING TIME COUNTER
        ElapsedTime runtime = new ElapsedTime();
    
    @Override
    public void init() 
    {
      // SHOWING THAT THE HARDWARE IS GETTING INITIALIZED ON DRIVER STATION VIA TELEMETRY
      telemetry.addData("Status", "INITIALIZING");
      telemetry.update();
      
      //Initialize in hardware map to use in configurations
      // DECLARING CONFIGURATION NAMES OF HARDWARE COMPONENTS
        BL = hardwareMap.dcMotor.get("FL");
        FL = hardwareMap.dcMotor.get("BL");
        BR = hardwareMap.dcMotor.get("FR");
        FR= hardwareMap.dcMotor.get("BR");
        
        //Delivery Mechanism
        In1= hardwareMap.dcMotor.get("In1");
        In2= hardwareMap.dcMotor.get("In2");
        del= hardwareMap.servo.get("Drop"); 
      Parkings = hardwareMap.crservo.get("Pk"); 

        Rot = hardwareMap.dcMotor.get("Rotation");
        
        //Pulley Mechanism
        pull= hardwareMap.dcMotor.get("Pulley");
       
        // CRServo for foundation movement 
        F1 = hardwareMap.servo.get("F1");
        F2 = hardwareMap.servo.get("FOUND2");
         
        Capstone = hardwareMap.servo.get("Cap"); 
    
        //Give initial direction because of bevel gear
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
       
        // initially apply brake to hold the position of pulley
        pull.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       
    }
     @Override
    public void loop() 
    {
            //double leftstickx = 0;
            //double leftsticky = 0;
            //double dpadpower = 0.25;
            
            float vertical;
            float horizontal;
            float pivot;
            
            double power_factor = 1.0;
        
            vertical = -gamepad1.right_stick_y;
            horizontal = gamepad1.right_stick_x;
            
            if(gamepad1.left_trigger>0){
              pivot = -gamepad1.left_trigger;
            }else if(gamepad1.right_trigger>0){
              pivot = +gamepad1.right_trigger;
            }else{
              pivot = 0;
            }
            FR.setPower((-pivot + (vertical - horizontal)) * power_factor);
            BR.setPower((-pivot + vertical + horizontal) * power_factor);
            FL.setPower((pivot + vertical + horizontal) * power_factor);
            BL.setPower((pivot + (vertical - horizontal)) * power_factor);
        
            // Put loop blocks here.
            telemetry.addData("Motor", "power FL %f, FR %f, BL %f, BR %f", FL.getPower(), FR.getPower(), BL.getPower(), BR.getPower());
            telemetry.addData("Motor", "vertical %f, horizontal %f, pivot %f", vertical, horizontal, pivot);
            telemetry.addData("Motor", "gamepad1.left_stick %f %f", gamepad1.left_stick_x, gamepad1.left_stick_y);
            telemetry.addData("Motor", "gamepad1.right_stick %f %f", gamepad1.right_stick_x, gamepad1.right_stick_y);
            telemetry.update();
            
          /* if(gamepad1.dpad_up)
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
            
            
            //compliant wheel movement 
            if (gamepad2.right_stick_button) 
            {
                In1.setDirection(DcMotorSimple.Direction.REVERSE);
                In2.setDirection(DcMotorSimple.Direction.REVERSE);
                In1.setPower(1.0);
                In2.setPower(1.0);
            } 
            else
            {
                In1.setPower(0);
                In2.setPower(0);
            }
             //compliant wheel movement
            if(gamepad1.right_stick_button)
            {
                In1.setDirection(DcMotorSimple.Direction.FORWARD);
                In2.setDirection(DcMotorSimple.Direction.FORWARD);
                In1.setPower(1.0);
                In2.setPower(1.0);
            }
            
            //delivery mechanism
            //to grab
            if(gamepad2.right_bumper)
            {
                del.setPosition(1);
            }
            //to drop
            else if(gamepad2.left_bumper)
            {
                del.setPosition(0.44);
            }
            if (gamepad2.a)
            
             { 
                Parkings.setDirection(CRServo.Direction.FORWARD);
                Parkings.setPower(1.0);
                
            } 
            //if nothing is pressed maintain the normal position when play is pressed 
           
            else if (gamepad2.b)
            
              {
                Parkings.setDirection(CRServo.Direction.REVERSE);
                Parkings.setPower(1.0);
                
            } else{
            //if nothing is pressed maintain the normal position when play is pressed 
             Parkings.setPower(0);
            }
            //rotation of delivery mechanism
            //to go the side into robot 
            if(gamepad1.left_stick_x>0) 
            {
                
                Rot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
                Rot.setDirection(DcMotorSimple.Direction.FORWARD);
                Rot.setPower(0.15);
            } 
            //to go away from the robot
            else if(gamepad1.left_stick_x<0)
            {
                Rot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
                Rot.setDirection(DcMotorSimple.Direction.REVERSE);
                Rot.setPower(0.15);
            } 
            else 
            {
                Rot.setPower(0);
            }
            if(gamepad1.left_stick_y>0) 
            { 
                Capstone.setPosition(0.325);
                telemetry.addData("Capstone", "right bumper");
            } 
            else if(gamepad1.left_stick_y<0 ) 
            {   
                Capstone.setPosition(0.8);
                telemetry.addData("Capstone", "left bumper");  
            }    
                
            //foundation mechanism
            if (gamepad1.left_bumper)
            {
                F1.setPosition(0.45);
                F2.setPosition(0.15);
                telemetry.addData("FOUNDATION: ", "Up"); 
            }
            //hold foundation if button is pressed i.e down direction
            else if(gamepad1.right_bumper)
            {
                F1.setPosition(0.1);
                F2.setPosition(0.65);
                telemetry.addData("FOUNDATION: ", "Down");  
            }
            
            /*if(gamepad2.a)
            {
                pull.setDirection(DcMotorSimple.Direction.REVERSE);
                pull.setPower(0.5);
                
                while(pull.isBusy())
                
                Rot.setDirection(DcMotorSimple.Direction.FORWARD);
                Rot.setPower(0.15);
               
                pull.setDirection(DcMotorSimple.Direction.FORWARD);
                pull.setPower(0.5);
            }
            else
            {
              pull.setPower(0);
              Rot.setPower(0);
            
            }*/            
            //pulley movement
            //int is used for typecasting to know if the left stick is comletely moved to extreme position
            if((int)gamepad2.left_stick_y == 1){
                pull.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
                pull.setDirection(DcMotorSimple.Direction.REVERSE);
                pull.setPower(0.7);
            } else if((int)gamepad2.left_stick_y == -1) {
                pull.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
                pull.setDirection(DcMotorSimple.Direction.FORWARD);
                pull.setPower(0.7);
            } else {
                pull.setPower(0);
            }
            
            telemetry.update();
                 
      if(gamepad2.y)
      {
      /* pull.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pull.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        
        pull.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pull.setDirection(DcMotorSimple.Direction.FORWARD);
        pull.setTargetPosition(79*30);
        pull.setPower(1.0);
        while (pull.isBusy()) {
         telemetry.addLine("Delivery UP");
         telemetry.update();
          sleepTeleOp(runtime.seconds(),1);
          Rot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rot.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        
        Rot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Rot.setDirection(DcMotorSimple.Direction.FORWARD);
        Rot.setTargetPosition(12*15);;
      Rot.setPower(0.3);
        while (Rot.isBusy()) {
         telemetry.addLine("Delivery rotate");
         telemetry.update();
                  
        }
        Rot.setPower(0.0);
        Rot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      Rot.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);*/
      /* pull.setPower(0.0);
      pull.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      pull.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);*/
        
      }
      // if(gamepad2.a)
      {
      /* pull.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pull.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        
        pull.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pull.setDirection(DcMotorSimple.Direction.FORWARD);
        pull.setTargetPosition(79*30);
        pull.setPower(1.0);
        while (pull.isBusy()) {
         telemetry.addLine("Delivery UP");
         telemetry.update();
          sleepTeleOp(runtime.seconds(),1);*/
        /*  Rot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rot.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        
        Rot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Rot.setDirection(DcMotorSimple.Direction.FORWARD);
        Rot.setTargetPosition(12*11);
      Rot.setPower(0.3);
        while (Rot.isBusy()) {
         telemetry.addLine("Delivery rotate");
         telemetry.update();
                   
        }
        Rot.setPower(0.0);
        Rot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      Rot.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
      /* pull.setPower(0.0);
      pull.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      pull.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        
      }*/
      }
      }
    public void sleepTeleOp(double runTime, double reqTime) {
      while (runtime.seconds() < (reqTime + runTime)) {}
    }
       
}
    // todo: write your code here
