package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp(name="TestBasket2",group="Test")

public class TestBasket2 extends OpMode {
    // private DcMotor FL;
    private DcMotor FL;
    private DcMotor BL; 
    private DcMotor FR;
    private DcMotor BR;
    CRServo Out;
    DcMotor CarR;

    //In1 and In2 are compliant wheels
    //Rot is for rotating the delivery mechanism of TeleOp
    private DcMotor In,In2, Rot;

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

    // @Override
     public void init() 
    {
        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BR = hardwareMap.dcMotor.get("BR");
        BL = hardwareMap.dcMotor.get("BL");
        CarR=hardwareMap.dcMotor.get("CarR");
    }
    @Override
    public void loop() 
    {
        if(true){
            // FL.setPower(0.3);
            // FR.setPower(0.5);
            CarR.setPower(0.5);
            // BR.setPower(0.5);
            
        }
        else if(true)
        {
            CarR.setPower(-0.5);
            // FR.setPower(-0.5);
            // BL.setPower(-0.5);
            // BR.setPower(-0.5);
        }
        else if(gamepad1.y)
        {
            FR.setPower(-1.0);
            // FR.setPower(-0.5);
            // BL.setPower(-0.5);
            // BR.setPower(-0.5);
        }
        else if(gamepad1.x)
        {
            BR.setPower(-0.5);
            // FR.setPower(-0.5);
            // BL.setPower(-0.http://192.168.43.1:8080/java/editor.html?/src/org/firstinspires/ftc/teamcode/TestInnovativeidea.java5);
            // BR.setPower(-0.5);
        }
        else{
            FL.setPower(0.0);
            FR.setPower(0.0);
            BL.setPower(0.0);
            BR.setPower(0.0);
            CarR.setPower(0.0);
        }
        
    }
    // todo: write your code here
}