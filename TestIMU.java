package org.firstinspires.ftc.teamcode.Autonomous;  

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import java.util.ArrayList;
import java.util.List;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.teamcode.Hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
@Autonomous(name="TestIMU", group ="Vuforia")
public class TestIMU extends LinearOpMode
{
    public IMU imu;
    BNO055IMU gyro;
    
    //hardware components
    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;

    //creating time counter
    private ElapsedTime runtime = new ElapsedTime();

        private void turnToAngle(double target_angle)
        {
            Orientation angles;
            double angle_to_turn = target_angle;
            //  imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            double current_angle;// = imu.readCurrentHeading();
            angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            current_angle = imu.readCurrentHeading(); //Because REV Hub is upside down
            
            FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            
            FL.setDirection(DcMotorSimple.Direction.REVERSE);
            FR.setDirection(DcMotorSimple.Direction.REVERSE);
            BL.setDirection(DcMotorSimple.Direction.REVERSE);
            BR.setDirection(DcMotorSimple.Direction.REVERSE);
            setWheelbasePower(0.4);
            current_angle = angles.firstAngle;
            while (target_angle - current_angle > 0.5)
            {
                
                telemetry.addData("IMU: ", "current %f, target %f", current_angle, target_angle);
                telemetry.update();
                            //current_angle = imu.readCurrentHeading();
                angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                current_angle = angles.firstAngle; //Because REV Hub is upside down

            }
            
            setWheelbasePower(0.0);
        }
         private void GoForwardIMU(double inches,double pow, double target_angle)
        {
            Orientation angles;
            double angle_to_turn = target_angle;
            double current_angle;// = imu.readCurrentHeading();
            angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            current_angle = angles.firstAngle; //Because REV Hub is upside down
            
            FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            
            FL.setDirection(DcMotorSimple.Direction.REVERSE);
            FR.setDirection(DcMotorSimple.Direction.FORWARD);
            BL.setDirection(DcMotorSimple.Direction.REVERSE);
            BR.setDirection(DcMotorSimple.Direction.FORWARD);
            setTargetPosition(50*79);
            FL.setPower(pow);
            FR.setPower(pow);
            BL.setPower(pow);
            BR.setPower(pow); 
            //setWheelbasePower(0.5); current_angle = angles.firstAngle;
            while (opModeIsActive() && (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy())) 
            {
                current_angle = angles.firstAngle;
                if(current_angle-target_angle>0)
                {
                    double k=(current_angle-target_angle)/60;
                    telemetry.addData("k", k);
                    telemetry.update();
                    sleep(4000);
                    FL.setPower(pow);
                    FR.setPower(pow-k);
                    BL.setPower(pow);
                    BR.setPower(pow-k); 
                }
                else if (current_angle-target_angle<0)
                {   double k=(target_angle-current_angle)/60;
                    telemetry.addData("k", k);
                    telemetry.update();
                    sleep(4000);
                    FL.setPower(pow-k);
                    FR.setPower(pow);
                    BL.setPower(pow-k);
                    BR.setPower(pow); 
                }
                else
                {
                    double k=(target_angle-current_angle)/100;
                    telemetry.addData("k", k);
                    telemetry.update();
                    sleep(4000);
                }
            }
            
            setWheelbasePower(0.0);
        }

        @Override public void runOpMode()
        {
            //hardwareMap
            FL = hardwareMap.dcMotor.get("FL");
            BR = hardwareMap.dcMotor.get("BR");
            FR = hardwareMap.dcMotor.get("FR");
            BL = hardwareMap.dcMotor.get("BL");

            gyro = hardwareMap.get(BNO055IMU.class, "imu");
            
            imu = new IMU(gyro);
            
            imu.initialize();   
            
            double current_angle = imu.readCurrentHeading();
            telemetry.addData("IMU: ", "current %f", current_angle);
            telemetry.update();
            
            waitForStart();
            Orientation angles;
            double target_angle=0;
            double angle_to_turn = target_angle;
            // double current_angle;// = imu.readCurrentHeading();
            angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
             //Because REV Hub is upside down
            
            // FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // Premov();
            // current_angle = angles.firstAngle;
            // FL.setDirection(DcMotorSimple.Direction.REVERSE);
            // FR.setDirection(DcMotorSimple.Direction.FORWARD);
            // BL.setDirection(DcMotorSimple.Direction.REVERSE);
            // BR.setDirection(DcMotorSimple.Direction.FORWARD);
            // setTargetPosition(400*79);
            // FL.setPower(0.8);
            // FR.setPower(0.8);
            // BL.setPower(0.8);
            // BR.setPower(0.8); 
            // // setWheelbasePower(0.5); 
            // current_angle = imu.readCurrentHeading();
            // sleep(100);
            // while (opModeIsActive() && (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy())) 
            // {
            //     current_angle = imu.readCurrentHeading();
            //     if(current_angle-target_angle>0)
            //     {
            //         current_angle = imu.readCurrentHeading();
            //         double k=(current_angle-target_angle);
            //         telemetry.addData("k", k);
            //         telemetry.update();
            //         sleep(4000);
            //         FL.setPower(0.8-k);
            //         FR.setPower(0.8);
            //         BL.setPower(0.8-k);
            //         BR.setPower(0.8); 
            //     }
            //     else if (current_angle-target_angle<0)
            //     {   current_angle = imu.readCurrentHeading();
            //         double k=(target_angle-current_angle);
            //         telemetry.addData("k", k);
            //         telemetry.update();
            //         // sleep(4000);
            //         FL.setPower(0.8);
            //         FR.setPower(0.8-k);
            //         BL.setPower(0.8);
            //         BR.setPower(0.8-k); 
            //     }
            //     else
            //     {
            //         double k=(target_angle-current_angle)/20;
            //         telemetry.addData("k", current_angle);
            //         telemetry.update();
            //         // sleep(4000);
            //     }
            // }
            
            // setWheelbasePower(0.0);
        turnToAngle(180);
            // GoForwardIMU(50,1,100);
            
            /*
            //init of
            int total_ticks;
            int R1, R2;
            double PMax;
            float D;
            double straight;
            double rot;

            //hardwareMap
            FL = hardwareMap.dcMotor.get("FL");
            BR = hardwareMap.dcMotor.get("BR");
            FR = hardwareMap.dcMotor.get("FR");
            BL = hardwareMap.dcMotor.get("BL");

            //after play is pressed
            waitForStart();

            //RIGHT Straffing79*26
            double startTime = runtime.seconds();
            FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            
            Premov();
            directionRight();
           total_ticks = 79*35;
            R1 = 500;
            R2 = 1000;
            PMax = 1.0;
            ACDC2(total_ticks, R1, R2, PMax);
            */
            
            /*setTargetPosition(79*35);
            setWheelbasePower(0.3);
            waitUntilMotorsBusy();
            setWheelbasePower(0.0);*/
            double endTime = runtime.seconds();
            //telemetry.addData("Time: %d sec", endTime - startTime);
            telemetry.update();
          }
          
    //while motors are busy dont perform next action
    private void waitUntilMotorsBusy()
    {
        while (opModeIsActive() &&
            (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy()))
            {
                // Display it for the driver.
                telemetry.addData("Autonomous",  "Motors busy");
                double P = FL.getPower();
                telemetry.addData("MOTOR", "Power %f", P);
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
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
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

    //ACCELERATION and DECELERATION method
   public void ACDC2(int total_ticks, int R1, int R2, double PMax)
    {
        //total_ticks=target position
        FL.setTargetPosition(total_ticks);
        FR.setTargetPosition(total_ticks);
        BL.setTargetPosition(total_ticks);
        BR.setTargetPosition(total_ticks);

        //minimum power
        double PMin = 0.25;
        while (FL.isBusy() || FR.isBusy() || BL.isBusy() || BR.isBusy())
        {
            int current_ticks = FL.getCurrentPosition();
            telemetry.addData("Current ticks: ", "%d", current_ticks);

            double P = 0.0;
            if (current_ticks<R1)
            {
                P = PMin + ((PMax - PMin) * current_ticks) / R1;
            }
            else if ((current_ticks > R1) && (current_ticks <= (total_ticks-R2)))
            {
                P = PMax;
            }
            else if (current_ticks > (total_ticks - R2))
            {
                int R3 = R2 / 2;
                P = PMin + (PMax - PMin) * (total_ticks - R3 - current_ticks) / R3;

                if (current_ticks > (total_ticks - R3))
                {
                    P = PMin;
                    setWheelbasePower(P);
                    telemetry.addData("Current power: ", "%f", P);
                    telemetry.update();
                    break;
                }
            }

            setWheelbasePower(P);
            telemetry.addData("Current power: ", "%f", P);
            if (P > 1.0)
                telemetry.addData("ERROR: ", "Power > 1.0");
                
            telemetry.update();
        }
    }



 }
