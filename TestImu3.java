package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Rev IMU Test")
public class TestImu3 extends LinearOpMode {
    BNO055IMU imu;
    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        initOpMode();
        FL = hardwareMap.dcMotor.get("FL");
        BR = hardwareMap.dcMotor.get("BR");
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        waitForStart();
        timer.reset();
        
            double startTime = timer.milliseconds();
            double yaw = imu.getAngularOrientation().firstAngle;
            double endTime = timer.milliseconds();
            FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            
            FL.setDirection(DcMotorSimple.Direction.REVERSE);
            FR.setDirection(DcMotorSimple.Direction.REVERSE);
            BL.setDirection(DcMotorSimple.Direction.REVERSE);
            BR.setDirection(DcMotorSimple.Direction.REVERSE);
            FL.setPower(-0.8);
            FR.setPower(-0.8);
            BL.setPower(-0.8);
            BR.setPower(-0.8);
            telemetry.addData("check3:",(90-yaw>= 1));
            telemetry.update();
            while(180-yaw>= 16){
                
                yaw = imu.getAngularOrientation().firstAngle;
                telemetry.addData("check:",String.format("%.2f,%.4f", endTime - startTime, yaw));
                telemetry.update();
            }
            FL.setPower(0.0);
            FR.setPower(0.0);
            BL.setPower(0.0);
            BR.setPower(0.0);
        
    }

    private void initOpMode() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        parameters.loggingTag          = "IMU";
        // parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = (BNO055IMU) hardwareMap.get("imu");
        imu.initialize(parameters);
    }
}    // todo: write your code here
