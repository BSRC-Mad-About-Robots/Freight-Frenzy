package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Rev IMU Test2")
public class TestImu4 extends LinearOpMode {
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
        while(opModeIsActive())
        {
            double startTime = timer.milliseconds();
            Orientation or = imu.getAngularOrientation();
            double endTime = timer.milliseconds();
            
            
           
                
                double first = or.firstAngle;
                double second = or.secondAngle;
                double third = or.thirdAngle;
                telemetry.addData("first:",String.format("%.2f,%.4f", endTime - startTime, first));
                telemetry.addData("second:", second);
                telemetry.addData("third:", third);
                telemetry.update();
            }
            
        
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
