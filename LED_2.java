package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class LED_2 extends LinearOpMode {
    @Override 
    public void runOpMode()
    {
        waitForStart();
        NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        DcMotor CarR = hardwareMap.dcMotor.get("CarR");
        CarR.setDirection(DcMotorSimple.Direction.REVERSE);
        while (true)
        {
            if (colorSensor instanceof DistanceSensor && ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM)<2.0) CarR.setPower(0.5);
            else CarR.setPower(0.0);
        }
    }
}