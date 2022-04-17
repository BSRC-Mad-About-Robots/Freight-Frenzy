package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous

public class Randomization_Test extends LinearOpMode
{
    SkystoneDetector_Copy detector = new SkystoneDetector_Copy(telemetry);

    @Override
    public void runOpMode()
    {
        telemetry.addData("location", detector.getPostitionCapstone());
        telemetry.update();
        sleep(10000);
                telemetry.addData("location1", detector.getPostitionCapstone());
        telemetry.update();
sleep(10000);
    }
}