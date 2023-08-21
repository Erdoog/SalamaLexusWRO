package org.firstinspires.ftc.teamcode.drive.Detectio;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class DetectionTest extends LinearOpMode
{
    DetectionMaster cvMaster;
    @Override
    public void runOpMode()
    {
        cvMaster = new DetectionMaster(this);
        cvMaster.observeStick();
        waitForStart();
        while (opModeIsActive())
        {
            telemetry.addData(  "redArea", DetectionPipeLine.redArea);
            telemetry.addData("greenArea", DetectionPipeLine.greenArea);
            telemetry.update();
        }
    }
}
