package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@Autonomous(name="FieldChange")
public class FieldChangeOpMode extends OpMode {
    SampleTankDrive drive1;

    Trajectory traj;
    public static double ang = 90;

    FtcDashboard dash;

    @Override
    public void init()
    {
        drive1 = new SampleTankDrive(hardwareMap);
        dash = FtcDashboard.getInstance();

        digitalChannel = hardwareMap.get(DigitalChannel.class, "DigitalDevice");

        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay()
                .setFill("grey")
                .fillRect(-59.055, -59.055, 118.11, 118.11)
                .setFill("yellow")
                .fillRect(-19.685, -19.685, 39.37, 39.37)
                .drawGrid(12.945, 12.945, 118.11, 118.11, 4, 4);
        dash.sendTelemetryPacket(packet);

        traj = drive1.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(-20, 0))
//                .splineTo(new Vector2d(-54, 36), Math.toRadians(90))
//                .lineToConstantHeading(new Vector2d(-54, 74))
//                .splineTo(new Vector2d(-20, 108), Math.toRadians(0))
//                .lineToConstantHeading(new Vector2d(20, 110))
//                .splineTo(new Vector2d(54, 74), Math.toRadians(270))
//                .lineToConstantHeading(new Vector2d(54, 36))
//                .splineTo(new Vector2d(20, 0), Math.toRadians(180))
//                .lineToConstantHeading(new Vector2d(-20, 0))
//                .splineTo(new Vector2d(-54, 36), Math.toRadians(90))
//                .lineToConstantHeading(new Vector2d(-54, 74))
//                .splineTo(new Vector2d(-20, 108), Math.toRadians(0))
//                .lineToConstantHeading(new Vector2d(20, 108))
//                .splineTo(new Vector2d(54, 74), Math.toRadians(270))
//                .lineToConstantHeading(new Vector2d(54, 36))
//                .splineTo(new Vector2d(20, 0), Math.toRadians(180))
//                .lineToConstantHeading(new Vector2d(-20, 0))
//                .splineTo(new Vector2d(-54, 36), Math.toRadians(90))
//                .lineToConstantHeading(new Vector2d(-54, 74))
//                .splineTo(new Vector2d(-20, 108), Math.toRadians(0))
//                .lineToConstantHeading(new Vector2d(20, 108))
//                .splineTo(new Vector2d(54, 74), Math.toRadians(270))
//                .lineToConstantHeading(new Vector2d(54, 36))
//                .splineTo(new Vector2d(20, 0), Math.toRadians(180))
//                .lineToConstantHeading(new Vector2d(0, 0))
                .build();

        drive1.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(180)));
        drive1.followTrajectoryAsync(traj);
    }

    DigitalChannel digitalChannel;
    @Override
    public void loop()
    {
        drive1.update();

        telemetry.addData("class", drive1.outputable);
        telemetry.update();
    }
}
