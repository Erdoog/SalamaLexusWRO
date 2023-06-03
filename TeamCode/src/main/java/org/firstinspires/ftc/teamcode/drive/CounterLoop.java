package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Config
@Autonomous
public class CounterLoop extends OpMode {
    SampleTankDrive drive1;

    Trajectory traj;
    public static double ang = 90;

    @Override
    public void init()
    {
        drive1 = new SampleTankDrive(hardwareMap);

        traj = drive1.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(20, 0))
                .splineTo(new Vector2d(54, 36), Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(54, 74))
                .splineTo(new Vector2d(20, 110), Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(-20, 110))
                .splineTo(new Vector2d(-54, 74), Math.toRadians(270))
                .lineToConstantHeading(new Vector2d(-54, 36))
                .splineTo(new Vector2d(-20, 0), Math.toRadians(360))
                .lineToConstantHeading(new Vector2d(20, 0))
                .splineTo(new Vector2d(54, 36), Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(54, 74))
                .splineTo(new Vector2d(20, 110), Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(-20, 110))
                .splineTo(new Vector2d(-54, 74), Math.toRadians(270))
                .lineToConstantHeading(new Vector2d(-54, 36))
                .splineTo(new Vector2d(-20, 0), Math.toRadians(360))
                .lineToConstantHeading(new Vector2d(20, 0))
                .splineTo(new Vector2d(54, 36), Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(54, 74))
                .splineTo(new Vector2d(20, 110), Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(-20, 110))
                .splineTo(new Vector2d(-54, 74), Math.toRadians(270))
                .lineToConstantHeading(new Vector2d(-54, 36))
                .splineTo(new Vector2d(-20, 0), Math.toRadians(360))
                .lineToConstantHeading(new Vector2d(10, 0))
                .build();

        drive1.followTrajectoryAsync(traj);
    }

    @Override
    public void loop()
    {
        drive1.update();
        telemetry.addData("class", drive1.outputable);
    }
}
