package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Config
public class LogDriving extends OpMode {
    DcMotorEx leftMotor;
    DcMotorEx rightMotor;
    Servo steer;

    @Override
    public void init()
    {
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class,   "rightMotor");

        steer = hardwareMap.get(Servo.class, "steer");

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive = new SampleTankDrive(hardwareMap);
    }

    public static double steerMulti = -0.11;
    public static double speed = 1;
    public static double zeroSteer = 0.425;

    SampleTankDrive drive;

    @Override
    public void loop()
    {
        double forwardPower = -gamepad1.left_stick_y * LogDriving.speed;
        double rotationalPower = gamepad1.right_stick_x;

        drive.setDrivePower(new Pose2d(forwardPower, 0, rotationalPower));
        telemetry.addData("turn", drive.outputable);
        telemetry.update();
    }
}
