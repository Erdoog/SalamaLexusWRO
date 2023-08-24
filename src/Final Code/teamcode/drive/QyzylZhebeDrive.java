package org.firstinspires.ftc.teamcode.drive;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp
public class QyzylZhebeDrive extends OpMode {
    MotorEx leftDrive, rightDrive;
    MotorEx leftLift, rightLift;
//    MotorEx intake;

    Servo intakeDoor, depositorDoor;

    GamepadEx gamepadEx;

    @Override
    public void init()
    {
        gamepadEx = new GamepadEx(gamepad1);

        leftDrive = new MotorEx(hardwareMap, "left_drive");
        rightDrive = new MotorEx(hardwareMap, "right_drive");
        leftDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        leftLift = new MotorEx(hardwareMap, "left_lift");
        rightLift = new MotorEx(hardwareMap, "right_lift");
        leftLift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

//        intake = new MotorEx(hardwareMap, "intake");

        intakeDoor = hardwareMap.get(Servo.class, "intake_door");
        depositorDoor = hardwareMap.get(Servo.class, "depositor_door");

        leftDrive.setInverted(true);
        leftLift.setInverted(true);
    }

    boolean intakeState = false;
    boolean intakeDoorState = false;
    boolean depositorDoorState = false;
    boolean aPrevState = false;
    boolean bPrevState = false;
    boolean rbPrevState = false;

    @Override
    public void loop()
    {
        double forw = Math.pow(gamepad1.right_trigger, 3) - Math.pow(gamepad1.left_trigger, 3);
        double rot = Math.pow(gamepad1.left_stick_x, 2) * (gamepad1.left_stick_x > 0 ? 1.0 : -1.0);

        if (gamepad1.right_bumper)
        {
            if (rbPrevState)
                intakeState = !intakeState;
        }
        rbPrevState = gamepad1.right_bumper;

        if (gamepad1.b)
        {
            if (!bPrevState)
                depositorDoorState = !depositorDoorState;
            telemetry.addLine("gamepadB");
        }
        bPrevState = gamepad1.b;

        if (gamepad1.a)
        {
            if (!aPrevState)
                intakeDoorState = !intakeDoorState;
            telemetry.addLine("gamepadA");
        }
        aPrevState = gamepad1.a;

        if (Math.abs(rot) > 0.5)
        {
            forw *= 0.4;
        }

        double leftPower = forw + rot;
        double rightPower = forw - rot;
        rightPower *= 0.85;

        double liftPower = -gamepad1.right_stick_y + 0.04;

         leftDrive.set(leftPower);
        rightDrive.set(rightPower);
          leftLift.set(liftPower);
         rightLift.set(liftPower);

        telemetry.addData("intakeDoor", intakeDoorState);
        telemetry.addData("depositorDoor", depositorDoorState);
        telemetry.update();

        depositorDoor.setPosition(depositorDoorState ? 0.93 : 0.45);
        intakeDoor.setPosition(intakeDoorState ? 1.0 : 0.45);

//        intake.set(intakeState ? 1.0 : 0.0);
    }
}
