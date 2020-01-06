package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name = "Mecanum Drive", group = "TeleOp")
//FTC Team 11848 | Spare Parts Robotics
public class MecanumDrive extends LinearOpMode
{
    robot robot;
    //Variables
    public double leftStickY;
    public double leftStickX;
    public double rightStickX;
    public double FL_power_raw;
    public double FR_power_raw;
    public double RL_power_raw;
    public double RR_power_raw;
    public double FL_power;
    public double FR_power;
    public double RL_power;
    public double RR_power;

    public double newForward;
    public double newStrafe;

    // State used for updating telemetry
    public Orientation angles;
    public Acceleration gravity;

    @Override
    public void runOpMode()
    {
        robot = new robot(hardwareMap);

        //Wait for the start button to be pressed.
        waitForStart();

        while (opModeIsActive())
        {
            controls();
            telemetry.addData("LF",robot.leftFront.getPower());
            telemetry.addData("RF",robot.rightFront.getPower());
            telemetry.addData("LB",robot.leftBack.getPower());
            telemetry.addData("RB",robot.rightBack.getPower());
            telemetry.update();
        }

    }

    public void controls()
    {
        angles = robot.IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("LF",robot.leftFront.getPower());
        telemetry.addData("RF",robot.rightFront.getPower());
        telemetry.addData("LB",robot.leftBack.getPower());
        telemetry.addData("RB",robot.rightBack.getPower());
        telemetry.update();

        holonomicFormula();
        setDriveChainPower();
    }

    public void getJoyValues()
    {
        leftStickY = gamepad1.left_stick_y;
        leftStickX = gamepad1.left_stick_x;
        rightStickX = gamepad1.right_stick_x;

        float pi = 3.1415926f;

        float gyro_degrees = angles.firstAngle;
        float gyro_radians = gyro_degrees * pi/180;
        newForward = leftStickY * Math.cos(gyro_radians) + leftStickX * Math.sin(gyro_radians);
        newStrafe = -leftStickY * Math.sin(gyro_radians) + leftStickX * Math.cos(gyro_radians);
        telemetry.addData("LF",robot.leftFront.getPower());
        telemetry.addData("RF",robot.rightFront.getPower());
        telemetry.addData("LB",robot.leftBack.getPower());
        telemetry.addData("RB",robot.rightBack.getPower());
        telemetry.update();
    }

    public void holonomicFormula()
    {
        getJoyValues();

        FL_power_raw = newForward - newStrafe - rightStickX;
        FR_power_raw = newForward + newStrafe + rightStickX;
        RL_power_raw = newForward + newStrafe - rightStickX;
        RR_power_raw = newForward - newStrafe + rightStickX;

        FL_power = Range.clip(FL_power_raw, -1, 1);
        FR_power = Range.clip(FR_power_raw, -1, 1);
        RL_power = Range.clip(RL_power_raw,-1 ,1);
        RR_power = Range.clip(RR_power_raw, -1, 1);
        telemetry.addData("LF",robot.leftFront.getPower());
        telemetry.addData("RF",robot.rightFront.getPower());
        telemetry.addData("LB",robot.leftBack.getPower());
        telemetry.addData("RB",robot.rightBack.getPower());
        telemetry.update();
    }

    public void setDriveChainPower()
    {
        robot.leftFront.setPower(FL_power);
        robot.rightFront.setPower(FR_power);
        robot.leftBack.setPower(RL_power);
        robot.rightBack.setPower(RR_power);
        telemetry.addData("LF",robot.leftFront.getPower());
        telemetry.addData("RF",robot.rightFront.getPower());
        telemetry.addData("LB",robot.leftBack.getPower());
        telemetry.addData("RB",robot.rightBack.getPower());
        telemetry.update();
    }
}