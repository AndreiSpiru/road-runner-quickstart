package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class control {
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
    control ()
    {

    }
    public void drive(robot robot,double x1, double y1, double x2)
    {
        angles = robot.IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        holonomicFormula(robot,x1,y1,x2);
        setDriveChainPower(robot,x1,y1,x2);
    }

    public void getJoyValues(robot robot,double x1, double y1, double x2)
    {
        leftStickY = y1;
        leftStickX = x1;
        rightStickX = x2;

        float pi = 3.1415926f;

        float gyro_degrees = angles.firstAngle;
        float gyro_radians = gyro_degrees * pi/180;
        newForward = leftStickY * Math.cos(gyro_radians) + leftStickX * Math.sin(gyro_radians);
        newStrafe = -leftStickY * Math.sin(gyro_radians) + leftStickX * Math.cos(gyro_radians);

    }

    public void holonomicFormula(robot robot,double x1, double y1, double x2)
    {
        getJoyValues(robot,x1,y1,x2);

        FL_power_raw = newForward - newStrafe - rightStickX;
        FR_power_raw = newForward + newStrafe + rightStickX;
        RL_power_raw = newForward + newStrafe - rightStickX;
        RR_power_raw = newForward - newStrafe + rightStickX;

        FL_power = Range.clip(FL_power_raw, -1, 1);
        FR_power = Range.clip(FR_power_raw, -1, 1);
        RL_power = Range.clip(RL_power_raw,-1 ,1);
        RR_power = Range.clip(RR_power_raw, -1, 1);

    }

    public void setDriveChainPower(robot robot,double x1, double y1, double x2)
    {
        robot.leftFront.setPower(FL_power);
        robot.rightFront.setPower(FR_power);
        robot.leftBack.setPower(RL_power);
        robot.rightBack.setPower(RR_power);

    }
    public double getLF()
    {
        return FL_power;
    }
    public double getLB()
    {
        return RL_power;
    }
    public double getRF()
    {
        return FR_power;
    }
    public double getRB()
    {
        return RR_power;
    }
}
