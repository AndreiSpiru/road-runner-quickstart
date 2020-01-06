package org.firstinspires.ftc.teamcode;/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
// clasa pentru bratul luui Cristi
@Disabled
public class brat {
    private double power;
    private int position;
    private int delta;

    //constructor
    brat (double inputPower)
    {
        power = inputPower;
    }
    //getter
    public double getPower()
    {
        return power;
    }
    public int getPosition(robot robot)
    {
        return position;
    }

    // setter
    public void setPower(double inputPower)
    {
        power = inputPower;
    }
    public void setPosition(int inputPosition)
    {
        position = inputPosition;
    }

    // send power to motor
    public void setPwr(robot robot)
    {
        robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.arm.setPower(power/5);
    }
    //get encoder position
    public void setTargetPosition(robot robot)
    {
        position = robot.arm.getCurrentPosition();
    }
    // try to hold last recorded position
    public void holdPosition(robot robot)
    {
        robot.arm.setTargetPosition(position);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        delta = position - robot.arm.getCurrentPosition();
        if(Math.abs(delta) <= 25) robot.arm.setPower(0);
        else if(Math.abs(delta) <= 200)robot.arm.setPower(0.1);
        else robot.arm.setPower(0.2);
    }
}*/