package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class bratx {
    public int position=0;
    public int directie=0;
    public int target1=0;
    public int target2=0;//14252;

    double COUNTS_PER_MOTOR_REV1=1993.6;
    double count_per_mm1=COUNTS_PER_MOTOR_REV1/8 ;

    double COUNTS_PER_MOTOR_REV2=1425.2;
    double count_per_mm2=COUNTS_PER_MOTOR_REV2/8;

    bratx(int position1)
    {
        position=position1;
    }

    void miscare(int delta)
    {
        position+=delta;
        if(delta<0) directie=-1;
        else directie=1;
    }

    void runToPos(robot robot)
    {
        target1=(int)(position*count_per_mm1);
        //target2=(int)((target1*0.521)*(5.0/4.0));
        target2=(int)(position*count_per_mm2);

        robot.arm1.setTargetPosition(target1);
        robot.arm2.setTargetPosition(target2);

        robot.arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.arm2.setPower(0.521);
        robot.arm1.setPower(1);
    }

}