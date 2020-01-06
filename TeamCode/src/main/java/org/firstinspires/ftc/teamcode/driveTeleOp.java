package org.firstinspires.ftc.teamcode;

// aici actualizam puterile petru motoarele de drive si le si aplicam
public class driveTeleOp {
    private double LF,RB,LB,RF;
    //costructor
    driveTeleOp (double X1, double Y1, double X2, double Y2)
    {
        LF = RF = LB = RB = 0;
    }

    public void calculatePower(double X1, double Y1, double X2, double Y2)
    {
        // Forward/back movement
        LF = RF = LB = RB = 0;
        LF += Y1; RF += Y1; LB += Y1; RB += Y1;

        // Side to side movement
        LF += X1; RF -= X1; LB -= X1; RB += X1;

        // Rotation movement
        LF += X2; RF -= X2; LB += X2; RB -= X2;


    }
    //getting motor power
    public double getLF()
    {
        return LF;
    }
    public double getRB()
    {
        return RB;
    }
    public double getLB()
    {
        return LB;
    }
    public double getRF()
    {
        return RF;
    }

    //setting motor power
    public void setLF (double newLF)
    {
        LF=newLF;
    }
    public void setRF (double newRF)
    {
        RF=newRF;
    }
    public void setLB (double newLB)
    {
        LB=newLB;
    }
    public void setRB (double newRB)
    {
        RB=newRB;
    }


    public  void scaleDownAmount(double motorMax)
    {
        // Clip motor power values to +-motorMax
        LF = Math.max(-motorMax, Math.min(LF, motorMax));
        RF = Math.max(-motorMax, Math.min(RF, motorMax));
        LB = Math.max(-motorMax, Math.min(LB, motorMax));
        RB = Math.max(-motorMax, Math.min(RB, motorMax));
    }

    static public void setPwr(double lb, double lf, double rb, double rf, robot robot)
    {
        robot.leftBack.setPower(lb);
        robot.leftFront.setPower(lf);
        robot.rightBack.setPower(rb);
        robot.rightFront.setPower(rf);
    }
}