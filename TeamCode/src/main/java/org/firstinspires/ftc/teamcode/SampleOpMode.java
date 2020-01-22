package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="SampleOpMode", group="Linear Opmode")
//@Disabled

// Acesta este programul principal. El combina toate elementele din celelalte programe
public class SampleOpMode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    double joyScale = 1;
    double motorMax = 0.6;
    double position = 0;

    boolean ok=false, apasat=false;
    boolean ok1=false, apasat1=false;

    boolean x=false,xx=false;

    @Override
    public void runOpMode() {

        robot  robot=new robot(hardwareMap);
        //drive drive=new drive(0,0,0,0);
        control control= new control();
        bratx bratx= new bratx(0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // aici facem drive-ul pe meccanum folosind clasa drive
            double X1, Y1, X2, Y2;
            Y1 = -gamepad1.right_stick_y * joyScale;
            X1 = gamepad1.right_stick_x * joyScale;
            Y2 = -gamepad1.left_stick_y * joyScale;
            X2 = gamepad1.left_stick_x * joyScale;

            control.drive(robot,X1,Y1,X2);
            telemetry.update();
            // pentru sistemul de colectare cu roti, B: strange cuburi, X: arunca cubul
            if (gamepad1.b && apasat == false) {
                if (ok == false) {
                    robot.colectare1.setPower(1);
                    robot.colectare2.setPower(-1);
                    apasat = true;
                    ok = true;
                } else {
                    robot.colectare1.setPower(0);
                    robot.colectare2.setPower(0);
                    ok = false;
                    apasat = true;
                }
            } else if (gamepad1.x && apasat1 == false) {

                if (ok1 == false) {
                    robot.colectare1.setPower(-1);
                    robot.colectare2.setPower(1);
                    apasat1 = true;
                    ok1 = true;
                } else {
                    robot.colectare1.setPower(0);
                    robot.colectare2.setPower(0);
                    ok1 = false;
                    apasat1 = true;
                }
            } else {
                if (!gamepad1.b) apasat = false;
                if (!gamepad1.x) apasat1 = false;
            }


            // aici facem bratele in X
            telemetry.update();
            if (gamepad1.right_bumper )
            {
                if(x==false)
                {
                    bratx.miscare(15);
                    x = true;
                    bratx.runToPos(robot);
                }
            }
            else if(gamepad1.left_bumper)
            {
                if(xx==false)
                {
                    bratx.miscare(-15);
                    xx = true;
                    bratx.runToPos(robot);
                }
            }
            else { x=false; xx=false;}
            if(gamepad1.a)robot.fnd1.setPower(1);
            else robot.fnd1.setPower(0);

            telemetry.update();

            //extinderea sistemului de colectare
            if(gamepad1.right_trigger!=0) robot.extindere1.setPower(1);
            else if(gamepad1.left_trigger!=0) robot.extindere1.setPower(-1);
            else robot.extindere1.setPower(0);


            //miscarea carligului
            robot.rotatiecarlig.setPosition(gamepad2.right_trigger);
            robot.carlig.setPosition(gamepad2.left_trigger);

            telemetry.update();

            telemetry.addData("Front left",control.getLF());
            telemetry.addData("Back left",control.getLB());
            telemetry.addData("Front right",control.getRF());
            telemetry.addData("Back right",control.getRB());
            telemetry.addData("v1",robot.arm1.getPower());
            telemetry.addData("v2",robot.arm2.getPower());
            telemetry.addData("arm1",robot.arm1.getCurrentPosition());
            telemetry.addData("arm2",robot.arm2.getCurrentPosition());
            telemetry.addData("tinta1",bratx.target1);
            telemetry.addData("tinta2",bratx.target2);
            telemetry.addData("extindere1",robot.extindere1.getPower());
            telemetry.addData("rotatiecarlig",robot.rotatiecarlig.getPosition());
            telemetry.addData("carlig",robot.carlig.getPosition());



            telemetry.update();
        }
    }
}