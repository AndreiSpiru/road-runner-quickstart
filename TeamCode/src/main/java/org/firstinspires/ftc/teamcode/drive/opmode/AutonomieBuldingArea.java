package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

import java.util.concurrent.TimeUnit;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class AutonomieBuldingArea extends LinearOpMode {

    public ElapsedTime runtime = new ElapsedTime();
    public CRServo fnd1=null,fnd2=null;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);

        fnd1 = hardwareMap.get(CRServo.class,"fnd1");
        fnd2 = hardwareMap.get(CRServo.class,"fnd2");

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(26, 24, 0))
                        .build()

        );
        moveClawboth(-1,600);
        sleep(500);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(0, 0, 90))
                        .build()

        );
        moveClawboth(1,600);
       drive.turnSync(Math.toRadians(35));
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                        .forward(30)
                        .build()

        );

    }
    public void moveClawboth(int direction,double time)
    {
        runtime.reset();
        while(runtime.time(TimeUnit.MILLISECONDS)<=time){fnd2.setPower(direction);fnd1.setPower(-direction);}

    }
}
