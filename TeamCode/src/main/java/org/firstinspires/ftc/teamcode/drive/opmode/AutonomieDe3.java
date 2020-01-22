package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Webcam;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.opencvSkystoneDetector;
import org.firstinspires.ftc.teamcode.robot;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Config
@Autonomous(group = "drive")
public  class AutonomieDe3 extends LinearOpMode {
    public static int poz = 2,tava =0;
    public ElapsedTime runtime = new ElapsedTime();

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    public static int valMid = -1;
    public static int valLeft = -1;
    public static int valRight = -1;

    public static float rectHeight = .6f/8f;
    public static float rectWidth = 1.5f/8f;

    public static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    public static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    public static float[] midPos = {4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    public static float[] leftPos = {2f/8f+offsetX, 4f/8f+offsetY};
    public static float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    public final int rows = 640;
    public final int cols = 480;

    OpenCvCamera phoneCam;
    public CRServo fnd1=null,fnd2=null;
    public DcMotor leftFront=null,leftBack=null,rightFront=null,rightBack=null;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);

        fnd1 = hardwareMap.get(CRServo.class,"fnd1");
        fnd2 = hardwareMap.get(CRServo.class,"fnd2");
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        /*int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new AutonomieDe3.StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC



        if (isStopRequested()) return;

        boolean ok=true;

        if(valLeft == 0&&valMid == 255&&valRight == 255)poz=1;
        else if(valLeft == 255&&valMid == 0&&valRight == 255)poz=2;
        else if(valLeft == 255&&valMid == 255&&valRight == 0)poz=3;
        telemetry.addData("Values", valLeft+"   "+valMid+"   "+valRight);
        telemetry.addData("poz", poz);*/
        moveClaw(-1,350);

        waitForStart();
        if (isStopRequested()) return;
        telemetry.update();
        runtime.reset();


        if(poz == 1) {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .splineTo(new Pose2d(27, 8.5, 0))
                            .build()

            );
            moveClaw(-1,600);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .reverse()
                            .forward(4.5)
                            .build()

            );
            drive.turnSync(Math.toRadians(91));
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(48)
                            .build()
            );
        }
        else if(poz== 2)
        {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(27)
                            .build()
            );
            moveClaw(-1,600);
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .reverse()
                            .forward(6)
                            .build()
            );

            drive.turnSync(Math.toRadians(90));
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(55.5)
                            .build()
            );
        }
        else if(poz ==3)
        {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .splineTo(new Pose2d(27, -8, 0))
                            .build()
            );
            moveClaw(-1,600);
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .reverse()
                            .forward(4.5)
                            .build()
            );

            drive.turnSync(Math.toRadians(90));
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(65.5)
                            .build()
            );
        }

        moveClaw(1,350);
        if(poz==1) {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .reverse()
                            .forward(74)
                            .build()
            );
            drive.turnSync(Math.toRadians(-90));
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()

                            .forward(5.5)
                            .build()
            );
            moveClaw(-1,350);
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .reverse()
                            .forward(7)
                            .build()
            );
            drive.turnSync(Math.toRadians(91));
            if(tava == 0)
            {
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .forward(74)
                                .build()
                );

                moveClaw(1,350);

                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .reverse()
                                .forward(56)
                                .build()
                );

                drive.turnSync(Math.toRadians(-90));
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()

                                .forward(4)
                                .build()
                );
                moveClaw(-1,500);

                drive.turnSync(Math.toRadians(103));
                vaj(1,2000);
                vaj(-1,500);
            }

        }
        else if(poz==2)
        {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .reverse()
                            .forward(73.5)
                            .build()
            );

            drive.turnSync(Math.toRadians(-90));
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()

                            .forward(7)
                            .build()
            );
            moveClaw2(1,350);
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .reverse()
                            .forward(8.5)
                            .build()
            );
            drive.turnSync(Math.toRadians(91));
            if(tava == 0) {
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .forward(74)
                                .build()
                );

                moveClaw2(-1,350);

                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .reverse()
                                .forward(47)
                                .build()
                );

                drive.turnSync(Math.toRadians(-90));
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()

                                .forward(5)
                                .build()
                );
                moveClaw(-1,400);

                drive.turnSync(Math.toRadians(103));
                vaj(1,1700);
                vaj(-1,500);
            }

        }

        else if(poz==3)
        {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .reverse()
                            .forward(73.5)
                            .build()
            );

            drive.turnSync(Math.toRadians(-135));
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()

                            .forward(5.5)
                            .build()
            );
            moveClaw2(1,350);
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .reverse()
                            .forward(7)
                            .build()
            );
            drive.turnSync(Math.toRadians(137));
            if(tava == 0) {
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .forward(74)
                                .build()
                );

                moveClaw2(-1, 350);
            }

        }

    }

    public void moveClaw(int direction,double time)
    {
        runtime.reset();
        while(runtime.time(TimeUnit.MILLISECONDS)<=time)fnd2.setPower(direction);
        fnd2.setPower(0);
    }
    public void moveClaw2(int direction,double time)
    {
        runtime.reset();
        while(runtime.time(TimeUnit.MILLISECONDS)<=time)fnd1.setPower(direction);
        fnd1.setPower(0);
    }
    public void moveClawboth(int direction,double time)
    {
        runtime.reset();
        while(runtime.time(TimeUnit.MILLISECONDS)<=time){fnd2.setPower(direction);fnd1.setPower(-direction);}

    }
    public void vaj(int direction,double time)
    {
        runtime.reset();
        while(runtime.time(TimeUnit.MILLISECONDS)<=time) {
            if(direction == 1)fnd2.setPower(0.3);
            leftBack.setPower(direction);
            leftFront.setPower(direction);
            rightBack.setPower(direction);
            rightFront.setPower(direction);
        }

    }
    public static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage
        {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        public AutonomieDe3.StageSwitchingPipeline.Stage stageToRenderToViewport = AutonomieDe3.StageSwitchingPipeline.Stage.detection;
        public AutonomieDe3.StageSwitchingPipeline.Stage[] stages = AutonomieDe3.StageSwitchingPipeline.Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2),
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2),
                            input.rows()*(midPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport)
            {
                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case detection:
                {
                    return all;
                }

                case RAW_IMAGE:
                {
                    return input;
                }

                default:
                {
                    return input;
                }
            }
        }

    }
}
