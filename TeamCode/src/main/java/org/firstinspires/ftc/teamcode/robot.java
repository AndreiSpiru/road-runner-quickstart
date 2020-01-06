package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

// Aici facem initializarea tuturor partilor robotului.
public class robot  {
    public DcMotor leftFront =null;
    public DcMotor leftBack =null;
    public DcMotor rightFront =null;
    public DcMotor rightBack =null;

    public DcMotor arm2 =null;
    public DcMotor arm1 =null;

    public DcMotor colectare1=null;
    public DcMotor colectare2=null;

    public CRServo extindere1= null;

    public Servo rotatiecarlig= null;
    public Servo carlig= null;

    public BNO055IMU IMU;
    public Orientation angles;
    public Acceleration gravity;
    robot(HardwareMap hardwareMap)
    {

        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");


        colectare1  = hardwareMap.get(DcMotor.class, "colectare1");
        colectare2 = hardwareMap.get(DcMotor.class, "colectare2");

        arm2 = hardwareMap.get(DcMotor.class, "arm2");
        arm1 = hardwareMap.get(DcMotor.class, "arm1");

        extindere1 = hardwareMap.get(CRServo.class, "extindere1");

        rotatiecarlig = hardwareMap.get(Servo.class, "rotatiecarlig");
        carlig = hardwareMap.get(Servo.class, "carlig");


        BNO055IMU.Parameters Bparameters = new BNO055IMU.Parameters();
        Bparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        Bparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        Bparameters.calibrationDataFile = "BNO055IMUCalibration.json";
        Bparameters.loggingEnabled = true;
        Bparameters.loggingTag = "IMU";
        Bparameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        IMU = hardwareMap.get(BNO055IMU.class, "IMU");
        IMU.initialize(Bparameters);
        angles   = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity  = IMU.getGravity();


        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        arm2.setDirection(DcMotor.Direction.FORWARD);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm1.setDirection(DcMotor.Direction.FORWARD);
        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


}