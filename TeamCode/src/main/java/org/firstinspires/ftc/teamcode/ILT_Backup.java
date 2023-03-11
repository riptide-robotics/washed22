package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.VisionPipelines.ContourPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "ilt backup!!", group = "linear opmode")

// comments will be sparse but will hopefuly provide a more general idea of what each portion is doing
public class ILT_Backup extends LinearOpMode{
    public static int SLIDES_MAX_LENGTH = 100;
    public static double MOTOR_POWER = 0.5;
    public static double SMALLEST_CONE_THRESH = 1000.0;
    public static double LARGEST_CONE_THRESH = 2000.0;

    private final double CLAW_OPEN = 0.9;
    private final double CLAW_CLOSE = 0.4;
    private final double ELBOW_UP = 0.9;
    private final double ELBOW_DOWN = 0.55;
    private final double ELBOW_MID = 0.5;
    private final double BUCKET_UP = 0.75;
    private final double BUCKET_DOWN = 0.05;
    private final double WRIST_DOWN = 0.7;
    private final double WRIST_UP = 0.25;


    //Add the kp in your code.
    private final double vkf = 0.04;

    PIDController h_controller;
    PIDController v_controller;

    OpenCvWebcam camera = null;
    // I won't touch this but ?? what's this lmao
    ElapsedTime iltBotStuff = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException
    {
        iltBotStuff.reset();

        h_controller = new PIDController(0.001,0.009, 0);
        v_controller = new PIDController(0.008, 0.003, 0);

        WebcamName webcamname = hardwareMap.get(WebcamName.class, "Webcam");
        // Acquire the camera ID
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName() );
        //set the cam name and id to the webcam.
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamname, cameraMonitorViewId);
        camera.setPipeline(new ContourPipeline());
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Webcam not working");
            }
        });
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("leftFront");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("leftRear");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("rightFront");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("rightRear");
        DcMotor leftHoriz = hardwareMap.dcMotor.get("leftHorizontal");
        DcMotor rightHoriz = hardwareMap.dcMotor.get("slides2");
        DcMotor leftVert = hardwareMap.dcMotor.get("slides3");
        DcMotor rightVert = hardwareMap.dcMotor.get("slides4");

        Servo servo0 = hardwareMap.servo.get("servo0");
        Servo servo1 = hardwareMap.servo.get("servo1");
        Servo left_elbow = hardwareMap.servo.get("left_elbow");
        Servo right_elbow = hardwareMap.servo.get("right_elbow");
        Servo wrist = hardwareMap.servo.get("wrist");
        Servo claw = hardwareMap.servo.get("claw");

        leftHoriz.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightHoriz.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftVert.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightVert.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftHoriz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftHoriz.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftVert.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        double offset = 0;


        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        double h_pid;
        double v_pid;
        double h_power;
        double v_power;
        double ff = vkf;

        waitForStart();

        if (isStopRequested()) return;

        while(opModeIsActive())
        {
            //control the slides!!
            if(gamepad2.a)
            {
                leftHoriz.setPower(0.9);
                rightHoriz.setPower(0.9);
            }
            else if(gamepad2.b)
            {
                leftHoriz.setPower(-0.9);
                rightHoriz.setPower(-0.9);
            }
            if(gamepad2.x)
            {
                leftVert.setPower(-0.9);
                rightVert.setPower(0.9);
            }
            else if(gamepad2.y)
            {
                leftVert.setPower(0.9);

            }
        }


    }

}
