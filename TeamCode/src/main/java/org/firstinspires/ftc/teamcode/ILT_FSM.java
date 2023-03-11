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

@TeleOp(name = "ILTFINAL", group = "linear opmode")

@Config
public class ILT_FSM extends LinearOpMode {

    //Consts, these consts right now are crude, as we don't have a bot rn ;-;
    public static int SLIDES_MAX_LENGTH = 870;
    private final int H_MAX = 700;
    private final int V_MAX = 870;
    public static double MOTOR_POWER = 0.5;
    public static double SMALLEST_CONE_THRESH = 1000.0;
    public static double LARGEST_CONE_THRESH = 2000.0;

    private final double CLAW_OPEN = 0.9;
    private final double CLAW_CLOSE = 0.4;
    private final double ELBOW_UP = 0.9;
    private final double ELBOW_DOWN = 0.55;
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


    // The "States" for the finite state Machine,
    // also known as FSM from now
    public enum cycle{
        RESET,
        SWEEP,
        GRABCONE,
        RETRACTHORVER,
        TRANSFER,
        EXTEND_HOR_VER,
        DEPOSITCONE,



    }
    cycle cycling = cycle.SWEEP;

    @Override
    public void runOpMode() throws InterruptedException {

        // This is the only time that we use this lmao what
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


        // Declare our motors
        // Make sure your ID's match your configuration
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

        while (opModeIsActive()) {
            switch (cycling){
                case RESET:
                    if (gamepad2.a) {
                        if (leftHoriz.getCurrentPosition() <= 8) {
                            h_pid = h_controller.calculate(leftHoriz.getCurrentPosition(), 0);
                            h_power = h_pid;
                            leftHoriz.setPower(h_power);
                            rightHoriz.setPower(h_power);
                        }

                        if (leftVert.getCurrentPosition() >= 8) {
                            v_pid = v_controller.calculate(leftVert.getCurrentPosition(), 0);


                            v_power = v_pid + ff;

                            leftVert.setPower(-v_pid - vkf);
                            rightVert.setPower(v_pid + vkf);

                        }

                        right_elbow.setPosition(1 - ELBOW_UP);
                        left_elbow.setPosition(ELBOW_UP);

                        claw.setPosition(CLAW_OPEN);
                    }


                    cycling = cycle.SWEEP;
                    break;

                case SWEEP:
                    if (!gamepad2.dpad_up)
                    {
                        break;
                    }
                    // Extend slides
                    if (gamepad2.x)
                    {
                        claw.setPosition(0.4);
                        left_elbow.setPosition(ELBOW_DOWN);
                        right_elbow.setPosition(1-ELBOW_DOWN);
                        wrist.setPosition((WRIST_UP-WRIST_DOWN)/2);
                        h_pid = h_controller.calculate(leftHoriz.getCurrentPosition(), 700);
                        v_pid = v_controller.calculate(leftVert.getCurrentPosition(), 700);
                        while (!gamepad1.dpad_left && leftHoriz.getCurrentPosition() <  H_MAX)
                        {
                            double latestContour = ContourPipeline.getLargestSize();
                            if (latestContour > SMALLEST_CONE_THRESH)
                            {

                                h_pid = h_controller.calculate(leftHoriz.getCurrentPosition(), leftHoriz.getCurrentPosition() + 10);
                                claw.setPosition(0.9);
                                cycling = cycle.GRABCONE;
                            }
                            //we have this temp variable for syntatic sugar as we add kP
                            h_power = h_pid;
                            leftHoriz.setPower(h_power);
                            rightHoriz.setPower(h_power);

                        }
                        claw.setPosition(0.4);
                        cycling = cycle.RESET;
                    }


                    break;
                // after you've grabbed cone, go back to home position!! REWRITTEN BY ALOK [check]
                case GRABCONE:
                    if(gamepad2.dpad_left){
                        cycling = cycle.SWEEP;
                        break;
                    }
                    v_pid = v_controller.calculate(leftVert.getCurrentPosition(), -5);
                    h_pid = h_controller.calculate(leftHoriz.getCurrentPosition(), -5);
                    leftHoriz.setPower(h_pid);
                    rightHoriz.setPower(h_pid);

                    leftVert.setPower(-v_pid - vkf);
                    rightVert.setPower(v_pid + vkf);
                    if(leftHoriz.getCurrentPosition() <= 10 && leftVert.getCurrentPosition() <= 10) {
                        left_elbow.setPosition(ELBOW_UP);
                        right_elbow.setPosition(1-ELBOW_UP);
                        servo0.setPosition(BUCKET_DOWN);
                        servo1.setPosition(BUCKET_UP);
                        // I would put an if statement here depicting each height of junction.
                        /*

                        if ( gamepad i) -> set height to 100 or cycling = cycle.low_junc



                         */
                        cycling = cycle.TRANSFER;
                    }

                    break;
                    // Extend time! extend vertical slides! remember to also move elbow down first - Aaron [check]
                case TRANSFER:

                    if(gamepad2.dpad_left)
                    {
                        cycling = cycle.SWEEP;
                        break;
                    }

                    wrist.setPosition(WRIST_DOWN);

                    claw.setPosition(CLAW_OPEN);

                    // just to get out of the way.
                    left_elbow.setPosition(0.7);
                    right_elbow.setPosition(0.3);
                    // 900

                    if (gamepad2.dpad_up) {
                        v_pid = v_controller.calculate(leftVert.getCurrentPosition(), V_MAX);
                        leftVert.setPower(-v_pid - vkf);
                        rightVert.setPower(v_pid + vkf);


                        cycling = cycle.EXTEND_HOR_VER;
                    }

                    break;
                case EXTEND_HOR_VER:
                    if(gamepad2.dpad_left){
                        cycling = cycle.RESET;
                        break;
                    }
                    h_pid = h_controller.calculate(leftHoriz.getCurrentPosition(), -5);
                    leftHoriz.setPower(h_pid);
                    rightHoriz.setPower(h_pid);


                    cycling = cycle.DEPOSITCONE;
                    break;





                case DEPOSITCONE:

                    if(gamepad2.dpad_left){
                        cycling = cycle.SWEEP;
                        break;
                    }
                    servo1.setPosition(BUCKET_DOWN);
                    servo0.setPosition(BUCKET_UP);
                    sleep(50);
                    servo1.setPosition(BUCKET_UP);
                    servo0.setPosition(BUCKET_DOWN);
                    cycling = cycle.RESET;
                    break;

            }

            double y = 0.55 * gamepad1.left_stick_y; // Remember, this is reversed!
            double x = -0.55 * gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -0.55 * gamepad1.right_stick_x;
            if(gamepad1.left_bumper)
            {
                y = 1.77 * y;
                x = 1.77 * x;
                rx = 1.77 * rx;
            }
            if(gamepad1.right_bumper)
            {
                y = 0.66 * y;
                x = 0.66 * x;
                rx = 0.66 * rx;
            }


            double botHeading = -imu.getAngularOrientation().firstAngle;
            // Read inverse IMU heading, as the IMU heading is CW positive
            if(gamepad1.a)
            {
                offset = -imu.getAngularOrientation().firstAngle;
            }
            if(gamepad1.b)
            {
                offset = 0;
            }
            double rotX = x * Math.cos(botHeading - offset) - y * Math.sin(botHeading - offset);
            double rotY = x * Math.sin(botHeading - offset) + y * Math.cos(botHeading - offset);


            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
            telemetry.addData("y speed:", y);
            telemetry.addData("x speed:", x);
            telemetry.update();
        }
    }
}




