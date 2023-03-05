package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
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

@TeleOp(name = "ilt fsm", group = "linear opmode")

@Config
public class ILT_FSM extends LinearOpMode {

    //Consts, these consts right now are crude, as we don't have a bot rn ;-;
    public static int SLIDES_MAX_LENGTH = 900;
    public static double MOTOR_POWER = 0.5;
    public static double SMALLEST_CONE_THRESH = 1000.0;
    public static double LARGEST_CONE_THRESH = 2000.0;


    OpenCvWebcam camera = null;
    // I won't touch this but ?? what's this lmao
    ElapsedTime iltBotStuff = new ElapsedTime();


    // The "States" for the finite state Machine,
    // also known as FSM from now
    public enum cycle{
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
        DcMotor leftHoriz = hardwareMap.dcMotor.get("leftHoriz");
        DcMotor rightHoriz = hardwareMap.dcMotor.get("rightHoriz");
        DcMotor leftVert = hardwareMap.dcMotor.get("leftVert");
        DcMotor rightVert = hardwareMap.dcMotor.get("rightVert");
        //run all motors using the encoders

        leftHoriz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftHoriz.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightHoriz.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightHoriz.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftVert.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightVert.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Servo vertServo = hardwareMap.servo.get("servo0");
        Servo vertServo2 = hardwareMap.servo.get("servo1");
        Servo armServo = hardwareMap.servo.get("servo2");
        Servo armServo2 = hardwareMap.servo.get("servo3");
        Servo clawServo = hardwareMap.servo.get("servo4");
        Servo clawServo2 = hardwareMap.servo.get("servo5");
        Servo claw = hardwareMap.servo.get("servo6");


        leftHoriz.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightHoriz.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftVert.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightVert.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            switch (cycling){
                case SWEEP:
                    if (!gamepad2.dpad_up)
                    {
                        break;
                    }

                    while (!gamepad1.dpad_left && leftHoriz.getCurrentPosition() <  SLIDES_MAX_LENGTH )
                    {
                        double latestContour = ContourPipeline.getLargestSize();
                        if (latestContour > SMALLEST_CONE_THRESH && latestContour < LARGEST_CONE_THRESH) {
                            claw.setPosition(1);
                            leftHoriz.setTargetPosition(leftHoriz.getCurrentPosition());
                            rightHoriz.setTargetPosition(rightHoriz.getCurrentPosition());
                            cycling = cycle.GRABCONE;
                            break;
                        }
                        /*
                        The setTargetPosition from here and on is temperary, we are implementing PID loops

                        how the PID loop will work is this

                        Check if the cone is in range, ( already done from line 125 to 130)
                        if cone is in range, then set target position is
                        Set target position to MAX_RANGE note that max range is a little less than the actual max range
                        start moving. This way we can stop the PID loop on its way to the max and we avoid juttering too much.
                         */

                        leftHoriz.setTargetPosition(SLIDES_MAX_LENGTH);
                        rightHoriz.setTargetPosition(SLIDES_MAX_LENGTH);
                        leftHoriz.setPower(MOTOR_POWER);
                        rightHoriz.setPower(MOTOR_POWER);

                    }

                    break;

                case GRABCONE:
                    if(gamepad2.dpad_left){
                        cycling = cycle.SWEEP;
                        break;
                    }
                    if ((leftVert.getCurrentPosition() != 0 && rightVert.getCurrentPosition() != 0)) {
                        leftVert.setTargetPosition(0);
                        rightVert.setTargetPosition(0);
                        leftVert.setPower(-MOTOR_POWER);

                        rightVert.setPower(-MOTOR_POWER);
                        leftHoriz.setTargetPosition(0);
                        rightHoriz.setTargetPosition(0);
                        leftHoriz.setPower(-MOTOR_POWER);
                        rightHoriz.setPower(-MOTOR_POWER);
                        cycling = cycle.EXTEND_HOR_VER;
                    }
                    else if(leftVert.getCurrentPosition() == 0 && rightVert.getCurrentPosition() == 0)
                    {
                        leftHoriz.setTargetPosition(0);
                        rightHoriz.setTargetPosition(0);
                        leftHoriz.setPower(-MOTOR_POWER);
                        rightHoriz.setPower(-MOTOR_POWER);
                        cycling = cycle.RETRACTHORVER;
                    }
                    else{
                        cycling = cycle.SWEEP;
                    }

                    break;

                case TRANSFER:

                    if(gamepad2.dpad_left){
                        cycling = cycle.SWEEP;
                        break;
                    }
                    if (leftVert.getCurrentPosition() == 0 && rightVert.getCurrentPosition() == 0 && armServo.getPosition() == 1) {
                            armServo.setPosition(0);
                            armServo2.setPosition(1);
                            clawServo.setPosition(0);
                            clawServo2.setPosition(1);
                            claw.setPosition(0);



                            cycling = cycle.EXTEND_HOR_VER;
                        }
                        else {
                            cycling = cycle.GRABCONE;
                        }

                    break;
                case EXTEND_HOR_VER:
                    if(gamepad2.dpad_left){
                        cycling = cycle.SWEEP;
                        break;
                    }

                    if (armServo.getPosition() == 0 && leftVert.getCurrentPosition() == 0 && rightVert.getCurrentPosition() == 0) {
                        armServo.setPosition(1);
                        armServo2.setPosition(0);
                        clawServo.setPosition(1);
                        clawServo2.setPosition(0);
                        leftVert.setTargetPosition(SLIDES_MAX_LENGTH);
                        rightVert.setTargetPosition(SLIDES_MAX_LENGTH);
                        leftVert.setPower(MOTOR_POWER);
                        rightVert.setPower(MOTOR_POWER);
                        while (!gamepad1.dpad_left && leftHoriz.getCurrentPosition() < SLIDES_MAX_LENGTH) {
                            double latestContour = ContourPipeline.getLargestSize();
                            if (latestContour > SMALLEST_CONE_THRESH && latestContour < LARGEST_CONE_THRESH) {
                                claw.setPosition(1);
                                leftHoriz.setTargetPosition(leftHoriz.getCurrentPosition());
                                rightHoriz.setTargetPosition(rightHoriz.getCurrentPosition());

                                break;
                            }
                        /*
                        The setTargetPosition from here and on is temperary, we are implementing PID loops

                        how the PID loop will work is this

                        Check if the cone is in range, ( already done from line 125 to 130)
                        if cone is in range, then set target position is
                        Set target position to MAX_RANGE note that max range is a little less than the actual max range
                        start moving. This way we can stop the PID loop on its way to the max and we avoid juttering too much.
                         */

                            leftHoriz.setTargetPosition(SLIDES_MAX_LENGTH);
                            rightHoriz.setTargetPosition(SLIDES_MAX_LENGTH);
                            leftHoriz.setPower(MOTOR_POWER);
                            rightHoriz.setPower(MOTOR_POWER);

                            cycling = cycle.DEPOSITCONE;
                        }
                    }
                    else
                    {
                        cycling = cycle.TRANSFER;
                    }
                    break;

                case DEPOSITCONE:

                    if(gamepad2.dpad_left){
                        cycling = cycle.SWEEP;
                        break;
                    }
                    if (armServo.getPosition() == 0 && leftVert.getCurrentPosition() != 0 && rightVert.getCurrentPosition() != 0) {
                        vertServo.setPosition(1);
                        vertServo2.setPosition(0);
                        sleep(50);
                        vertServo.setPosition(0);
                        vertServo2.setPosition(1);
                        leftVert.setTargetPosition(0);
                        rightVert.setTargetPosition(0);
                        leftVert.setPower(-MOTOR_POWER);
                        rightVert.setPower(-MOTOR_POWER);
                        cycling = cycle.SWEEP;
                    }
                    else
                    {
                        cycling = cycle.EXTEND_HOR_VER;
                    }

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

        }
    }
}




