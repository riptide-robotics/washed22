package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.VisionPipelines.ContourPipeline;
import org.firstinspires.ftc.teamcode.VisionPipelines.WebcamPipelineNotes;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "ILTFINAL", group = "linear opmode")
public class ILT_FSM extends LinearOpMode {
    OpenCvWebcam camera = null;
    ElapsedTime iltBotStuff = new ElapsedTime();

    public enum cycle{
        START,
        EXTENDEDHOR,
        GRABCONE,
        RETRACTHORVER,
        TRANFER,
        EXTENDHORVER,
        DEPOSITCONE,

    }
    cycle cycling = cycle.START;

    @Override
    public void runOpMode() throws InterruptedException {
        iltBotStuff.reset();
        // Declare our motors
        // Make sure your ID's match your configuration
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



        // Reverse thv e right side motors
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
                case START:
                    if(gamepad2.dpad_up){
                        while (!gamepad2.dpad_left) {
                            if(gamepad2.dpad_left){
                                cycling = cycle.START;
                                break;
                            }
                            if (leftHoriz.getCurrentPosition() < 100 && rightHoriz.getCurrentPosition() < 100) {
                                leftHoriz.setTargetPosition(100);
                                rightHoriz.setTargetPosition(100);
                                leftHoriz.setPower(0.5);
                                rightHoriz.setPower(0.5);
                                if(gamepad2.dpad_left){
                                    cycling = cycle.START;
                                    break;
                                }

                            }
                            if(gamepad2.dpad_left){
                                break;
                            }
                            cycling = cycle.EXTENDEDHOR;
                        }

                    }
                    break;
                case EXTENDEDHOR:
                    double lc = ContourPipeline.getLargestSize();
                    while (!gamepad2.dpad_left) {
                        if(gamepad2.dpad_left){
                            cycling = cycle.START;
                            break;
                        }
                        if ((lc < 2000 && lc > 1000) || leftHoriz.getCurrentPosition() <= 100) {
                            claw.setPosition(1);
                            cycling = cycle.GRABCONE;
                        }
                        else{
                            cycling = cycle.START;
                        }
                        if(gamepad2.dpad_left){
                            cycling = cycle.START;
                            break;
                        }

                    }

                    break;
                case GRABCONE:
                    while (!gamepad2.dpad_left) {
                        if(gamepad2.dpad_left){
                            cycling = cycle.START;
                            break;
                        }
                        if ((leftVert.getCurrentPosition() != 0 && rightVert.getCurrentPosition() != 0)) {
                            leftVert.setTargetPosition(0);
                            rightVert.setTargetPosition(0);
                            leftVert.setPower(-0.5);
                            rightVert.setPower(-0.5);
                            leftHoriz.setTargetPosition(0);
                            rightHoriz.setTargetPosition(0);
                            leftHoriz.setPower(-0.5);
                            rightHoriz.setPower(-0.5);
                            cycling = cycle.EXTENDHORVER;
                        }
                        else if(leftVert.getCurrentPosition() == 0 && rightVert.getCurrentPosition() == 0){
                            leftHoriz.setTargetPosition(0);
                            rightHoriz.setTargetPosition(0);
                            leftHoriz.setPower(-0.5);
                            rightHoriz.setPower(-0.5);
                            cycling = cycle.RETRACTHORVER;
                        }
                        else{
                            cycling = cycle.EXTENDEDHOR;
                        }
                        if(gamepad2.dpad_left){
                            cycling = cycle.START;
                            break;
                        }
                    }

                    break;

                case TRANFER:
                    while(!gamepad2.dpad_left) {
                        if(gamepad2.dpad_left){
                            cycling = cycle.START;
                            break;
                        }
                        if (leftVert.getCurrentPosition() == 0 && rightVert.getCurrentPosition() == 0 && armServo.getPosition() == 1) {
                            armServo.setPosition(0);
                            armServo.setPosition(1);
                            clawServo.setPosition(0);
                            clawServo2.setPosition(1);
                            claw.setPosition(0);
                            cycling = cycle.EXTENDHORVER;
                        }
                        else {
                            cycling = cycle.GRABCONE;
                        }
                        if(gamepad2.dpad_left){
                            cycling = cycle.START;
                            break;
                        }

                    }
                    break;
                case EXTENDHORVER:
                    while (!gamepad2.dpad_left) {
                        if(gamepad2.dpad_left){
                            cycling = cycle.START;
                            break;
                        }
                        if (armServo.getPosition() == 0 && leftVert.getCurrentPosition() == 0 && rightVert.getCurrentPosition() == 0) {
                            leftVert.setTargetPosition(100);
                            rightVert.setTargetPosition(100);
                            leftVert.setPower(0.5);
                            rightVert.setPower(0.5);
                            leftHoriz.setTargetPosition(100);
                            rightHoriz.setTargetPosition(100);
                            leftHoriz.setPower(0.5);
                            rightHoriz.setPower(0.5);
                            armServo.setPosition(1);
                            armServo2.setPosition(0);
                            clawServo.setPosition(1);
                            clawServo2.setPosition(0);
                            cycling = cycle.DEPOSITCONE;
                        } else {
                            cycling = cycle.GRABCONE;
                        }
                        if(gamepad2.dpad_left){
                            cycling = cycle.START;
                            break;
                        }
                    }
                    break;
                case DEPOSITCONE:
                    while ((!gamepad2.dpad_left)) {
                        if(gamepad2.dpad_left){
                            cycling = cycle.START;
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
                            leftVert.setPower(-0.5);
                            rightVert.setPower(-0.5);
                            cycling = cycle.START;
                        } else {
                            cycling = cycle.EXTENDHORVER;
                        }
                        if(gamepad2.dpad_left){
                            cycling = cycle.START;
                            break;
                        }

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
            telemetry.addData("y speed:", y);
            telemetry.addData("x speed:", x);
            telemetry.update();
        }
    }
}




