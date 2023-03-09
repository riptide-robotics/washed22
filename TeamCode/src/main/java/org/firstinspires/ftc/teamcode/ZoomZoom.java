package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.VisionPipelines.ContourPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

// blah blah blah


@TeleOp(name="calibration testing", group="Linear Opmode")
// decorator so the robot knows what mode to operate in
//vertical encoder max 933 do 900
//horizontal encoder max 873 do 860
@Config
public class ZoomZoom extends LinearOpMode {

    // Declare teleOp members.
    private ElapsedTime runtime = new ElapsedTime();
    public static double out = 0.5;
    public static double elbow_val = 0.7;
    public static double wrist_val = 0.5;
    public static double claw_val = 0.5;
    public static int vertical_movement = 0;
    public static double power = 0;
    public static int horizontal_movement = 0;
    public static int encoder_pos = 0;


    // webcam threshold
    public static double threshold = 0;
    OpenCvWebcam webcam = null;

    @Override
    // related to inheritance: runOpMode is a necessary function as you NEED to override the runOpMode in the superclass
    public void runOpMode() {
        //System.out on the phones = telemetry
        // Elbow range is from 1 to 0.5
        Servo servo0 = hardwareMap.servo.get("servo0out");
        Servo servo1 = hardwareMap.servo.get("servo1out");
        Servo left_elbow = hardwareMap.servo.get("left_elbow");
        Servo claw = hardwareMap.servo.get("claw");
        Servo right_elbow = hardwareMap.servo.get("right_elbow");
        //Servo servo5 = hardwareMap.servo.get("servo5wrist");
        Servo wrist = hardwareMap.servo.get("wrist");
        DcMotor leftHoriz = hardwareMap.dcMotor.get("leftHoriz");
        DcMotor rightHoriz = hardwareMap.dcMotor.get("rightHoriz");
        DcMotor rightVert = hardwareMap.dcMotor.get("rightVert");
        DcMotor leftVert = hardwareMap.dcMotor.get("leftVert");
        rightVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightVert.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftVert.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftHoriz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftHoriz.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftHoriz.setDirection(DcMotor.Direction.REVERSE);
        rightHoriz.setDirection(DcMotor.Direction.REVERSE);

        WebcamName webcamname = hardwareMap.get(WebcamName.class, "Webcam 1");
        // Acquire the camera ID
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName() );
        //set the cam name and id to the webcam.
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamname, cameraMonitorViewId);

        //set webcam Pipeline
        webcam.setPipeline(new ContourPipeline());

        //You're creating an instance of the AsyncCameraOpenListener class. The class contains the two methods, onOpened and onError, which you are overriding with your code. That instance is passed to the openCameraDeviceAsync method as a parameter.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Webcam not working");
            }
        });



        // int encoderValue = 0;
        boolean bool = false;

        //slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // slides.setPower(0.5);

        waitForStart();

        if (isStopRequested())
            return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y * 0.47; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1 * 0.47; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x * 0.47;

            // If there is a weird Horizontal Arm Config issue, it might be here
            // IE, telling claw to move but smth else does.
           servo0.setPosition(out);
           servo1.setPosition(1 - out);
           right_elbow.setPosition(1-elbow_val);
           left_elbow.setPosition(elbow_val);
           wrist.setPosition(wrist_val);
           claw.setPosition(claw_val);


            if ( vertical_movement == 1) {
                leftVert.setPower(-power);
                rightVert.setPower(power);
            } else if ( vertical_movement == 2) {
                leftVert.setPower(power);
                rightVert.setPower(-power);
            } else {
                leftVert.setPower(0);
                rightVert.setPower(0);
            }
            if (horizontal_movement == 1) {

                if (leftHoriz.getCurrentPosition() < encoder_pos)
                {
                    leftHoriz.setTargetPosition(encoder_pos);
                    leftHoriz.setPower(power);
                    rightHoriz.setPower(leftHoriz.getPower());
                }


            } else if (horizontal_movement == 2) {

                leftHoriz.setTargetPosition(0);
                leftHoriz.setPower(power);
                rightHoriz.setPower(power);

            } else {
                leftHoriz.setPower(0);
                rightHoriz.setPower(0);
            }
            // ReLEASE slide to drop claw.

            double Largest_Contour = ContourPipeline.getLargestSize();

            if (Largest_Contour > threshold)
            {
                claw.setPosition(0.9);
            }
            else
            {
                claw.setPosition(claw_val);
            }
        }
        telemetry.update();






//            if(frontLeftPower == 0 && backLeftPower == 0 && frontRightPower == 0 && backRightPower == 0)
//            {
//                motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                motorFrontLeft.setTargetPosition(0);
//                motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                motorFrontRight.setTargetPosition(0);
//                motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                motorBackLeft.setTargetPosition(0);
//                motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }




            // Show the elapsed game time and wheel power



            telemetry.addData("rightSlide encoder:", rightHoriz.getCurrentPosition());
            telemetry.addData("vert encoder", rightVert.getCurrentPosition());
            telemetry.update();
        }
    }
