package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.*;

// blah blah blah


@TeleOp(name="calibration testing", group="Linear Opmode")
// decorator so the robot knows what mode to operate in
@Config
public class ZoomZoom extends LinearOpMode {
    // Declare teleOp members.
    private ElapsedTime runtime = new ElapsedTime();
    public static double out = 0.5;
    public static double arm = 0.5;
    public static double wrist = 0.5;
    public static double claw = 0.5;
    public static int onOff = 0;
    public static double power = 0;
    public static int horiz = 0;

    @Override
    // related to inheritance: runOpMode is a necessary function as you NEED to override the runOpMode in the superclass
    public void runOpMode() {
        //System.out on the phones = telemetry
        Servo servo0 = hardwareMap.servo.get("servo0out");
        Servo servo1 = hardwareMap.servo.get("servo1out");
        Servo servo2 = hardwareMap.servo.get("servo2arm");
        Servo servo3 = hardwareMap.servo.get("servo3arm");
        Servo servo4 = hardwareMap.servo.get("servo4wrist");
        Servo servo5 = hardwareMap.servo.get("servo5wrist");
        Servo servo6 = hardwareMap.servo.get("servo6claw");
        DcMotor leftHoriz = hardwareMap.dcMotor.get("leftHoriz");
        DcMotor rightHoriz = hardwareMap.dcMotor.get("rightHoriz");
        DcMotor rightVert = hardwareMap.dcMotor.get("rightVert");
        DcMotor leftVert = hardwareMap.dcMotor.get("leftVert");
        rightVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightVert.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftVert.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        // int encoderValue = 0;
        boolean bool = false;

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        //motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //  slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //  slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // slides.setPower(0.5);

        waitForStart();

        if (isStopRequested())
            return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y * 0.47; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1 * 0.47; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x * 0.47;

            servo0.setPosition(out);
            servo1.setPosition(1 - out);
            servo2.setPosition(arm);
            servo3.setPosition(1-arm);
            servo4.setPosition(wrist);
            //servo5.setPosition(1-wrist);
            servo6.setPosition(claw);
            if (onOff == 1) {
                leftVert.setPower(-power);
                rightVert.setPower(power);
            } else if (onOff == 2) {
                leftVert.setPower(power);
                rightVert.setPower(-power);
            } else {
                leftVert.setPower(0);
                rightVert.setPower(0);
            }
            if (horiz == 1) {
                leftHoriz.setPower(-power);
                rightHoriz.setPower(power);
            } else if (horiz == 2) {
                leftHoriz.setPower(power);
                rightHoriz.setPower(-power);
            } else {
                leftHoriz.setPower(0);
                rightHoriz.setPower(0);
            }
            // ReLEASE slide to drop claw.







            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            // to move left --


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
}