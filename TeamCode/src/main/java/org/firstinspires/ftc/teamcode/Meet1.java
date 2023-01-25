package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.*;
// blah blah blah



@TeleOp(name="movement teleop", group="Linear Opmode")
// decorator so the robot knows what mode to operate in
public class Meet1 extends LinearOpMode {
    // Declare teleOp members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    // related to inheritance: runOpMode is a necessary function as you NEED to override the runOpMode in the superclass
    public void runOpMode() {
        //System.out on the phones = telemetry
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("leftFront");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("leftRear");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("rightFront");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("rightRear");
        DcMotor slides = hardwareMap.dcMotor.get("slides");
        Servo servo0 = hardwareMap.servo.get("servo0");
        Servo servo1 = hardwareMap.servo.get("servo1");
       // int encoderValue = 0;
        boolean bool = false;
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
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

            if(gamepad1.right_bumper)
            {
                y = 0.6 * y;
                x = 0.6  * x;
                rx = 0.6 * rx;
            }
            else if(gamepad1.left_bumper)
            {
                y = 1.7 * y;
                x = 1.7  * x;
                rx = 1.7 * rx;
            }

            if(gamepad2.a)
            {
                bool = true;
            }
            else if(gamepad2.b)
            {
                bool = false;
            }
            if(bool)
            {
                servo0.setPosition(0);
                servo1.setPosition(1);
            }
            else {
                servo0.setPosition(1);
                servo1.setPosition(0);
            }
            // ReLEASE slide to drop claw.

            if(gamepad2.x)
            {
                //slides.setPower(-0.95);
                slides.setPower(0.9);

            }

            else if(gamepad2.y){
                slides.setPower(-0.9);

            }
            else {
                slides.setPower(0.5);
            }






            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            // to move left --

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
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
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.addData("encoderVal", slides.getCurrentPosition());

            telemetry.addData("lf val:", motorFrontLeft.getCurrentPosition());
            telemetry.addData("lr val:", motorBackLeft.getCurrentPosition());
            telemetry.addData("rf val:", motorFrontRight.getCurrentPosition());
            telemetry.addData("rr val:", motorBackRight.getCurrentPosition());
            telemetry.update();
        }
    }
}