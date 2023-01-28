package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
// blah blah blah



@TeleOp(name="testing_encoders dec 9", group="Linear Opmode")
// decorator so the robot knows what mode to operate in
public class EncoderSetup extends LinearOpMode {
    // Declare teleOp members.
    private ElapsedTime runtime = new ElapsedTime();
    int encoderValue = 0;
    boolean bool = false;

    @Override
    // related to inheritance: runOpMode is a necessary function as you NEED to override the runOpMode in the superclass
    public void runOpMode() {
        //System.out on the phones = telemetry


        int i = 0;
        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests

        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("leftFront");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("leftRear");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("rightFront");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("rightRear");
        DcMotor slides = hardwareMap.dcMotor.get("slides");
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setTargetPosition(100);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setTargetPosition(100);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setTargetPosition(100);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setTargetPosition(100);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slides.setTargetPosition(100);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        motorFrontRight.setPower(0.1);
        motorBackRight.setPower(0.1);
       motorFrontLeft.setPower(0.1);
        motorBackLeft.setPower(0.1);
        slides.setPower(0.1);
        // slides.setPower(0.5);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            /* if(i == 0) {
                motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackRight.setPower(0.1);
            }
            i++;
            */

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



            telemetry.addData("Encoder val:", motorFrontRight.getCurrentPosition());
            telemetry.addData("Encoder val:", motorBackRight.getCurrentPosition());
            telemetry.addData("Encoder val:", motorBackLeft.getCurrentPosition());
            telemetry.addData("Encoder val:", motorFrontLeft.getCurrentPosition());
            telemetry.addData("Encoder val:", slides.getCurrentPosition());
            telemetry.update();
            // Show the elapsed game time and wheel power

        }
    }
}