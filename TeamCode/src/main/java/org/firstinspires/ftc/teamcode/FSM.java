package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="FSM Testssss")
public class FSM extends OpMode {
    private DcMotor carry;
    private Servo bucket;

    ElapsedTime liftTimer = new ElapsedTime();

    final double DUMP_IDLE = -0.5;
    final double DUMP_DEPOSIT = 0.7;
    final double DUMP_TIME = .5;
    final int LIFT_LOW = 0;
    final int LIFT_HIGH = 1000;

    public enum liftState {
        START,
        EXTEND,
        DUMP,
        RETRACT
    }

    liftState state = liftState.START;

    public void init() {
        liftTimer.reset();
        carry = hardwareMap.get(DcMotor.class, "carry");
        carry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bucket = hardwareMap.get(Servo.class, "bucket");

    }

    public void loop() {
        switch (state) {
            case START:
                if (gamepad2.x) {
                    carry.setTargetPosition(LIFT_HIGH);
                    state = liftState.EXTEND;
                }
                break;
            case EXTEND:
                if (Math.abs(carry.getCurrentPosition() - LIFT_HIGH) < 10) {
                    bucket.setPosition(DUMP_DEPOSIT);
                    liftTimer.reset();
                    state = liftState.DUMP;
                }
                break;
            case DUMP:
                if (liftTimer.seconds() >= DUMP_TIME) {
                    bucket.setPosition(DUMP_IDLE);
                    carry.setTargetPosition(LIFT_LOW);
                    state = liftState.RETRACT;
                }
                break;
            case RETRACT:
                if (Math.abs(carry.getCurrentPosition() - LIFT_LOW) < 10) {
                    state = liftState.START;
                }
                if (gamepad1.y && state != liftState.START) {
                    state = liftState.START;
                }

                //drive stuff here


        }
    }
}