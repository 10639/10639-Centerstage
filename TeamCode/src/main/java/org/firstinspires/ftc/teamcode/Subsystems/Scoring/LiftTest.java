package org.firstinspires.ftc.teamcode.Subsystems.Scoring;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LiftTest {


    interface gamepadLoop {
        void run();
    }

    private final HardwareMap hardwareMap;

    public DcMotorEx leftSlide, rightSlide;
    public PIDController controller;
    public enum LiftState {LIFT_REST, LIFT_IDLE, LIFT_EXTENDING}


    public LiftTest(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    LiftState liftState;
    static int targetPosition = 0;

    public void init() {
        controller = new PIDController(Constants.Kp, Constants.Ki, Constants.Kd);
        controller.setPID(Constants.Kp, Constants.Ki, Constants.Kd);
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        rightSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);

        leftSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setDirection(DcMotor.Direction.FORWARD);
        liftState = LiftState.LIFT_REST;
    }

    public void reset() {
        targetPosition = Constants.LIFT_LEVEL_ZERO;
        rightSlide.setTargetPosition(targetPosition);
        leftSlide.setTargetPosition(rightSlide.getCurrentPosition());
        leftSlide.setPower(calcDirection(targetPosition));
        rightSlide.setPower(calcDirection(targetPosition));
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void extendLow() {
        targetPosition = Constants.LIFT_FIRST_LEVEL;
        rightSlide.setTargetPosition(targetPosition);
        leftSlide.setTargetPosition(rightSlide.getCurrentPosition());
        leftSlide.setPower(calcDirection(targetPosition));
        rightSlide.setPower(calcDirection(targetPosition));
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void extendMedium() {
        targetPosition = Constants.LIFT_SECOND_LEVEL;
        rightSlide.setTargetPosition(targetPosition);
        leftSlide.setTargetPosition(rightSlide.getCurrentPosition());
        leftSlide.setPower(calcDirection(targetPosition));
        rightSlide.setPower(calcDirection(targetPosition));
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void extendHigh() {
        targetPosition = Constants.LIFT_THIRD_LEVEL;
        rightSlide.setTargetPosition(targetPosition);
        leftSlide.setTargetPosition(rightSlide.getCurrentPosition());
        leftSlide.setPower(calcDirection(targetPosition));
        rightSlide.setPower(calcDirection(targetPosition));
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void ascend() {
        targetPosition = rightSlide.getCurrentPosition() + Constants.MANUAL_EXTEND_INCREMENT;
        rightSlide.setTargetPosition(targetPosition);
        leftSlide.setTargetPosition(rightSlide.getCurrentPosition());
        leftSlide.setPower(calcDirection(targetPosition));
        rightSlide.setPower(calcDirection(targetPosition));
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void descend() {
        targetPosition = rightSlide.getCurrentPosition() - Constants.MANUAL_DESCEND_INCREMENT;
        rightSlide.setTargetPosition(targetPosition);
        leftSlide.setTargetPosition(rightSlide.getCurrentPosition());
        leftSlide.setPower(calcDirection(targetPosition));
        rightSlide.setPower(calcDirection(targetPosition));
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void loop(Gamepad gamepad1, Gamepad gamepad2) {

        gamepadLoop FSM = () -> {
            if (gamepad1.square) {
                extendLow();
                liftState = LiftState.LIFT_EXTENDING;
            } else if (gamepad1.triangle) {
                extendMedium();
                liftState = LiftState.LIFT_EXTENDING;
            } else if (gamepad1.circle) {
                extendHigh();
                liftState = LiftState.LIFT_EXTENDING;
            } else if (gamepad2.dpad_down) {
                //Go down incrementally on each press for more precision
                descend();
                liftState = LiftState.LIFT_EXTENDING;
            } else if (gamepad2.dpad_up) {
                //Go up incrementally on each press for more precision
                ascend();
                liftState = LiftState.LIFT_EXTENDING;
            } else if (gamepad1.cross) {
                //Drop the lift regardless of position
                liftState = LiftState.LIFT_REST;
            }
        };

        switch (liftState) {
            case LIFT_REST:
                reset();
                liftState = LiftState.LIFT_EXTENDING;
                break;
            case LIFT_IDLE:
                FSM.run();
                break;

            //The following cases ensures that the motors have arrived to their designated positions before stopping power
            case LIFT_EXTENDING:
                //Check if the current position is near the target position before applying anti-gravity measures
                    //Constant output based on Target Position & Current Position to counter gravity
                if (Math.abs(rightSlide.getCurrentPosition() - rightSlide.getTargetPosition()) < 10) {
                    double antiGravity = controller.calculate(rightSlide.getCurrentPosition(), rightSlide.getTargetPosition()) + Constants.Kf;
                    rightSlide.setPower(antiGravity);
                    leftSlide.setPower(antiGravity);
                }
                    FSM.run(); // Still allow alternate positions to be set while continuing to apply constant power


                break;

        }
    }

    public double calcDirection(double targetPosition) {
        double encoderDirection = 1;
        double currentPosition = rightSlide.getCurrentPosition() * encoderDirection;
        //Use max speed if going up otherwise use 83% of max speed (0.83).
        return (currentPosition <= targetPosition) ? 0.8 : -0.4;
    }



}
