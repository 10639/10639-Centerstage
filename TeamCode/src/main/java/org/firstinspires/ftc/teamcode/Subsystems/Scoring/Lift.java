package org.firstinspires.ftc.teamcode.Subsystems.Scoring;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {

//REDACTED DO NOT USE
//REDACTED DO NOT USE
//REDACTED DO NOT USE
//REDACTED DO NOT USE
//REDACTED DO NOT USE
//REDACTED DO NOT USE
//REDACTED DO NOT USE
//REDACTED DO NOT USE
//REDACTED DO NOT USE
//REDACTED DO NOT USE
//REDACTED DO NOT USE
//REDACTED DO NOT USE
//REDACTED DO NOT USE
//REDACTED DO NOT USE
//REDACTED DO NOT USE
//REDACTED DO NOT USE
//REDACTED DO NOT USE
//REDACTED DO NOT USE
//REDACTED DO NOT USE
//REDACTED DO NOT USE
//REDACTED DO NOT USE
//REDACTED DO NOT USE
//REDACTED DO NOT USE

    private final HardwareMap hardwareMap;

    public DcMotorEx leftSlide, rightSlide;
    public PIDController controller;
    public static int target = 0;

    public Lift(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

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
    }

    public void loop(Gamepad gamepad) {

        if (gamepad.square) {
            target = Constants.LIFT_FIRST_LEVEL;
        } else if (gamepad.triangle) {
            target = Constants.LIFT_SECOND_LEVEL;
        } else if (gamepad.circle) {
            target = Constants.LIFT_THIRD_LEVEL;
        } else if (gamepad.cross) {
            target = Constants.LIFT_LEVEL_ZERO;


            int state = leftSlide.getCurrentPosition();
            double pid = controller.calculate(state, target);
            double power = pid + Constants.Kf;
            leftSlide.setPower(state < target ? (power * 0.8) : (power * 0.1));
            rightSlide.setPower(state < target ? (power * 0.8) : (power * 0.1));
        }


//    public void showInfo(Telemetry telemetry) {
//        telemetry.addData("Lift Position", Motor.getCurrentPosition());
//        telemetry.addData("Distance Verifier", Math.abs(Motor.getCurrentPosition() - Motor.getTargetPosition()));
//
//        telemetry.addData("Lift Power", Motor.getPower());
//        telemetry.addData("Lift Velocity", Motor.getVelocity());
//    }

    }

}
