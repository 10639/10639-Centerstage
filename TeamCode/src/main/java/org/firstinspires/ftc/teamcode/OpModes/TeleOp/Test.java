package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.Subsystems.Control.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.LiftTest;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TEST")
public class Test extends LinearOpMode {

    public SampleMecanumDrive driveTrain;
    public PIDController controller;
    public DcMotorEx leftSlide, rightSlide;
    public Arm armSystem;
    public Intake intakeSystem;
  //  public LiftTest liftSystem;
    public static int target = 0;

    public enum SpeedState {
        NORMAL(0.5),
        FAST(0.9);
        double multiplier = 0.5; //Default

    SpeedState(double value) {
        this.multiplier = value;
    }
}
    SpeedState speedState;

    @Override
    public void runOpMode() throws InterruptedException {
        driveTrain = new SampleMecanumDrive(hardwareMap);
        controller = new PIDController(Constants.Kp, Constants.Ki, Constants.Kd);
        controller.setPID(Constants.Kp, Constants.Ki, Constants.Kd);
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);

        leftSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setDirection(DcMotor.Direction.FORWARD);

        armSystem = new Arm(hardwareMap);
        intakeSystem = new Intake(hardwareMap);
        //liftSystem = new LiftTest(hardwareMap);
        armSystem.init(); //
        intakeSystem.init();

        // liftSystem.init();


        speedState = SpeedState.NORMAL;


        if (isStopRequested()) return;
        while (!isStopRequested()) {
            while (opModeIsActive()) {


                if(gamepad1.left_bumper) {
                    speedState = SpeedState.NORMAL;
                } else if(gamepad1.right_bumper) {
                    speedState = SpeedState.FAST;
                }

                driveTrain.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * speedState.multiplier,
                                -gamepad1.left_stick_x * speedState.multiplier,
                                -gamepad1.right_stick_x * speedState.multiplier
                        )
                );

                armSystem.loop(gamepad2);
                intakeSystem.loop(gamepad2);

                if(gamepad1.square) {
                    target = Constants.LIFT_FIRST_LEVEL;
                } else if(gamepad1.triangle) {
                    target = Constants.LIFT_SECOND_LEVEL;
                } else if(gamepad1.circle) {
                    target = Constants.LIFT_THIRD_LEVEL;
                } else if(gamepad1.cross) {
                    target = Constants.LIFT_LEVEL_ZERO;
                } else if(gamepad1.dpad_up) {
                    target = rightSlide.getCurrentPosition() + Constants.MANUAL_EXTEND_INCREMENT;
                } else if(gamepad1.dpad_down) {
                    target = rightSlide.getCurrentPosition() - Constants.MANUAL_DESCEND_INCREMENT;
                }

                int state = rightSlide.getCurrentPosition();
                double pid = controller.calculate(state, target);
                double power = pid + Constants.Kf;

                if (state > target) {
                    // lift is going down, adjust the power
                    leftSlide.setPower(power * 0.1);
                    rightSlide.setPower(power * 0.1);
                } else {
                    // lift is going up, no need to adjust the power
                    leftSlide.setPower(power * 0.3);
                    rightSlide.setPower(power * 0.3);
                }
            }
        }

    }

}
