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

import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Constants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TEST")
public class TeleOp extends LinearOpMode {

    public SampleMecanumDrive driveTrain;
    public PIDController controller;
    public DcMotorEx leftSlide, rightSlide;
    public Arm armSystem;
    public Intake intakeSystem;
    public static int target = 0;
    public static int previousTarget;

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
        armSystem = new Arm(hardwareMap);
        intakeSystem = new Intake(hardwareMap);
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


        armSystem.init();
        intakeSystem.init();


        speedState = SpeedState.NORMAL;
        target = 0;


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


                if (gamepad1.square) {
                    if (target == Constants.LIFT_LEVEL_ZERO) {
                        previousTarget = 0;
                    }
                    target = Constants.LIFT_FIRST_LEVEL;
                } else if (gamepad1.triangle) {
                    if (target == Constants.LIFT_LEVEL_ZERO) {
                        previousTarget = 0;
                    }
                    target = Constants.LIFT_SECOND_LEVEL;
                } else if (gamepad1.circle) {
                    if (target == Constants.LIFT_LEVEL_ZERO) {
                        previousTarget = 0;
                    }
                    target = Constants.LIFT_THIRD_LEVEL;
                } else if (gamepad1.cross) {
                    previousTarget = target;
                    target = Constants.LIFT_LEVEL_ZERO;
                }

                int state = leftSlide.getCurrentPosition();
                double pid = controller.calculate(state, target);
                double power = pid + Constants.Kf;

                leftSlide.setPower(power * 0.2);
                rightSlide.setPower(power * 0.2);


                telemetry.addData("Slides Target", target);
                telemetry.addData("rightSlide Position", rightSlide.getCurrentPosition());
                telemetry.addData("leftSlide Position", leftSlide.getCurrentPosition());
                telemetry.addData("Drivetrain Speed", speedState);
                telemetry.update();

            }
        }
    }
}


