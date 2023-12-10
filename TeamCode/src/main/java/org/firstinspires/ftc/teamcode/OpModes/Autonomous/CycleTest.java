package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.Detection;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Arm;

@Autonomous(name = "CycleTest", preselectTeleOp = "PowerPlay_TeleOp")
public class CycleTest extends LinearOpMode {

    SampleMecanumDrive driveTrain;
    Lift liftSystem;
    Arm armSystem;
    Detection detectionSystem;

    Pose2d initPose, parkingLocationOne, parkingLocationTwo, parkingLocationThree;
    Vector2d midWayPose, midWayCyclePose;
    Vector2d coneStackPose;

    public enum TrajectoryState { PRELOAD, CS_TO_HJ, HJ_TO_CS, PARK, VERIFY, IDLE }
    TrajectoryState trajectoryState = TrajectoryState.PRELOAD;

    public static int identifiedLocation = 2;
    public static int targetPosition = 0;
    public static final int conesCount = 6;
    public static int conesScored = 0;

    public static double poleAlignmentHeading = Math.toRadians(22.92);
    public static double reverseDriveTrainHeading = Math.toRadians(270);
    public static double coneStackHeading = Math.toRadians(180);

    public static final TrajectoryVelocityConstraint defaultVelocity = SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(187.94061000000002), Math.toRadians(187.94061000000002));
    public static final TrajectoryAccelerationConstraint defaultAcceleration = SampleMecanumDrive.getAccelerationConstraint(45);

    public static final TrajectoryVelocityConstraint fastVelocity = SampleMecanumDrive.getVelocityConstraint(50, Math.toRadians(187.94061000000002), Math.toRadians(187.94061000000002));
    public static final TrajectoryAccelerationConstraint fastAcceleration = SampleMecanumDrive.getAccelerationConstraint(50);

    @Override
    public void runOpMode() throws InterruptedException {

        driveTrain = new SampleMecanumDrive(hardwareMap);
        liftSystem = new Lift(hardwareMap);
        armSystem = new Arm(hardwareMap);
        detectionSystem = new Detection(hardwareMap);
        ElapsedTime time = new ElapsedTime();

        liftSystem.init();
        armSystem.init();
        detectionSystem.init();

        initPose = new Pose2d(-35, -58, reverseDriveTrainHeading);
        midWayPose = new Vector2d(-33, -10);
        midWayCyclePose = new Vector2d(-32, -7);
        coneStackPose = new Vector2d(-60, -12);

        parkingLocationOne = new Pose2d(-60, -12, coneStackHeading);
        parkingLocationTwo = new Pose2d(-34, -12, reverseDriveTrainHeading);
        parkingLocationThree = new Pose2d(-13, -12, reverseDriveTrainHeading);


        TrajectorySequence preload = driveTrain.trajectorySequenceBuilder(initPose)
                .lineToConstantHeading(midWayPose)
                .setReversed(true)
                .splineTo(midWayCyclePose, poleAlignmentHeading)
                .build();

        TrajectorySequence preloadToConeStack = driveTrain.trajectorySequenceBuilder(preload.end())
                .setConstraints(defaultVelocity, defaultAcceleration)
                .setReversed(false)
                .splineTo(coneStackPose, coneStackHeading)
                .build();

        TrajectorySequence coneStackToHigh = driveTrain.trajectorySequenceBuilder(preloadToConeStack.end())
                .setConstraints(defaultVelocity, defaultAcceleration)
                .setReversed(true)
                .splineTo(midWayCyclePose, poleAlignmentHeading)
                .build();


        TrajectorySequence locationOne = driveTrain.trajectorySequenceBuilder(coneStackToHigh.end())
                .setConstraints(fastVelocity, defaultAcceleration)
                .setReversed(false)
                .splineToLinearHeading(parkingLocationOne, coneStackHeading)
                .build();

        TrajectorySequence locationTwo = driveTrain.trajectorySequenceBuilder(coneStackToHigh.end())
                .setConstraints(defaultVelocity, defaultAcceleration)
                .setReversed(false)
                .splineToLinearHeading(parkingLocationTwo, coneStackHeading)
                .build();

        TrajectorySequence locationThree = driveTrain.trajectorySequenceBuilder(coneStackToHigh.end())
                .setConstraints(fastVelocity, fastAcceleration)
                .setReversed(false)
                .splineToLinearHeading(parkingLocationThree, coneStackHeading)
                .build();

        driveTrain.getLocalizer().setPoseEstimate(initPose);
        while (!isStopRequested() && !opModeIsActive()) {
            identifiedLocation = detectionSystem.detect();
            telemetry.addData("Detected Parking Location:", identifiedLocation);
            telemetry.addLine("Initialization Ready");
            telemetry.update();
        }

        waitForStart();
        driveTrain.followTrajectorySequenceAsync(preload);
        detectionSystem.stop();

        while (!isStopRequested()) {
            while (opModeIsActive()) {
                driveTrain.update();

                liftSystem.controller.setPID(Constants.Kp, Constants.Ki, Constants.Kd);
                int state = liftSystem.rightSlide.getCurrentPosition();
                double pid = liftSystem.controller.calculate(state, targetPosition);
                double power = pid + Constants.Kf;
                liftSystem.leftSlide.setPower(power * 0.8);
                liftSystem.rightSlide.setPower(power * 0.8);

                switch (trajectoryState) {
                    case PRELOAD:
                        targetPosition = Constants.LIFT_THIRD_LEVEL;
                        if (!driveTrain.isBusy()) {
                            armSystem.scoreReady();
                            sleep(500);
                            trajectoryState = TrajectoryState.PARK;
                        }
                        break;
//                    case HJ_TO_CS:
//                        if(!driveTrain.isBusy()) {
//                            sleep(500);
//                            armSystem.closeArm();
//                            driveTrain.followTrajectorySequenceAsync(coneStackToHigh);
//                            armSystem.scoreReady();
//                            trajectoryState = TrajectoryState.CS_TO_HJ;
//                        }
//                        break;
//                    case CS_TO_HJ:
//                        targetPosition = Constants.LIFT_HIGH_JUNCTION;
//                        if(!driveTrain.isBusy()) {
//                            targetPosition = (int) Constants.CONE_STACK_POSITIONS[conesScored];
//                            armSystem.openArm();
//                            conesScored++;
//                            armSystem.scoreIdle();
//                            trajectoryState = TrajectoryState.VERIFY;
//                        }
//                        break;
//                    case VERIFY:
//                        if(conesScored != conesCount) {
//                            driveTrain.followTrajectorySequenceAsync(preloadToConeStack);
//                            trajectoryState = TrajectoryState.HJ_TO_CS;
//                        } else {
//                            targetPosition = Constants.LIFT_LEVEL_ZERO;
//                            trajectoryState = TrajectoryState.PARK;
//                        }
//                        break;
                    case PARK:
                        targetPosition = Constants.LIFT_LEVEL_ZERO;
                        armSystem.scoreIdle();
                        armSystem.closeArm();
                        switch (identifiedLocation) {
                            case 1:
                                if (!driveTrain.isBusy()) {
                                    driveTrain.followTrajectorySequenceAsync(locationOne);
                                    trajectoryState = TrajectoryState.IDLE;
                                }
                                break;
                            case 2:
                                if (!driveTrain.isBusy()) {
                                    driveTrain.followTrajectorySequenceAsync(locationTwo);
                                    trajectoryState = TrajectoryState.IDLE;
                                }

                            case 3:
                                if (!driveTrain.isBusy()) {
                                    driveTrain.followTrajectorySequenceAsync(locationThree);
                                    trajectoryState = TrajectoryState.IDLE;
                                }
                                break;
                        }
                        break;
                    case IDLE:
                        liftSystem.leftSlide.setTargetPosition(0);
                        liftSystem.leftSlide.setPower(1);
                        liftSystem.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        break;



                }

            }
        }
    }

}