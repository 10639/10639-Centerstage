package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Control.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.ConeSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.Detection;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Arm;

@Autonomous(name = "Auto_BlueLeft", preselectTeleOp = "PowerPlay_TeleOp")
public class Auto_BlueLeft extends LinearOpMode {

    public SampleMecanumDrive driveTrain;
    public PIDController controller;
    Lift liftSystem;
    Arm armSystem;
    ConeSensor coneSensor;
    Detection detectionSystem;


    Pose2d initPose;
    Vector2d midWayPose;
    Vector2d dropConePose;
    Vector2d parkingLocationOne, parkingLocationTwo, parkingLocationThree;

    public enum TrajectoryState {PRELOAD, CS, CS_TO_MJ, PARK, IDLE}

    TrajectoryState trajectoryState = TrajectoryState.PRELOAD;

    public static int identifiedLocation = 2;
    public static int targetPosition = 0;
    public boolean preloadDone = false;

    @Override
    public void runOpMode() throws InterruptedException {

        driveTrain = new SampleMecanumDrive(hardwareMap);
        liftSystem = new Lift(hardwareMap);
        armSystem = new Arm(hardwareMap);
        //    coneSensor = new ConeSensor(hardwareMap);
        detectionSystem = new Detection(hardwareMap);
        ElapsedTime time = new ElapsedTime();

        liftSystem.init();
        armSystem.init();
        armSystem.rotationIdle();
        armSystem.leftPivot.setPosition(0.5);
        armSystem.rightPivot.setPosition(0.5);
        //     coneSensor.init();
        detectionSystem.init();

        initPose = new Pose2d(-35, -58, Math.toRadians(270));
        midWayPose = new Vector2d(-35, -10);
        //  dropConePose = new Vector2d(-29, 28);

        parkingLocationOne = new Vector2d(-58, -32);
        parkingLocationTwo = new Vector2d(-35, -32);
        parkingLocationThree = new Vector2d(-13, -32);

        TrajectorySequence preload = driveTrain.trajectorySequenceBuilder(initPose)
                .lineToConstantHeading(new Vector2d(-34, -32))
                .build();

        TrajectorySequence locationOne = driveTrain.trajectorySequenceBuilder(preload.end())
                .lineToConstantHeading(parkingLocationOne)
                .build();

        TrajectorySequence locationTwo = driveTrain.trajectorySequenceBuilder(preload.end())
                .lineToConstantHeading(parkingLocationTwo)
                .build();

        TrajectorySequence locationThree = driveTrain.trajectorySequenceBuilder(preload.end())
                .lineToConstantHeading(parkingLocationThree)
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
                telemetry.addData("state", state);
                telemetry.addData("target", targetPosition);
                telemetry.update();
                double pid = liftSystem.controller.calculate(state, targetPosition);
                double power = pid + Constants.Kf;
                liftSystem.leftSlide.setPower(power * 0.8);
                liftSystem.rightSlide.setPower(power * 0.8);

//                switch (identifiedLocation) {
//                    case 1:
//                        if (!driveTrain.isBusy()) {
//                            driveTrain.followTrajectorySequenceAsync(locationOne);
//                            trajectoryState = TrajectoryState.IDLE;
//                        }
//                        break;
//                    case 2:
//                        if (!driveTrain.isBusy()) {
//                            driveTrain.followTrajectorySequenceAsync(locationTwo);
//                            trajectoryState = TrajectoryState.IDLE;
//                        }
//
//                    case 3:
//                        if (!driveTrain.isBusy()) {
//                            driveTrain.followTrajectorySequenceAsync(locationThree);
//                            trajectoryState = TrajectoryState.IDLE;
//                        }
//                        break;
//                }

                switch (trajectoryState) {
                    case PRELOAD:
                        targetPosition = Constants.LIFT_LEVEL_ZERO;
                        if (!driveTrain.isBusy()) {
                            trajectoryState = TrajectoryState.PARK;
                        }
                        break;
                    case PARK:
                        targetPosition = Constants.LIFT_LEVEL_ZERO;
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
                                break;
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




