package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.blueProp;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Arm;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name = "Auto_RedRight", preselectTeleOp = "CenterStage_TeleOp")
public class Auto_RedRight extends LinearOpMode {

    public SampleMecanumDrive driveTrain;
    public PIDController controller;
    Lift liftSystem;
    Arm armSystem;
    Intake intakeSystem;
    blueProp bluePipeline;
    OpenCvCamera camera;
    //  Detection detectionSystem;


    Pose2d initPose;
    Vector2d leftLocation, centerLocation, rightLocation;
    Vector2d finalPose;
    // Vector2d parkingLocationOne, parkingLocationTwo, parkingLocationThree;


    @Override
    public void runOpMode() throws InterruptedException {

        driveTrain = new SampleMecanumDrive(hardwareMap);
        liftSystem = new Lift(hardwareMap);
        armSystem = new Arm(hardwareMap);
        intakeSystem = new Intake(hardwareMap);
        bluePipeline = new blueProp(telemetry);
        //  detectionSystem = new Detection(hardwareMap);
        //ElapsedTime time = new ElapsedTime();

        //  liftSystem.init();
        armSystem.init();
        intakeSystem.init();

        //  detectionSystem.init(bluePipeline);




        initPose = new Pose2d(13, 58, Math.toRadians(90));
        finalPose = new Vector2d(60, 56);
        //    midWayPose = new Vector2d(-35, -10);
        //  dropConePose = new Vector2d(-29, 28);



        TrajectorySequence centerPreload = driveTrain.trajectorySequenceBuilder(initPose)
                .lineToConstantHeading(new Vector2d(13,30))
                .lineToConstantHeading(new Vector2d(13, 58))
                .setReversed(true)
                .strafeTo(new Vector2d(-60, 58))
                .build();


        driveTrain.getLocalizer().setPoseEstimate(initPose);
        while (!isStopRequested() && !opModeIsActive()) {
            //identifiedLocation = detectionSystem.detect();
            //   telemetry.addData("Detected Parking Location:", identifiedLocation);
            telemetry.addLine("Initialization Ready");
            telemetry.update();
        }

        waitForStart();
        driveTrain.followTrajectorySequenceAsync(centerPreload);
       // detectionSystem.stop();

        while (!isStopRequested()) {
            while (opModeIsActive()) {
                driveTrain.update();


               /*
                d


                liftSystem.controller.setPID(Constants.Kp, Constants.Ki, Constants.Kd);
                int state = liftSystem.leftSlide.getCurrentPosition();
                double pid = liftSystem.controller.calculate(state, targetPosition);
                double power = pid + Constants.Kf;
                liftSystem.leftSlide.setPower(power);

                switch (trajectoryState) {
                    case PRELOAD:
                        targetPosition = Constants.LIFT_SECOND_LEVEL;
                        if (!driveTrain.isBusy()) {
                            armSystem.openArm();
                            sleep(500);
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
                */

            }
        }
    }

}


