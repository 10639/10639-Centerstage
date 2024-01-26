package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Box;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.BluePipeline;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name = "🟦 Left Preload", preselectTeleOp = "CenterStage_TeleOp")
public class BL_Preload extends LinearOpMode {

    public SampleMecanumDrive driveTrain;
    public Arm armSystem;
    public Intake intakeSystem;
    public Box pixelDetector;
    BluePipeline pipeline;
    OpenCvWebcam webcam;

    Pose2d initPose;
    Vector2d midwayVector;
    Vector2d scoringVector;
    Vector2d centerVector;
    Vector2d leftVector;
    Vector2d rightVector;
    Vector2d finalPose;
    Vector2d parkingPose;

    @Override
    public void runOpMode() throws InterruptedException {

        driveTrain = new SampleMecanumDrive(hardwareMap);
        armSystem = new Arm(hardwareMap);
        intakeSystem = new Intake(hardwareMap);
        pixelDetector = new Box(hardwareMap);
        BluePipeline.Location location = BluePipeline.Location.RIGHT;

        armSystem.init(); //De-Powers
        intakeSystem.init();
        pixelDetector.init();

        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()
                );

        webcam = OpenCvCameraFactory
                .getInstance().createWebcam(
                        hardwareMap.get(WebcamName.class, "Webcam"),
                        cameraMonitorViewId);

        pipeline = new BluePipeline(telemetry);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        initPose = new Pose2d(13, 58, Math.toRadians(-270));
        midwayVector = new Vector2d(13, 35);
        leftVector = new Vector2d(22,35);
        rightVector = new Vector2d(0, 35);
        centerVector = new Vector2d(13, 30);
        scoringVector = new Vector2d(47, 35);
        parkingPose = new Vector2d(47,60);
        finalPose = new Vector2d(60, 60);
        double backwardsDistance = 3;
        double turnAngle = 115;


        TrajectorySequence centerPreload = driveTrain.trajectorySequenceBuilder(initPose)
                .lineToConstantHeading(centerVector)
                .setReversed(true)
                .lineToConstantHeading(midwayVector)
                .setReversed(false)
                .turn(Math.toRadians(turnAngle))
                .lineToConstantHeading(scoringVector)
                .strafeTo(parkingPose)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> { //0.5 Seconds after Strafing
                    intakeSystem.reverseSweep();
                    intakeSystem.boxReverseSweep();
                    intakeSystem.retractIntake();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> { //0.1 Seconds AFTER wait seconds is over
                    intakeSystem.terminateSweep();
                    intakeSystem.terminateBoxSweeper();
                })
                .waitSeconds(0.5)
                .lineToConstantHeading(finalPose)
                .build();

        TrajectorySequence rightPreload = driveTrain.trajectorySequenceBuilder(initPose)
                .lineToConstantHeading(midwayVector)
                .strafeTo(rightVector)
                .lineToConstantHeading(new Vector2d(rightVector.getX(), rightVector.getY() + backwardsDistance))
                .strafeTo(new Vector2d(midwayVector.getX(), rightVector.getY() + backwardsDistance))
                .turn(Math.toRadians(turnAngle))
                .lineToConstantHeading(new Vector2d(scoringVector.getX(), rightVector.getY() + backwardsDistance))
                .strafeTo(parkingPose)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> { //0.5 Seconds after Strafing
                    intakeSystem.reverseSweep();
                    intakeSystem.boxReverseSweep();
                    intakeSystem.retractIntake();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> { //0.1 Seconds AFTER wait seconds is over
                    intakeSystem.terminateSweep();
                    intakeSystem.terminateBoxSweeper();
                })
                .waitSeconds(0.5)
                .lineToConstantHeading(finalPose)
                .build();

        TrajectorySequence leftPreload = driveTrain.trajectorySequenceBuilder(initPose)
                .lineToConstantHeading(midwayVector)
                .strafeTo(leftVector)
                .lineToConstantHeading(new Vector2d(leftVector.getX(), leftVector.getY() + backwardsDistance))
                .turn(Math.toRadians(turnAngle))
                .lineToConstantHeading(new Vector2d(scoringVector.getX(), leftVector.getY() + backwardsDistance))
                .strafeTo(parkingPose)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> { //0.5 Seconds after Strafing
                    intakeSystem.reverseSweep();
                    intakeSystem.boxReverseSweep();
                    intakeSystem.retractIntake();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> { //0.1 Seconds AFTER wait seconds is over
                    intakeSystem.terminateSweep();
                    intakeSystem.terminateBoxSweeper();
                })
                .lineToConstantHeading(finalPose)
                .build();




        driveTrain.getLocalizer().setPoseEstimate(initPose);
        while (!isStarted()) {
            location = pipeline.getLocation();
            telemetry.addData("Identified Location", location);
            telemetry.addLine("Ready to Start! [Preload + Park)");
            telemetry.update();
        }

        waitForStart();
        switch(location) {
            case LEFT:
                driveTrain.followTrajectorySequenceAsync(leftPreload);
                break;
            case MIDDLE:
                driveTrain.followTrajectorySequenceAsync(centerPreload);
                break;
            case RIGHT:
                driveTrain.followTrajectorySequenceAsync(rightPreload);
                break;
        }
        BluePipeline.stop(webcam);

        while (!isStopRequested()) {
            while (opModeIsActive()) {
                driveTrain.update();
                telemetry.addData("Pixel Count", pixelDetector.getCount());
            }

        }

    }
}

