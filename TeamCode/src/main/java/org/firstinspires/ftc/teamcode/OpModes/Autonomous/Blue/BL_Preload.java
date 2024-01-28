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
    Vector2d retractPos;

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
        retractPos = new Vector2d(13, 58);
        midwayVector = new Vector2d(13, 30);
        leftVector = new Vector2d(27,30);
        rightVector = new Vector2d(-1, 30);
        centerVector = new Vector2d(13, 22);
        scoringVector = new Vector2d(47, 30);
        parkingPose = new Vector2d(47,58);
        finalPose = new Vector2d(65, 58);
        double backwardsDistance = 7;
        double turnAngle = -115;


        TrajectorySequence centerPreload = driveTrain.trajectorySequenceBuilder(initPose)
                .lineToConstantHeading(centerVector)
                .lineToConstantHeading(retractPos)
                .strafeTo(finalPose)
                .turn(Math.toRadians(turnAngle))
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
                .build();

        TrajectorySequence rightPreload = driveTrain.trajectorySequenceBuilder(initPose)
                .lineToConstantHeading(midwayVector)
                .strafeTo(rightVector)
                .lineToConstantHeading(new Vector2d(rightVector.getX(), rightVector.getY() + backwardsDistance)) //Goes Back
                .strafeTo(new Vector2d(midwayVector.getX(), rightVector.getY() + backwardsDistance))
                .lineToConstantHeading(new Vector2d(midwayVector.getX(), 58)) //
                .strafeTo(finalPose)
                .turn(Math.toRadians(turnAngle))
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
                .build();

        TrajectorySequence leftPreload = driveTrain.trajectorySequenceBuilder(initPose)
                .lineToConstantHeading(midwayVector)
                .strafeTo(leftVector)
                .lineToConstantHeading(new Vector2d(leftVector.getX(), leftVector.getY() + backwardsDistance)) //Goes Back
                .lineToConstantHeading(new Vector2d(leftVector.getX(), 58)) //
                .strafeTo(finalPose)
                .turn(Math.toRadians(turnAngle))
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