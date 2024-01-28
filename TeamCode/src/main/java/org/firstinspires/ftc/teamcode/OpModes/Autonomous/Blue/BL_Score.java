package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Box;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Intake;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.BluePipeline;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name = "🟦 Left Backdrop", preselectTeleOp = "CenterStage_TeleOp")
public class BL_Score extends LinearOpMode {

    public SampleMecanumDrive driveTrain;
    public DcMotorEx leftSlide, rightSlide;
    private PIDController controller;

    public Arm armSystem;
    public Intake intakeSystem;
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

    public static int target = 0;
    public static boolean rightSlideRest = true;
    public static boolean scoreAllowed = false;
    public static boolean tiltBox = false;

    enum autoState {
        PRELOAD,
        BACKDROP,
        IDLE
    }
    autoState currentState = autoState.IDLE;
    @Override
    public void runOpMode() throws InterruptedException {

        driveTrain = new SampleMecanumDrive(hardwareMap);
        armSystem = new Arm(hardwareMap);
        intakeSystem = new Intake(hardwareMap);

        Lift lift = new Lift(hardwareMap);
        controller = new PIDController(Constants.Kp, Constants.Ki, Constants.Kd);
        BluePipeline.Location location = BluePipeline.Location.RIGHT;


        armSystem.init(); ///De-Powers
        intakeSystem.init();

        rightSlideRest = true;
        scoreAllowed = false;
        tiltBox = false;
        target = 0;


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
        scoringVector = new Vector2d(58, 30);
        parkingPose = new Vector2d(47,58);
        finalPose = new Vector2d(65, 58);
        double backwardsDistance = 7;
        double turnAngle = 115; //Counter-Clockwise


        TrajectorySequence centerPreload = driveTrain.trajectorySequenceBuilder(initPose)
                .lineToConstantHeading(centerVector)
                .waitSeconds(0.5)
                .lineToConstantHeading(midwayVector)
                .strafeTo(new Vector2d(scoringVector.getX(), midwayVector.getY()))
                .lineToConstantHeading(new Vector2d(scoringVector.getX(), midwayVector.getY() + 7))
                .turn(Math.toRadians(turnAngle))
                .back(11)
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    target = Constants.LIFT_FIRST_LEVEL;
                    scoreAllowed = true;
                    armSystem.armIdle();
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    tiltBox = true;
                    armSystem.armScore();
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    intakeSystem.boxSweeper.setPower(0.5);
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    intakeSystem.terminateBoxSweeper();
                    target = Constants.LIFT_LEVEL_ZERO;
                    scoreAllowed = false;
                    tiltBox = false;
                })
                .forward(5)
                .strafeRight(24)
                .back(15)
                .build();

        TrajectorySequence rightPreload = driveTrain.trajectorySequenceBuilder(initPose)
                .lineToConstantHeading(midwayVector)
                .strafeTo(rightVector)
                .lineToConstantHeading(new Vector2d(rightVector.getX(), rightVector.getY() + backwardsDistance))
                .strafeTo(new Vector2d(leftVector.getX(), rightVector.getY() + backwardsDistance))
                .waitSeconds(1)
                .strafeTo(new Vector2d(scoringVector.getX(), rightVector.getY() + backwardsDistance))
                .lineToConstantHeading(new Vector2d(scoringVector.getX(), midwayVector.getY() - (7))) //Up to Y:30
                .turn(Math.toRadians(turnAngle))
                .back(11)
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    target = Constants.LIFT_FIRST_LEVEL;
                    scoreAllowed = true;
                    armSystem.armIdle();
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    tiltBox = true;
                    armSystem.armScore();
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    intakeSystem.boxSweeper.setPower(0.5);
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    intakeSystem.terminateBoxSweeper();
                    target = Constants.LIFT_LEVEL_ZERO;
                    scoreAllowed = false;
                    tiltBox = false;
                })
                .forward(5)
                .strafeRight(38)
                .back(15)
                .build();

        TrajectorySequence leftPreload = driveTrain.trajectorySequenceBuilder(initPose)
                .lineToConstantHeading(midwayVector)
                .strafeTo(leftVector)
                .lineToConstantHeading(new Vector2d(leftVector.getX(), leftVector.getY() + backwardsDistance)) //Goes Back
                .strafeTo(new Vector2d(scoringVector.getX(), rightVector.getY() + backwardsDistance))
                .lineToConstantHeading(new Vector2d(scoringVector.getX(), leftVector.getY() + (backwardsDistance + 2))) //Goes Back
                .turn(Math.toRadians(turnAngle))
                .back(11)
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    target = Constants.LIFT_FIRST_LEVEL;
                    scoreAllowed = true;
                    armSystem.armIdle();
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    tiltBox = true;
                    armSystem.armScore();
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    intakeSystem.boxSweeper.setPower(0.5);
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    intakeSystem.terminateBoxSweeper();
                    target = Constants.LIFT_LEVEL_ZERO;
                    scoreAllowed = false;
                    tiltBox = false;
                })
                .forward(5)
                .strafeRight(27)
                .back(15)
                .build();


        driveTrain.getLocalizer().setPoseEstimate(initPose);
        while (!isStarted()) {
            location = pipeline.getLocation();
            telemetry.addData("Identified Location", location);
            telemetry.addLine("Ready to Start! [Preload + Park)");
            telemetry.update();
        }

        waitForStart();
        currentState = autoState.PRELOAD;
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

                switch(currentState) {
                    case PRELOAD:
                        if(!driveTrain.isBusy()) {
                            currentState = autoState.IDLE;
                        }
                        break;
                    case IDLE:
                        break;
                }

                telemetry.addData("Autonomous State", currentState);
                telemetry.addData("Slides Target", target);
                telemetry.addData("Right Slide @ Rest", rightSlideRest);
                telemetry.update();
                driveTrain.update(); //Update deadwheel encoder counts
                lift.update();


            }

        }

    }
    class Lift {
        public Lift(HardwareMap hardwareMap) {
            controller = new PIDController(Constants.Kp, Constants.Ki, Constants.Kd);
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
        }

        public void update() {
            controller.setPID(Constants.Kp, Constants.Ki, Constants.Kd);
            int leftPosition = leftSlide.getCurrentPosition();
            double pid = controller.calculate(leftPosition, target);
            double power = pid + Constants.Kf;
            if (pid < 0) { // Going down
                power = Math.max(power, -0.1);
                scoreAllowed = false;
            } else { //Going up
                power = Math.min(power, 0.8); //Power Range 0 -> 1;
            }
            leftSlide.setPower(power);
            rightSlide.setPower(power);
            if (leftSlide.getCurrentPosition() > 15) {
                rightSlideRest = false;
                scoreAllowed = true;
            }
            if (pid < 0) {
                armSystem.armIdle();
            }
            if (scoreAllowed) {
                if (gamepad2.cross || gamepad2.triangle) {
                    tiltBox = true;
                }
                if (tiltBox) {
                    armSystem.armScore();
                } else {
                    armSystem.armIdle();
                }

            }

            if ((target == 0)) { //Ensure Lifts are Fully Down (Observation: Right Slide Mainly Issues)
                armSystem.armIdle();
                scoreAllowed = false;
                tiltBox = false;
                if (leftSlide.getCurrentPosition() < 2 || (leftSlide.getCurrentPosition() < 0 && leftSlide.getCurrentPosition() >= -1)) {
                    armSystem.dePower();
                } else if (leftSlide.getCurrentPosition() > 2 || leftSlide.getCurrentPosition() < 0) {
                    while (leftSlide.getCurrentPosition() > 2 || leftSlide.getCurrentPosition() < 0) {
                        leftSlide.setPower((Math.signum(leftSlide.getCurrentPosition() * -1) * 0.3));
                    }
                    armSystem.dePower();
                }
            }
        }
    }

}