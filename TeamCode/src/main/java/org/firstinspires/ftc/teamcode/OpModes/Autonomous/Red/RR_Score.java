package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Red;

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
import org.firstinspires.ftc.teamcode.Subsystems.Vision.BluePipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.RedPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name = "🟥 Right Backdrop", preselectTeleOp = "CenterStage_TeleOp")
public class RR_Score extends LinearOpMode {

    public SampleMecanumDrive driveTrain;
    public DcMotorEx leftSlide, rightSlide;
    private PIDController controller;

    public Arm armSystem;
    public Intake intakeSystem;
    public Box pixelDetector;
    RedPipeline pipeline;
    OpenCvWebcam webcam;

    Pose2d initPose;
    Vector2d midwayVector;
    Vector2d scoringVector;
    Vector2d centerVector;
    Vector2d leftVector;
    Vector2d rightVector;
    Vector2d finalPose;
    Vector2d parkingPose;

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
        pixelDetector = new Box(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        controller = new PIDController(Constants.Kp, Constants.Ki, Constants.Kd);
        RedPipeline.Location location = RedPipeline.Location.RIGHT;


        armSystem.init(); //De-Powers
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

        pipeline = new RedPipeline(telemetry);
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


        initPose = new Pose2d(13, -58, Math.toRadians(-270));
        midwayVector = new Vector2d(13, -35);
        leftVector = new Vector2d(22,-35);
        rightVector = new Vector2d(0, -35);
        centerVector = new Vector2d(13, -30);
        scoringVector = new Vector2d(47, -35);
        parkingPose = new Vector2d(47,-60);
        finalPose = new Vector2d(60, -60);
        double backwardsDistance = -3;
        double turnAngle = -115;


        TrajectorySequence midWayPos = driveTrain.trajectorySequenceBuilder(initPose)
                .lineToConstantHeading(midwayVector)
                .build();

        TrajectorySequence centerPreload = driveTrain.trajectorySequenceBuilder(midWayPos.end())
                .lineToConstantHeading(midwayVector)
                .lineToConstantHeading(centerVector)
                .setReversed(true)
                .lineToConstantHeading(midwayVector)
                .setReversed(false)
                .turn(Math.toRadians(turnAngle))
                .lineToConstantHeading(scoringVector)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> { //-0.5 Seconds BEFORE heading to Backdrop
                    target = Constants.LIFT_FIRST_LEVEL;
                    armSystem.armIdle();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> { //0.1 Seconds AFTER heading to Backdrop
                    tiltBox = true; //Tilt Box
                    armSystem.armIdle();
                    intakeSystem.boxReverseSweep();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> { //0.3 Seconds AFTER Waiting 1 Second
                    intakeSystem.terminateBoxSweeper();
                    tiltBox = false;
                    armSystem.armIdle(); //Extra Restrictions to get Box Down Safely
                    target = Constants.LIFT_LEVEL_ZERO;
                })
                .strafeTo(parkingPose)
                .lineToConstantHeading(finalPose)
                .build();


        TrajectorySequence rightPreload = driveTrain.trajectorySequenceBuilder(midWayPos.end())
                .lineToConstantHeading(midwayVector)
                .strafeTo(rightVector)
                .lineToConstantHeading(new Vector2d(rightVector.getX(), rightVector.getY() + backwardsDistance))
                .strafeTo(new Vector2d(midwayVector.getX(), rightVector.getY() + backwardsDistance))
                .turn(Math.toRadians(turnAngle))
                .lineToConstantHeading(new Vector2d(scoringVector.getX(), rightVector.getY() + backwardsDistance))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> { //-0.5 Seconds BEFORE heading to Backdrop
                    target = Constants.LIFT_FIRST_LEVEL;
                    armSystem.armIdle();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> { //0.1 Seconds AFTER heading to Backdrop
                    tiltBox = true; //Tilt Box
                    armSystem.armIdle();
                    intakeSystem.boxReverseSweep();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> { //0.3 Seconds AFTER Waiting 1 Second
                    intakeSystem.terminateBoxSweeper();
                    tiltBox = false;
                    armSystem.armIdle(); //Extra Restrictions to get Box Down Safely
                    target = Constants.LIFT_LEVEL_ZERO;
                })
                .strafeTo(parkingPose)
                .lineToConstantHeading(finalPose)
                .build();

        TrajectorySequence leftPreload = driveTrain.trajectorySequenceBuilder(midWayPos.end())
                .lineToConstantHeading(midwayVector)
                .strafeTo(leftVector)
                .lineToConstantHeading(new Vector2d(leftVector.getX(), leftVector.getY() + backwardsDistance))
                .turn(Math.toRadians(turnAngle))
                .lineToConstantHeading(new Vector2d(scoringVector.getX(), leftVector.getY() + backwardsDistance))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> { //-0.5 Seconds BEFORE heading to Backdrop
                    target = Constants.LIFT_FIRST_LEVEL;
                    armSystem.armIdle();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> { //0.1 Seconds AFTER heading to Backdrop
                    tiltBox = true; //Tilt Box
                    armSystem.armIdle();
                    intakeSystem.boxReverseSweep();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> { //0.3 Seconds AFTER Waiting 1 Second
                    intakeSystem.terminateBoxSweeper();
                    tiltBox = false;
                    armSystem.armIdle(); //Extra Restrictions to get Box Down Safely
                    target = Constants.LIFT_LEVEL_ZERO;
                })
                .strafeTo(parkingPose)
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
                telemetry.addData("Pixel Count", pixelDetector.getCount());

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
                power = Math.max(power, -0.17);
                scoreAllowed = false;
            } else { //Going up
                power = Math.min(power, 1); //Power Range 0 -> 1;
            }
            leftSlide.setPower(power);
            rightSlide.setPower(power);
            if(leftSlide.getCurrentPosition() > 15) {
                rightSlideRest = false;
                scoreAllowed = true;
            }
            if(pid < 0) {
                armSystem.armIdle();
            }
            if(scoreAllowed) {

                if(tiltBox) {
                    armSystem.armScore();
                } else {
                    armSystem.armIdle();
                }

            }

            if( (target == 0)  ) { //Ensure Lifts are Fully Down (Observation: Right Slide Mainly Issues)
                armSystem.armIdle();
                scoreAllowed = false;
                tiltBox = false;
                while( (rightSlide.getCurrentPosition() > 1 || rightSlide.getCurrentPosition() <= -1) && !rightSlideRest) {
                    rightSlide.setPower( (Math.signum(rightSlide.getCurrentPosition() * -1) * 0.3) );
                    if(rightSlide.getCurrentPosition() < 1 || rightSlide.getCurrentPosition() >= -1) {
                        rightSlideRest = true;
                        rightSlide.setPower(0);
                        break;
                    }
                }
                while(leftSlide.getCurrentPosition() > 0) {
                    leftSlide.setPower(-0.3);
                    if(leftSlide.getCurrentPosition() == 0) {
                        leftSlide.setPower(0);
                        break;
                    }
                }
            }

            if(rightSlideRest) {
                armSystem.dePower();
                scoreAllowed = false;
                tiltBox = false;
            }
        }
    }

}



