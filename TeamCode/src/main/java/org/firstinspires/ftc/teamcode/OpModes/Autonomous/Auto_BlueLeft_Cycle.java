package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Control.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.ConeSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.Detection;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Arm;

@Autonomous(name = "Auto_BlueLeft_Cycle", preselectTeleOp = "PowerPlay_TeleOp")
public class Auto_BlueLeft_Cycle extends LinearOpMode {

    DriveTrain driveTrain;
    Lift liftSystem;
    Arm armSystem;
    ConeSensor coneSensor;
    Detection detectionSystem;

    Pose2d initPose;
    Vector2d midWayPose, lowJunctionMidPose;
    Vector2d dropConePose, ConeStackPose;
    Vector2d parkingLocationOne, parkingLocationTwo, parkingLocationThree;

    public enum TrajectoryState { PRELOAD, CS, CS_TO_MJ, PARK, IDLE }
    TrajectoryState trajectoryState = TrajectoryState.PRELOAD;

    public static int identifiedLocation = 2;
    public static int targetPosition = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        driveTrain = new DriveTrain(hardwareMap);
        liftSystem = new Lift(hardwareMap);
        armSystem = new Arm(hardwareMap);
        coneSensor = new ConeSensor(hardwareMap);
        detectionSystem = new Detection(hardwareMap);
        ElapsedTime time = new ElapsedTime();

        liftSystem.init();
        armSystem.init();
        coneSensor.init();
        detectionSystem.init();

        initPose = new Pose2d(-54, 36, Math.toRadians(0));
        midWayPose = new Vector2d(-36, 36);
        dropConePose = new Vector2d(-36, 29);
        lowJunctionMidPose = new Vector2d(-16, 36);
        ConeStackPose = new Vector2d(-16, 54);


        parkingLocationOne = new Vector2d(-36, 55);
        parkingLocationTwo = new Vector2d(-36, 38);
        parkingLocationThree = new Vector2d(-36, 15);

        TrajectorySequence preload = driveTrain.trajectorySequenceBuilder(initPose)
                .lineToConstantHeading(midWayPose)
                .turn(Math.toRadians(-90))
                .lineToConstantHeading(dropConePose)
                .turn(Math.toRadians(90))
                .build();

        TrajectorySequence conestack_mid = driveTrain.trajectorySequenceBuilder(preload.end())
                .turn(Math.toRadians(90))
                .lineToConstantHeading(midWayPose)
                .turn(Math.toRadians(-90))
                .lineToConstantHeading(lowJunctionMidPose)
                .build();

        TrajectorySequence conestack_stack = driveTrain.trajectorySequenceBuilder(preload.end())
                .turn(Math.toRadians(90))
                .lineToConstantHeading(midWayPose)
                .turn(Math.toRadians(-90))
                .lineToConstantHeading(ConeStackPose)
                .build();



        TrajectorySequence locationOne = driveTrain.trajectorySequenceBuilder(preload.end())
                .turn(Math.toRadians(90))
                .lineToConstantHeading(parkingLocationOne)
                .build();

        TrajectorySequence locationTwo = driveTrain.trajectorySequenceBuilder(preload.end())
                .turn(Math.toRadians(90))
                .lineToConstantHeading(parkingLocationTwo)
                .build();

        TrajectorySequence locationThree = driveTrain.trajectorySequenceBuilder(preload.end())
                .turn(Math.toRadians(90))
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
                int state = liftSystem.leftSlide.getCurrentPosition();
                double pid = liftSystem.controller.calculate(state, targetPosition);
                double power = pid + Constants.Kf;
                liftSystem.leftSlide.setPower(power);

                switch (trajectoryState) {
                    case PRELOAD:
                        targetPosition = Constants.LIFT_SECOND_LEVEL;
                        time.reset();
                        if (!driveTrain.isBusy()) {
                            armSystem.openArm();
                            sleep(500);
                            trajectoryState = TrajectoryState.CS;
                            targetPosition = 0;
                        }
                        break;
                    case CS:
                        driveTrain.followTrajectorySequenceAsync(conestack_mid);
                        if(!driveTrain.isBusy()) {
                            targetPosition = Constants.LIFT_FIRST_LEVEL;
                            driveTrain.followTrajectorySequenceAsync(conestack_stack);
                            sleep(500);
                            armSystem.openArm();

                        }
                        break;
                    case PARK:
                        armSystem.closeArm();
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

            }
        }
    }

}


