package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.Vector;


public class MeepMeepTesting {
    public static void main(String[] args) {

        Pose2d initPose;
        Vector2d midwayVector;
        Vector2d scoringVector;
        Vector2d leftVector;
        Vector2d rightVector;
        Vector2d finalPose;
        Vector2d parkingPose;
        Vector2d centerVector;

        initPose = new Pose2d(13, 58, Math.toRadians(-270));
        midwayVector = new Vector2d(13, 35);
        leftVector = new Vector2d(22,35);
        rightVector = new Vector2d(0, 35);
        centerVector = new Vector2d(13, 30);
        scoringVector = new Vector2d(47, 35);
        parkingPose = new Vector2d(47,60);
        finalPose = new Vector2d(60, 60);
        double backwardsDistance = 3;
        MeepMeep meepMeep = new MeepMeep(800);

        Vector2d finalCenterVector = centerVector;
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 14.1)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(initPose)
                                .lineToConstantHeading(midwayVector)
                                .strafeTo(rightVector)
                                .lineToConstantHeading(new Vector2d(rightVector.getX(), rightVector.getY() + backwardsDistance))
                                .strafeTo(new Vector2d(midwayVector.getX(), rightVector.getY() + backwardsDistance))
                                .turn(Math.toRadians(90))
                                .lineToConstantHeading(new Vector2d(scoringVector.getX(), rightVector.getY() + backwardsDistance))
                                .strafeTo(parkingPose)
                                .build()
//                                .lineToConstantHeading(midwayVector)
//                                .strafeTo(rightVector)
//                                .back(3)
//                                .turn(Math.toRadians(90))
//                                .strafeTo(new Vector2d(35, -36))
//                                .lineToConstantHeading(scoringVector)
//                                .strafeTo(new Vector2d(47,-56))
//                                .waitSeconds(3)
//                                .strafeTo(parkingPose)
//                                .lineToConstantHeading(finalPose)
//                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}