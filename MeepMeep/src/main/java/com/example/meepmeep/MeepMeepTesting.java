package com.example.meepmeep;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;
//test comment
public class MeepMeepTesting {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 9.5)
                .setDimensions(17.7, 12)
                .build();
//
//        Action moveArm = new SequentialAction(
//
//        );


        TrajectoryActionBuilder redSample = myBot.getDrive().actionBuilder(new Pose2d(new Vector2d(-36, -56), Math.toRadians(-180)))
                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(45))
                .splineTo(new Vector2d(-33, -33), Math.toRadians(45));
                
//                .splineToConstantHeading(new Vector2d(-45, -15), Math.toRadians(-45));


        TrajectoryActionBuilder redSpecimen = myBot.getDrive().actionBuilder(new Pose2d(19, -66, Math.toRadians(-90)))
                .strafeTo(new Vector2d(2, -37)) // Inital Block
                // To Human Player
                .strafeToLinearHeading(new Vector2d(48, -55), Math.toRadians(90))
                .strafeTo(new Vector2d(48, -63))

                //Hang 1
                .strafeToLinearHeading(new Vector2d(5 , -32), Math.toRadians(-90))

                // Drive to spikes
                .strafeTo(new Vector2d(5, -42))
                .strafeToLinearHeading(new Vector2d(48 , -35), Math.toRadians(90))
                .strafeTo(new Vector2d(48, -15))

                //Push Block 1
                .strafeTo(new Vector2d(55, -15))
                .strafeTo(new Vector2d(55, -60))

                //Push Block 2
                .strafeTo(new Vector2d(55, -15))
                .strafeTo(new Vector2d(67, -15))
                .strafeTo(new Vector2d(67, -60))

                //Grab Specimen 1
                .strafeTo(new Vector2d(48, -60))
                .strafeTo(new Vector2d(48, -63))

                // Hang 2
                .strafeToLinearHeading(new Vector2d(15 , -32), Math.toRadians(-90));




       myBot.runAction(redSample.build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}