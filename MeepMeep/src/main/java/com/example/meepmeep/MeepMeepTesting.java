package com.example.meepmeep;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 9.5)
                .setDimensions(17.7, 12)
                .build();

//        Action moveArm = new SequentialAction(
//
//        );

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-35, -55, Math.toRadians(-180)))
                                .strafeTo(new Vector2d(-35, -15))
                                .strafeTo(new Vector2d(-20, -15))

                                //drop pixel

//                                .strafeTo(new Vector2d(60))
//                .splineToConstantHeading(new Vector2d(5, -29), Math.toRadians(90))
//                .lineToY(-46)
//                .splineTo(new Vector2d(45, -10), Math.toRadians(0))
//                .turn(Math.toRadians(-90))
//                .strafeTo(new Vector2d(45, -60))
//                .strafeTo(new Vector2d(45, -10))
//                .strafeTo(new Vector2d(55, -10))
//                .strafeTo(new Vector2d(55, -60))
//                .strafeTo(new Vector2d(55, -60))
//                .strafeTo(new Vector2d(55, -10))
//                .strafeTo(new Vector2d(63, -10))
//                .strafeTo(new Vector2d(63, -60))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}