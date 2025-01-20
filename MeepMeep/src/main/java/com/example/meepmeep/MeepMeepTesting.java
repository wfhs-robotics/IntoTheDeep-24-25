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


        TrajectoryActionBuilder redSample = myBot.getDrive().actionBuilder(new Pose2d(-35, -60, Math.toRadians(-180)))
                .strafeToLinearHeading(new Vector2d(-50, -42), Math.toRadians(45))
                .turn(Math.toRadians(35));

//                .turn(Math.toRadians(45))
//                .turn(Math.toRadians(-135))
//                .strafeTo(new Vector2d(-48, -35))//grabs first block
//                .strafeTo(new Vector2d(-50, -50))//scores
//                .turn(Math.toRadians(135))
//                .turn(Math.toRadians(-135))
//                .strafeTo(new Vector2d(-57, -35))//grabs second block
//                .strafeTo(new Vector2d(-50, -50))//scores
//                .turn(Math.toRadians(135))
//                .turn(Math.toRadians(-85))
//                .strafeTo(new Vector2d(-57, -32))//grabs third block
//                .turn(Math.toRadians(80))
//                .strafeTo(new Vector2d(-50, -50))//scores
//                .turn(Math.toRadians(-40))
//                .strafeTo((new Vector2d(-47, -12)))
//                .strafeTo((new Vector2d(-22, -12)));


        TrajectoryActionBuilder redSpecimen = myBot.getDrive().actionBuilder(new Pose2d(19, -66, Math.toRadians(-90)))
                .strafeTo(new Vector2d(8, -30))// strafe to submersible to hang pre loaded specimen
                .strafeTo(new Vector2d(36, -34))
                .strafeTo(new Vector2d(36, -13))
                .turn(Math.toRadians((-180))) // turns to push first block
                .strafeTo(new Vector2d(47, -13))// pushes first block
                .strafeTo(new Vector2d(47, -62))
                .strafeTo(new Vector2d(47, -13))
                .strafeTo(new Vector2d(57, -13))//pushes second block
                .strafeTo(new Vector2d(57, -62))
                .strafeTo(new Vector2d(57, -13))
                .strafeTo(new Vector2d(62, -13))//pushes third block
                .strafeTo(new Vector2d(62, -62))
                .strafeTo(new Vector2d(47, -55))
                .strafeTo(new Vector2d(47, -64))//grab the first block
                .strafeTo(new Vector2d(28, -49))
                .turn(Math.toRadians((180)))
                .strafeTo(new Vector2d(6, -30))//hangs first block
                .strafeTo(new Vector2d(47, -55))
                .turn(Math.toRadians((-180)))
                .strafeTo(new Vector2d(47, -64))//grabs second block
                .strafeTo(new Vector2d(28, -49))
                .turn(Math.toRadians((180)))
                .strafeTo(new Vector2d(4, -30))//hangs second block
                .strafeTo(new Vector2d(47, -55))
                .turn(Math.toRadians((-180)))
                .strafeTo(new Vector2d(47, -64))// grabs third block
                .strafeTo(new Vector2d(28, -49))
                .turn(Math.toRadians((180)))
                .strafeTo(new Vector2d(2, -30))//hangs third block
                .strafeTo(new Vector2d(60, -63)); //park

       myBot.runAction(redSpecimen.build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}