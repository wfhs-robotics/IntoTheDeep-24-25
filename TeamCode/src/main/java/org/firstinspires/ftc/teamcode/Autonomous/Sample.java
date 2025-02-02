package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Misc.ActionsCustom;
import org.firstinspires.ftc.teamcode.Misc.ArmPosStorage;
import org.firstinspires.ftc.teamcode.Misc.Hardware;
import org.firstinspires.ftc.teamcode.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.tuning.TuningOpModes;
@Config
@Autonomous
public final class Sample extends LinearOpMode {
    Hardware robot = new Hardware();
    public static int x1 = -60;
    public static int y1= -34;
    public static int heading = 90;
    public static int turn = 70;

    public static int turn1 = 68;

    public static int armPickupPos = 600;
    public static int slideArmPickupPos = 1700;

    public static int armPickupPos1 = 650;
    public static int slideArmPickupPos1 = 2050;

    public static int armHighPos = 3700;
    public static int slideArmHighPosInit = 2300;
    public static int slideArmHighPos = 2200;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        ActionsCustom actionsCustom = new ActionsCustom(robot);


        Pose2d startPose = new Pose2d(-36, -56, Math.toRadians(-180));
        if (TuningOpModes.DRIVE_CLASS.equals(PinpointDrive.class)) {
            PinpointDrive drive = new PinpointDrive(hardwareMap, startPose);


            waitForStart();

            TrajectoryActionBuilder startingBlock = drive.actionBuilder(startPose)
                    .strafeToLinearHeading(new Vector2d(-50, -42), Math.toRadians(-135));//scores starting sample
            TrajectoryActionBuilder toBucket = drive.actionBuilder(new Pose2d(new Vector2d(-50, -42), Math.toRadians(-135)))
                    .strafeTo(new Vector2d(-60, -52));//scores starting sample

            TrajectoryActionBuilder toBlock1 = drive.actionBuilder(new Pose2d(-50, -42,  Math.toRadians(45)))
                    .turn(Math.toRadians(46));

            TrajectoryActionBuilder stackBlock1 = drive.actionBuilder(new Pose2d(-50, -42, Math.toRadians(45 + 46)))
                    .turn(Math.toRadians(-46));

            TrajectoryActionBuilder toBlock2 = drive.actionBuilder(new Pose2d(-50, -42,  Math.toRadians(45)))
                    .turn(Math.toRadians(turn1));

            TrajectoryActionBuilder stackBlock2 = drive.actionBuilder(new Pose2d(-50, -42,  Math.toRadians(45 + turn1)))
                    .turnTo(Math.toRadians(45));


            Actions.runBlocking( new SequentialAction(
                    actionsCustom.closeArmClaw(),

                    // Stack Initial Block
                    new ParallelAction(
                            actionsCustom.stackHigh(armHighPos, slideArmHighPosInit - 1000),
                            startingBlock.build()

                    ),
                    actionsCustom.slideArmHigh(slideArmHighPosInit + 1000),

                    toBucket.build(),

                    new SleepAction(2),
                    actionsCustom.openArmClaw(),
                    new SleepAction(.5),
                    actionsCustom.openArmClaw()
            ));


//                    //Stack Block 1
//                    new ParallelAction(
//                            actionsCustom.pickupHeight(armPickupPos, slideArmPickupPos),
//                            new SleepAction(3),
//                            toBlock1.build()
//                    ),
//                    actionsCustom.pickupObject(),
//                    new ParallelAction(
//                            actionsCustom.stackHigh(armHighPos, slideArmHighPos),
//                            stackBlock2.build()
//                    ),
//                    new SleepAction(2),
//                    actionsCustom.releaseObject(),
//
//                    //Stack Block 2
//            new ParallelAction(
//                    actionsCustom.pickupHeight(armPickupPos1, slideArmPickupPos1),
//                    new SleepAction(3),
//                    toBlock2.build()
//            ),
//                    actionsCustom.pickupObject(),
//                    new ParallelAction(
//                            actionsCustom.stackHigh(armHighPos, slideArmHighPos),
//                            stackBlock1.build()
//                   ),
//                    new SleepAction(2),
//                  actionsCustom.releaseObject()
//            ));
            ArmPosStorage.armPos = robot.arm.getCurrentPosition();

        } else {
            throw new RuntimeException();
        }
    }
}