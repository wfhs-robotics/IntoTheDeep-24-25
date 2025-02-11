package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
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
    public static int y1= -50;
    public static int heading = -145;

    public static int slidePos = 2900;
    public static int armPos = 3825;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        ActionsCustom actionsCustom = new ActionsCustom(robot);


        Pose2d startPose = new Pose2d(-36, -56, Math.toRadians(-180));
        if (TuningOpModes.DRIVE_CLASS.equals(PinpointDrive.class)) {
            PinpointDrive drive = new PinpointDrive(hardwareMap, startPose);

            waitForStart();

            TrajectoryActionBuilder startingBlock = drive.actionBuilder(startPose)
                    .strafeTo(new Vector2d(-36, -52))
                    .strafeToLinearHeading(new Vector2d(x1, y1), Math.toRadians(-135));


            TrajectoryActionBuilder turnToBlock1 = drive.actionBuilder(new Pose2d(new Vector2d(-60, -50), Math.toRadians(-135)))
                    .turn(Math.toRadians(heading));
            TrajectoryActionBuilder toBasket1 = drive.actionBuilder(new Pose2d(new Vector2d(-60, -50), Math.toRadians(-280)))
                    .turn(Math.toRadians(-heading));

            Actions.runBlocking( new SequentialAction(
                    actionsCustom.closeArmClaw(),
                    new ParallelAction(
                            actionsCustom.arm(3825),
                            startingBlock.build(),
                            new SequentialAction(
                                new SleepAction(.5),
                                actionsCustom.slideArm(2625)
                            )
                    ),
                    new SleepAction(.5),
                    actionsCustom.openArmClaw(),
                    new SleepAction(.5),
                    new ParallelAction(
                            turnToBlock1.build(),
                            new SequentialAction(
                                    new SleepAction(.5),
                                    new ParallelAction(
                                        actionsCustom.slideArm(slidePos),
                                        actionsCustom.arm(1000)
                                    ),
                                    new SleepAction(.5),
                                    actionsCustom.arm(700),
                                    new SleepAction(2),
                                    actionsCustom.closeArmClaw(),
                                    new SleepAction(.5),
                                    new ParallelAction(
                                            toBasket1.build(),
                                            actionsCustom.arm(0),
                                            actionsCustom.slideArm(0)
                                    )
//                                    new SleepAction(2),
//                                    actionsCustom.slideArm(2625),
//                                    new SleepAction(1),
//                                    actionsCustom.openArmClaw(),
//                                    new SleepAction(2)

                            )
                    ),
                    new SleepAction(1.5)
            ));

            ArmPosStorage.armPos = robot.arm.getCurrentPosition();

        } else {
            throw new RuntimeException();
        }
    }
}