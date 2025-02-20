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
    public static int heading = -113;

    public static double sleep = 1;
    public static int slideArmPos = 2625;
    public static int armPos = 3825;
    public static int noHitRef = 500;
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
                    .turn(Math.toRadians(-138));
            TrajectoryActionBuilder toBasket1 = drive.actionBuilder(new Pose2d(new Vector2d(-60, -50), Math.toRadians(-280)))
                    .turn(Math.toRadians(138));
            TrajectoryActionBuilder turnToBlock2 = drive.actionBuilder(new Pose2d(new Vector2d(-60, -50), Math.toRadians(-142)))
                    .turn(Math.toRadians(heading));
            TrajectoryActionBuilder toBasket2 = drive.actionBuilder(new Pose2d(new Vector2d(-60, -50), Math.toRadians(-142 + heading)))
                    .turn(Math.toRadians(-heading));
            TrajectoryActionBuilder turnToBlock3 = drive.actionBuilder(new Pose2d(new Vector2d(-60, -50), Math.toRadians(-heading)))
                    .turnTo(Math.toRadians(-90));
            TrajectoryActionBuilder ToBlock3 = drive.actionBuilder(new Pose2d(new Vector2d(-60, -50), Math.toRadians(-90)))
                    .strafeTo(new Vector2d(-60, -23));
            TrajectoryActionBuilder ToBasket3 = drive.actionBuilder(new Pose2d(new Vector2d(-60, -23), Math.toRadians(-90)))
                    .strafeTo(new Vector2d(-65, -23))
                    .strafeTo(new Vector2d(-65, -50));
            TrajectoryActionBuilder ToPark = drive.actionBuilder(new Pose2d(new Vector2d(-65, -50), Math.toRadians(-90)))
                    .strafeTo(new Vector2d(-60, -50))
                    .strafeTo(new Vector2d(-35, -10))
                    .turnTo(Math.toRadians(-180))
                    .strafeTo(new Vector2d(-20, -10));

            Actions.runBlocking( new SequentialAction(
                    //Init Positions
                    actionsCustom.closeArmClaw(),
                    actionsCustom.wristReset(),

                    // Stack First Block
                    new ParallelAction(
                            actionsCustom.arm(3825),
                            startingBlock.build(), // Drive to bucket
                            new SequentialAction(
                                new SleepAction(1), // sleep to avoid contact with bucket
                                actionsCustom.slideArm(2625)
                            )
                    ),
                    new SleepAction(.5),
                    actionsCustom.openArmClaw(),

                    new ParallelAction(
                            actionsCustom.arm(4000), // Move arm up to avoid bucket collision
                            turnToBlock1.build(), // Turn to first block on spike
                            actionsCustom.slideArm(2900) // Pickup length for block 1
                    ),
                    new SleepAction(.1),

                    // Grabbing block logic
                    actionsCustom.arm(1000), // Arm low
                    new SleepAction(.5), // sleep to avoid rough contact
                    actionsCustom.arm(700), // Arm down all the way
                    new SleepAction(.6),
                    actionsCustom.closeArmClaw(), // Grab block
                    new SleepAction(.55),


                    actionsCustom.arm(3825 + 300), // Start arm movement, but adding 200 to avoid bucket collision
                    new SleepAction(.8),

                    // Score second block
                    new ParallelAction(
                            toBasket1.build(),
                            actionsCustom.slideArm(2625),
                            new SequentialAction(
                                    new SleepAction(1),
                                    actionsCustom.arm(3825)
                            )
                    ),
                    actionsCustom.openArmClaw(),
                    actionsCustom.arm(armPos + 300),
                    new SleepAction(.3),
                    new ParallelAction(
                            turnToBlock2.build(),
                            actionsCustom.slideArm(2900)
                            ),

                            actionsCustom.arm(1000), // Arm low
                            new SleepAction(.5), // sleep to avoid rough contact
                            actionsCustom.arm(700), // Arm down all the way
                            new SleepAction(.8),

                    actionsCustom.closeArmClaw(),
                    new SleepAction(.5),
                    actionsCustom.arm(armPos + 300), // Start arm movement, but adding 200 to avoid bucket collision
                    new SleepAction(.8),

                    // Score second block
                    new ParallelAction(
                            toBasket2.build(),
                            actionsCustom.slideArm(slideArmPos),
                            new SequentialAction(
                                    new SleepAction(sleep),
                                    actionsCustom.arm(armPos),
                                    new SleepAction(.8),
                                    actionsCustom.openArmClaw(),
                                    new SleepAction(.5),//new
                                    actionsCustom.arm(armPos +300),
                                    new ParallelAction(
                                            actionsCustom.slideArm(0),
                                            turnToBlock3.build(),
                                            new SleepAction(.5),
                                            actionsCustom.arm(0)
                                    )


                            )
                    ),
                    new SleepAction(.5)

            ));

            ArmPosStorage.armPos = robot.arm.getCurrentPosition();

        } else {
            throw new RuntimeException();
        }
    }
}