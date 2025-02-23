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
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.tuning.TuningOpModes;
@Config
@Autonomous
public final class Specimen extends LinearOpMode {
    Hardware robot = new Hardware();
    public static int x1 = 5;
    public static int y1 = -30;
    public static int heading = -90;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        ActionsCustom actionsCustom = new ActionsCustom(robot);


        Pose2d startPose = new Pose2d(19, -66, Math.toRadians(-90));
        if (TuningOpModes.DRIVE_CLASS.equals(PinpointDrive.class)) {
            PinpointDrive drive = new PinpointDrive(hardwareMap, startPose);


            waitForStart();

            TrajectoryActionBuilder initialHang = drive.actionBuilder(startPose)
                    .strafeTo(new Vector2d(2, -36));
            TrajectoryActionBuilder hang1 = drive.actionBuilder(new Pose2d(new Vector2d(48, -59), Math.toRadians(90)))
                    .strafeToLinearHeading(new Vector2d(5 , -31), Math.toRadians(-90));
            TrajectoryActionBuilder hang2 = drive.actionBuilder(new Pose2d(new Vector2d(48, -63), Math.toRadians(90)))
                    .strafeToLinearHeading(new Vector2d(15 , -32), Math.toRadians(-90));


            TrajectoryActionBuilder toHumanPlayer = drive.actionBuilder(new Pose2d(new Vector2d(2,-36), Math.toRadians(-90)))
                    .strafeToLinearHeading(new Vector2d(48, -55), Math.toRadians(90))
                    .strafeTo(new Vector2d(48, -63));
            TrajectoryActionBuilder forward = drive.actionBuilder(new Pose2d(new Vector2d(48, -63), Math.toRadians(90)))
                    .strafeTo(new Vector2d(48 , -59));
            TrajectoryActionBuilder pushBlocks = drive.actionBuilder(new Pose2d(new Vector2d(5, -31), Math.toRadians(-90)))
                    // Drive to spikes
                    .strafeTo(new Vector2d(5, -42))
                    .strafeToLinearHeading(new Vector2d(48 , -35), Math.toRadians(90))
                    .strafeTo(new Vector2d(48, -15))

                    //Push Block 1
                    .strafeTo(new Vector2d(56, -15))
                    .strafeTo(new Vector2d(56, -60))
                    .strafeTo(new Vector2d(56, -50));


            TrajectoryActionBuilder grabSpecimen = drive.actionBuilder(new Pose2d(new Vector2d(55, -60), Math.toRadians(90)))
                    //Grab Specimen 1
                    .strafeTo(new Vector2d(48, -55))
                    .strafeTo(new Vector2d(48, -63));
            TrajectoryActionBuilder park = drive.actionBuilder(new Pose2d(new Vector2d(15, -32), Math.toRadians(-90)))
                    //Grab Specimen 1
                    .strafeTo(new Vector2d(60, -72));






            Actions.runBlocking( new SequentialAction(
                    //Initial Hang
                    actionsCustom.slideArm(0),
                    actionsCustom.arm(300),

                    new ParallelAction(
                            initialHang.build(),
                            actionsCustom.slide(-1800)
                    ),
                    actionsCustom.slide(-1400),

                    // Block 1
                    new ParallelAction(
                            actionsCustom.slide(0),
                            actionsCustom.openClaw(),
                            toHumanPlayer.build()
                    ),
                    actionsCustom.closeClaw(),
                    new SleepAction(.2),
                    actionsCustom.slide(-500),
                    forward.build(),

                    // Hang 1
                    new ParallelAction(
                            actionsCustom.slide(-1900),
                            hang1.build()
                    ),

                    actionsCustom.slide(-1300),
                    new SleepAction(.4),

                    // Drive to and Push blocks
                    new ParallelAction(
                            actionsCustom.openClaw(),
                            actionsCustom.slide(0),
                            pushBlocks.build()
                    ),
                    new SleepAction(1.8),
                    grabSpecimen.build(),

                    //Grab Block 2
                    actionsCustom.closeClaw(),
                    new SleepAction(.2),
                    actionsCustom.slide(-500),
                    forward.build(),


                    // Hang Block 2
                    new ParallelAction(
                            actionsCustom.slide(-1800),
                            hang2.build()
                    ),
                    actionsCustom.slide(1600),
                    new SleepAction(.5),
                    actionsCustom.openClaw(),
                    new ParallelAction(
                            actionsCustom.slide(0),
                            park.build()
                    )
            ));

            ArmPosStorage.slideArmPos = robot.slideArm.getCurrentPosition();
        } else {
            throw new RuntimeException();
        }
    }
}
