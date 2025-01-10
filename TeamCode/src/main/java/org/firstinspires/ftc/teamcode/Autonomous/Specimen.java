package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.tuning.TuningOpModes;
@Autonomous
public final class Specimen extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startPose = new Pose2d(-36, -56, Math.toRadians(-180));
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);


            waitForStart();
            Action openClaw =  new SequentialAction(
                    sleepAction(1000)
            );

            TrajectoryActionBuilder toHumanPlayer = drive.actionBuilder(startPose)
                    .strafeTo(new Vector2d(9, -46)) // Drive forward
                    .strafeTo(new Vector2d(45, -46)) // To human player
                    .turn(Math.toRadians(-180)); // Turn to drop

            TrajectoryActionBuilder park = drive.actionBuilder(new Pose2d(45, -46, Math.toRadians(-90)))
                    .turn(Math.toRadians(180)) // Turn back
                    .strafeTo(new Vector2d(45, -60)); // Reverse to park

            Actions.runBlocking( new SequentialAction(
                    toHumanPlayer.build(),
                    openClaw,
                    park.build()
            ));

//            Actions.runBlocking(
//                drive.actionBuilder(startPose)
//                        .strafeTo(new Vector2d(9, -46))
//                        .strafeTo(new Vector2d(45, -46))
//                        .turn(Math.toRadians(-180))
//                        .turn(Math.toRadians(180))
//
//                        .strafeTo(new Vector2d(25, -34))
//                        .strafeTo(new Vector2d(25, -10))
//                        .strafeTo(new Vector2d(15, -10))
//
//
////                        .splineToConstantHeading(new Vector2d(5, -31), Math.toRadians(90))
////                        .lineToY(-46)
////                        .splineTo(new Vector2d(45, -10), Math.toRadians(0))
////                        .turn(Math.toRadians(-90))
////                        .strafeTo(new Vector2d(45, -60))
////                        .strafeTo(new Vector2d(45, -10))
////                        .strafeTo(new Vector2d(55, -10))
////                        .strafeTo(new Vector2d(55, -60))
////                        .strafeTo(new Vector2d(55, -60))
////                        .strafeTo(new Vector2d(55, -10))
////                        .strafeTo(new Vector2d(63, -10))
////                        .strafeTo(new Vector2d(63, -60))
//                        .build());

        } else {
            throw new RuntimeException();
        }
    }
    public Action sleepAction(int milliseconds) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                sleep(milliseconds);
                return false;
            }
        };
    }
}
