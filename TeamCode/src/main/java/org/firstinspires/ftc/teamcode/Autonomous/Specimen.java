package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

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
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.tuning.TuningOpModes;
@Autonomous
public final class Specimen extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ActionsCustom actionsCustom = new ActionsCustom(hardwareMap);

        Pose2d startPose = new Pose2d(-36, -56, Math.toRadians(-180));
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);


            waitForStart();

            TrajectoryActionBuilder initalHang = drive.actionBuilder(startPose)
                    .strafeTo(new Vector2d(8, -30));



            Actions.runBlocking( new SequentialAction(
                    new ParallelAction(
                            initalHang.build(),
                            actionsCustom.slideHigh(2000)
                    ),
                    new SleepAction(.5),
                    actionsCustom.slideZero()
            ));


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
