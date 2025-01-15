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
import org.firstinspires.ftc.teamcode.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.tuning.TuningOpModes;
@Autonomous
public final class Sample extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startPose = new Pose2d(-36, -56, Math.toRadians(-180));
        if (TuningOpModes.DRIVE_CLASS.equals(PinpointDrive.class)) {
            PinpointDrive drive = new PinpointDrive(hardwareMap, startPose);


            waitForStart();
            Action openClaw =  new SequentialAction(
                    sleepAction(1000)
            );

            TrajectoryActionBuilder stackPath = drive.actionBuilder(startPose)
                    .turn(Math.toRadians(360));//scores starting sample
//                    .turn(Math.toRadians(45));

//                    .turn(Math.toRadians(-135))
//                    .strafeTo(new Vector2d(-48, -35))//grabs first block
//                    .strafeTo(new Vector2d(-50, -50))//scores
//                    .turn(Math.toRadians(135))
//                    .turn(Math.toRadians(-135))
//                    .strafeTo(new Vector2d(-57, -35))//grabs second block
//                    .strafeTo(new Vector2d(-50, -50))//scores
//                    .turn(Math.toRadians(135))
//                    .turn(Math.toRadians(-85))
//                    .strafeTo(new Vector2d(-57, -32))//grabs third block
//                    .turn(Math.toRadians(80))
//                    .strafeTo(new Vector2d(-50, -50))//scores
//                    .turn(Math.toRadians(-40))
//                    .strafeTo((new Vector2d(-47, -12)))
//                    .strafeTo((new Vector2d(-22, -12)));



            Actions.runBlocking( new SequentialAction(
                    stackPath.build()

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