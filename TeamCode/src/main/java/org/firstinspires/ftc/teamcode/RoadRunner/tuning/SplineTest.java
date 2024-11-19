package org.firstinspires.ftc.teamcode.RoadRunner.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.TankDrive;

public final class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(35, -60, 0);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

            waitForStart();

            Actions.runBlocking(
                drive.actionBuilder(startPose)
                        .strafeTo(new Vector2d(37, 37))
                        .build());
        } else {
            throw new RuntimeException();
        }
    }
}
