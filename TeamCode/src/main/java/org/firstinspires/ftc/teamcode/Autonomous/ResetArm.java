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
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Misc.ActionsCustom;
import org.firstinspires.ftc.teamcode.Misc.ArmPosStorage;
import org.firstinspires.ftc.teamcode.Misc.Hardware;
import org.firstinspires.ftc.teamcode.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.tuning.TuningOpModes;
@Config
@Autonomous
public final class ResetArm extends LinearOpMode {
    Hardware robot = new Hardware();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);


        ActionsCustom actionsCustom = new ActionsCustom(robot);

        Pose2d startPose = new Pose2d(-36, -56, Math.toRadians(-180));
        if (TuningOpModes.DRIVE_CLASS.equals(PinpointDrive.class)) {
            PinpointDrive drive = new PinpointDrive(hardwareMap, startPose);

            Action armReset = new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    robot.arm.setTargetPosition(-ArmPosStorage.armPos + 100);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setVelocity(2100);
                    return false;
                }
            };

            waitForStart();



            Actions.runBlocking( new SequentialAction(
                    new ParallelAction(
                    actionsCustom.wristReset(),
                    armReset
                    ),
                    new SleepAction(5)
            ));

            ArmPosStorage.armPos = robot.arm.getCurrentPosition();

        } else {
            throw new RuntimeException();
        }
    }
}