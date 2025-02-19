package org.firstinspires.ftc.teamcode.Misc;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
public class ActionsCustom {
    Hardware robot;



    public ActionsCustom(Hardware robot) { this.robot = robot; }

    public Action intakeBack() { return new InstantAction(() -> robot.intake.setPower(-1)); }

    public Action intakeStop() { return new InstantAction(() -> robot.intake.setPower(0)); }

    public Action intake() { return new InstantAction(() -> robot.intake.setPower(1)); }

    public Action wristReset() { return new InstantAction(() -> robot.wrist.setPosition(.65)); }

    public Action arm(int armPos) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.arm.setTargetPosition(armPos);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm.setVelocity(2800);
                return false;
            }
        };
    }

    public Action slideArm(int slidePos) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.slideArm.setTargetPosition(slidePos);
                robot.slideArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.slideArm.setVelocity(2100);
                return false;
            }
        };
    }

    public Action slide(int slidePos) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.slide.setTargetPosition(slidePos);
                robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.slide.setVelocity(2100);
                return false;
            }
        };
    }


    public Action openClaw() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.slideClawLeft.setPosition(1);
                robot.slideClawRight.setPosition(0);
                return false;
            }
        };
    }

    public Action closeClaw() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.slideClawLeft.setPosition(0);
                robot.slideClawRight.setPosition(1);
                return false;
            }
        };
    }
    public Action openArmClaw() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.armClawLeft.setPosition(0);
                robot.armClawRight.setPosition(1);
                return false;
            }
        };
    }

    public Action closeArmClaw() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.armClawLeft.setPosition(1);
                robot.armClawRight.setPosition(0);
                return false;
            }
        };
    }
}
