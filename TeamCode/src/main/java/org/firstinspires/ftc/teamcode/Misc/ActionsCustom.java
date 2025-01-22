package org.firstinspires.ftc.teamcode.Misc;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Config
public class ActionsCustom {
    Hardware robot;



    public ActionsCustom(Hardware robot) { this.robot = robot; }

    public Action intakeBack() { return new InstantAction(() -> robot.intake.setPower(-1)); }

    public Action intakeStop() { return new InstantAction(() -> robot.intake.setPower(0)); }

    public Action intake() { return new InstantAction(() -> robot.intake.setPower(1)); }

    public Action wristDown() { return new InstantAction(() -> robot.wrist.setPosition(.3)); }
    public Action wristPickup() { return new InstantAction(() -> robot.wrist.setPosition(.5)); }
    public Action wristReset() { return new InstantAction(() -> robot.wrist.setPosition(0)); }

    public Action armHigh(int armPos) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.arm.setTargetPosition(armPos);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm.setVelocity(2100);
                return false;
            }
        };
    }

    public Action armZero() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.arm.setTargetPosition(0);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm.setVelocity(2100);
                return false;
            }
        };
    }

    public Action slideArmHigh(int slidePos) {
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

    public Action slideArmZero() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.slideArm.setTargetPosition(0);
                robot.slideArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.slideArm.setVelocity(2100);
                return false;
            }
        };
    }
    public Action slideArmPickupHeight(int slideHeight) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.slideArm.setTargetPosition(slideHeight);
                robot.slideArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.slideArm.setVelocity(2100);
                return false;
            }
        };
    }

    public Action armPickupHeight(int armPickupHeight) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.arm.setTargetPosition(armPickupHeight);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm.setVelocity(2100);
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

    public Action slideZero() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.slide.setTargetPosition(0);
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
                robot.clawLeft.setPosition(1);
                robot.clawRight.setPosition(0);
                return false;
            }
        };
    }

    public Action closeClaw() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.clawLeft.setPosition(0);
                robot.clawRight.setPosition(1);
                return false;
            }
        };
    }



    public Action releaseObject() {
        return new SequentialAction(
                intakeBack(),
                new SleepAction(0.8),
                intakeStop());
    }
    public Action pickupObject() {
        return new SequentialAction(
                intake(),
                new SleepAction(0.8),
                intakeStop());
    }

    public Action stackHigh(int armPos, int slidePos) {
        return new SequentialAction(
                wristDown(),
                armHigh(armPos),
                slideArmHigh(slidePos));
    }
    public Action pickupHeight(int armHeight, int slideHeight) {
        return new SequentialAction(
                wristPickup(),
                armPickupHeight(armHeight),
                slideArmPickupHeight(slideHeight));
    }



}
