package org.firstinspires.ftc.teamcode.Misc;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Actions {
    PidController pid;
    Hardware robot = new Hardware();

    public Actions(PidController pid, HardwareMap hardwareMap) {
        this.pid =  pid;
        robot.init(hardwareMap);
    }
    public Action slideHigh() { return new InstantAction(() -> pid.runSlide(2000)); }

    public Action slideLow() { return new InstantAction(() -> pid.runSlide(1000)); }

    public Action moveArm() { return new InstantAction(() -> pid.runArm(100)); }

    public Action openArmClaw() { return new InstantAction(() -> robot.armClaw.setPosition(0)); }

    public Action closeArmClaw() { return new InstantAction(() -> robot.armClaw.setPosition(1)); }

    public Action openSlideClaw() { return new InstantAction(() -> robot.slideClaw.setPosition(0)); }

    public Action closeSlideClaw() { return new InstantAction(() -> robot.slideClaw.setPosition(1)); }

    public Action stackLowBucket() {
        return new SequentialAction(
                slideLow(),
                new SleepAction(1),
                openArmClaw());
    }
    public Action stackHighBucket() {
        return new SequentialAction(
                slideHigh(),
                new SleepAction(1),
                openArmClaw());
    }

}
