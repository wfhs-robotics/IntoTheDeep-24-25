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
    public Action slideArmHigh() { return new InstantAction(() -> pid.runSlide(2000)); }

    public Action slideArmLow() { return new InstantAction(() -> pid.runSlide(1000)); }

    public Action moveArm() { return new InstantAction(() -> pid.runArm(100)); }

    public Action releaseObject() { return new InstantAction(() -> robot.intake.setPower(-1)); }

    public Action intake() { return new InstantAction(() -> robot.intake.setPower(1)); }

    public Action openSlideClawRight() { return new InstantAction(() -> robot.slideClawRight.setPosition(0)); }

    public Action closeSlideClawRight() { return new InstantAction(() -> robot.slideClawRight.setPosition(1)); }

    public Action openSlideClawLeft() { return new InstantAction(() -> robot.slideClawLeft.setPosition(0)); }

    public Action closeSlideClawLeft() { return new InstantAction(() -> robot.slideClawLeft.setPosition(1)); }

    public Action openSlideClaw() {
        return new SequentialAction(
                openSlideClawLeft(),
                openSlideClawRight());
    }

    public Action closeSlideClaw() {
        return new SequentialAction(
                closeSlideClawLeft(),
                closeSlideClawRight());
    }

    public Action stackLowBucket() {
        return new SequentialAction(
                slideArmLow(),
                new SleepAction(1),
                releaseObject());
    }
    public Action stackHighBucket() {
        return new SequentialAction(
                slideArmHigh(),
                new SleepAction(1),
                releaseObject());
    }

}
