package org.firstinspires.ftc.teamcode.Misc;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Actions {
    Hardware robot = new Hardware();

    public Actions(HardwareMap hardwareMap) {
        robot.init(hardwareMap);
    }

    public Action releaseObject() { return new InstantAction(() -> robot.intake.setPower(-1)); }

    public Action intake() { return new InstantAction(() -> robot.intake.setPower(1)); }

    public Action openSlideClawRight() { return new InstantAction(() -> robot.clawRight.setPosition(0)); }

    public Action closeSlideClawRight() { return new InstantAction(() -> robot.clawRight.setPosition(1)); }

    public Action openSlideClawLeft() { return new InstantAction(() -> robot.clawLeft.setPosition(0)); }

    public Action closeSlideClawLeft() { return new InstantAction(() -> robot.clawLeft.setPosition(1)); }

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


}
