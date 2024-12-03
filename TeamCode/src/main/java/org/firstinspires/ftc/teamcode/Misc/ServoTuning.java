package org.firstinspires.ftc.teamcode.Misc;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class ServoTuning extends OpMode {
    Hardware robot;
    public static double arm1Pos = 0;
    public static double arm2Pos = 0;
    public static double claw = 0;

    // Arm pickup mode, arm 1 -- .25, arm 2 -- .55
    // Arm away mode - 0s

    @Override
    public void init() {
        robot = new Hardware();
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        robot.armLeft.setPosition(arm1Pos);
        robot.armRight.setPosition(1- arm1Pos);
        robot.arm2.setPosition(arm2Pos);
        robot.claw.setPosition(claw);

    }
}
