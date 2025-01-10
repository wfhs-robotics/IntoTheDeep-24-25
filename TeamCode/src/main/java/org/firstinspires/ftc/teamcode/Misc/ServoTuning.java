package org.firstinspires.ftc.teamcode.Misc;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class ServoTuning extends OpMode {
    Hardware robot;
    public static double armClawPos = 0;
    public static double slideClawPos = 0;
    public static double wristPos = 0;

    // Arm pickup mode, arm 1 -- .25, arm 2 -- .55
    // Arm away mode - 0s

    @Override
    public void init() {
        robot = new Hardware();
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        robot.slideClawLeft.setPosition(slideClawPos);
        robot.slideClawRight.setPosition(slideClawPos);
        robot.wrist.setPosition(wristPos);
    }
}
