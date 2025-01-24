package org.firstinspires.ftc.teamcode.Misc;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class ServoTuning extends OpMode {
    Hardware robot;
    public static double clawLeft = 0;
    public static double clawRight = 0;
    public static double wristPos = 0;


    @Override
    public void init() {
        robot = new Hardware();
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        robot.slideClawLeft.setPosition(clawLeft);
        robot.slideClawRight.setPosition(clawRight);
        robot.wrist.setPosition(wristPos);
    }
}
