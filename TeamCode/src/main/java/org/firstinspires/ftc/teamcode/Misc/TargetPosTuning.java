package org.firstinspires.ftc.teamcode.Misc;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp(name = "TargetPosTuning")
public class TargetPosTuning extends LinearOpMode {
    public static int slideTarget = 0;
    public static int armTarget = 0;
    public static int slideArmTarget = 0;
    Hardware robot = new Hardware();
    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        waitForStart();
        if (opModeIsActive()) {
            // Pre-run
            while (opModeIsActive()) {
                robot.slide.setTargetPosition(slideTarget);
                robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.slide.setVelocity(2100);

                robot.arm.setTargetPosition(armTarget);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm.setVelocity(2100);

                robot.slideArm.setTargetPosition(slideArmTarget);
                robot.slideArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.slideArm.setVelocity(2100);
            }
        }
    }
}
