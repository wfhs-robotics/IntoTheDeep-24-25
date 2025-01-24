package org.firstinspires.ftc.teamcode.Misc;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp(name = "Presentation")
public class Presentation extends LinearOpMode {
    public static int slideTarget = 0;
    public static int armTarget = 0;
    public static int slideArmTarget = 0;
    public static boolean intake = false;
    public static boolean clawOpen = false;
    public static int intakePower = 1;
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

                if(intake) {
                    robot.intake.setPower(intakePower);
                }
                if(clawOpen) {
                    robot.slideClawRight.setPosition(0);
                    robot.slideClawLeft.setPosition(1);
                } else {
                    robot.slideClawRight.setPosition(1);
                    robot.slideClawLeft.setPosition(0);
                }
            }
        }
    }
}
