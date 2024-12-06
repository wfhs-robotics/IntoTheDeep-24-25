package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Misc.Hardware;


@Config
@TeleOp(name = "PidControllerTuning", group = "Test")
public class PidControllerTuning extends OpMode {
    // WATCH THIS VIDEO FOR TUNING THE PID LOOPS https://youtu.be/E6H6Nqe6qJo?si=ZiYEEjVgNf5uhZ-E

    PIDController controllerSlide;
    PIDController controllerArm;
    Hardware robot = new Hardware();
    public static double pSlide = 0.001, iSlide = 0, dSlide = 0.0001, fSlide = 0.03;
    public static double pArm = 0.018, iArm = 0, dArm = 0.0001, fArm = 0.03;
    public static int targetSlide = 0;
    public static int targetArm = 0;

    public static double ticks_in_degrees = 145.1/360;
    
    @Override
    public void init() {
        robot.init(hardwareMap);
        controllerSlide = new PIDController(pSlide, iSlide, dSlide);
        controllerArm = new PIDController(pArm, iArm, dArm);
    }

    @Override
    public void loop() {
        controllerSlide.setPID(pSlide, iSlide, dSlide);
        int slidePos = -robot.slide.getCurrentPosition();
        targetSlide = slidePos;
        double pidSlide = controllerSlide.calculate(slidePos, targetSlide);
        double ffSlide = Math.cos(Math.toRadians(targetSlide/ticks_in_degrees)) * fSlide;

        double slidePower = pidSlide + ffSlide;
//
        if (gamepad2.left_stick_y != 0) {
            robot.slide.setPower(gamepad2.left_stick_y);
            targetSlide = slidePos;
        } else {
            robot.slide.setPower(-slidePower);
        }

        controllerArm.setPID(pArm, iArm, dArm);
        int armPos = robot.arm.getCurrentPosition();
        targetArm = armPos;
        double pidArm = controllerArm.calculate(armPos, targetArm);
        double ffArm = Math.cos(Math.toRadians(targetArm/ticks_in_degrees)) * fArm;

        double armPower = pidArm + ffArm;

        if(gamepad2.right_stick_y != 0) {
            robot.arm.setPower(gamepad2.right_stick_y);
            targetArm = armPos;
        } else {
            robot.arm.setPower(armPower);
        }



        telemetry.addData("posSlide ", slidePos);
        telemetry.addData("posArm ", armPos);
        telemetry.addData("targetSlide ", targetSlide);
        telemetry.addData("targetArm ", targetArm);


        telemetry.update();
    }
}

