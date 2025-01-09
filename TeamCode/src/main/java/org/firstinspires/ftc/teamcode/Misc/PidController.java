package org.firstinspires.ftc.teamcode.Misc;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Config
public class PidController {
    // WATCH THIS VIDEO FOR TUNING THE PID LOOPS https://youtu.be/E6H6Nqe6qJo?si=ZiYEEjVgNf5uhZ-E

    PIDController controllerSlide;
    PIDController controllerArm;
    PIDController controllerSlideArm;
    Gamepad gamepad2;
    Telemetry telemetry;
    Hardware robot = new Hardware();
    public static double pSlide = 0.01, iSlide = 0, dSlide = 0.0001, fSlide = 0.05;
    public static double pSlideArm = 0.01, iSlideArm = 0, dSlideArm = 0.0001, fSlideArm = 0.05;
    public static double pArm = 0.01, iArm = 0, dArm = 0.0001, fArm = 0.05;
    public static int targetSlide = 0;
    public static int targetArm = 0;
    public static int targetSlideArm = 0;
    public static int prevTargetSlide = 0;
    public static int prevTargetArm = 0;
    public static int prevTargetSlideArm = 0;
    double slidePower = 0;
    double slideArmPower = 0;
    double armPower = 0;



    public static double ticks_in_degrees = 145.1/360;

    public PidController(Telemetry telemetry, Gamepad gamepad2) {
        this.telemetry = telemetry;
        this.gamepad2 = gamepad2;
    }


    public void init(HardwareMap hardwareMap, Hardware robot) {
        this.robot = robot;
        controllerSlide =  new PIDController(pSlide, iSlide, dSlide);
        controllerArm =  new PIDController(pArm, iArm, dArm);
        controllerSlideArm =  new PIDController(pSlideArm, iSlideArm, dSlideArm);

        telemetry.update();
    }
    //Move only the slide
    public void runSlide(int targetSlideParam) {
        run(false, false, targetSlideParam, targetArm, targetSlideArm); // Keep the arm at its current position
    }

    // Move only the arm

    public void runArm(int targetArmParam) {
        run(false, false, targetSlide, targetArmParam, targetSlideArm); // Keep the slide at its current position
    }

    public void runSlideArm(int slideArmParam) {
        run(false, false, targetSlide, targetArm, slideArmParam);
    }

    // Move both manually
    public void run(boolean manual, boolean prevStart) {
        // Call the main run method with current targetSlide and targetArm values
        run(manual, prevStart, targetSlide, targetArm, targetSlideArm);
    }

    // Main run method for both manual and automatic modes, can run both arm and slide
    public void run(boolean manual, boolean prevStart, int targetSlideParam, int targetArmParam, int targetSlideArmParam) {
        // Set PID coefficients
        controllerSlide.setPID(pSlide, iSlide, dSlide);
        controllerArm.setPID(pArm, iArm, dArm);
        controllerSlideArm.setPID(pSlideArm, iSlideArm, dSlideArm);

        // Read current positions
        int slidePos = -robot.slide.getCurrentPosition();
        int armPos = robot.arm.getCurrentPosition();
        int slideArmPos = robot.slideArm.getCurrentPosition();

        targetSlide = slidePos;
        targetArm = armPos;
        targetSlideArm = slideArmPos;

        double pidSlide = controllerSlide.calculate(robot.slide.getCurrentPosition(), targetSlide);
        double ffSlide = Math.cos(Math.toRadians(targetSlide / ticks_in_degrees)) * fSlide;
        slidePower = pidSlide + ffSlide;

        // Arm PID control
        double pidArm = controllerArm.calculate(robot.arm.getCurrentPosition(), targetArm);
        double ffArm = Math.cos(Math.toRadians(targetArm / ticks_in_degrees)) * fArm;
        armPower = pidArm + ffArm;

        double pidSlideArm = controllerSlideArm.calculate(robot.slideArm.getCurrentPosition(), targetSlideArm);
        double ffSlideArm = Math.cos(Math.toRadians(targetSlideArm / ticks_in_degrees)) * fSlideArm;
        slideArmPower = pidSlideArm + ffSlideArm;

        // Manual mode: controlled via gamepad
        if (manual) {
            // Slide control using gamepad
            if (gamepad2.right_stick_y != 0) {
                robot.arm.setPower(gamepad2.right_stick_y);
            } else {
                robot.arm.setPower(armPower);
            }

            if (gamepad2.left_stick_y != 0) {
                robot.slideArm.setPower(-gamepad2.left_stick_y);
            } else {
                robot.slideArm.setPower(slideArmPower);
            }

//            if(gamepad2.start && gamepad2.start != prevStart) {
//                if (gamepad2.left_stick_y != 0) {
//                    robot.slide.setPower(gamepad2.left_stick_y);
//                    targetSlide = slidePos; // Update targetSlide dynamically
//                }
//            }
        } else {
            // Automatic mode: controlled via target parameters
            targetSlide = targetSlideParam;
            targetArm = targetArmParam;
            targetSlideArm = targetSlideArmParam;


        }

//        if(targetArm != prevTargetArm || targetSlideArm != prevTargetSlideArm || prevTargetSlide != targetSlide) {
//            // Slide PID control
//            double pidSlide = controllerSlide.calculate(robot.slide.getCurrentPosition(), targetSlide);
//            double ffSlide = Math.cos(Math.toRadians(targetSlide / ticks_in_degrees)) * fSlide;
//            slidePower = pidSlide + ffSlide;
//
//            // Arm PID control
//            double pidArm = controllerArm.calculate(robot.arm.getCurrentPosition(), targetArm);
//            double ffArm = Math.cos(Math.toRadians(targetArm / ticks_in_degrees)) * fArm;
//            armPower = pidArm + ffArm;
//
//            double pidSlideArm = controllerSlideArm.calculate(robot.slideArm.getCurrentPosition(), targetSlideArm);
//            double ffSlideArm = Math.cos(Math.toRadians(targetSlideArm / ticks_in_degrees)) * fSlideArm;
//            slideArmPower = pidSlideArm + ffSlideArm;
//        }
//
//        robot.slide.setPower(-slidePower);
//        robot.arm.setPower(armPower);
//        robot.slideArm.setPower(slideArmPower);





        // Telemetry for debugging
        telemetry.addData("Slide Position", slidePos);
        telemetry.addData("Arm Position", armPos);
        telemetry.addData("Slide Arm Position", slideArmPos);
        telemetry.addData("Target Slide", targetSlide);
        telemetry.addData("Target Arm", targetArm);
        telemetry.addData("Slide Arm Target ", targetSlideArm);
        telemetry.update();

        prevTargetSlide = targetSlide;
        prevTargetSlideArm = targetSlideArm;
        prevTargetArm = targetArm;
    }
}


