package org.firstinspires.ftc.teamcode.Misc;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Config
public class PidController {
    // WATCH THIS VIDEO FOR TUNING THE PID LOOPS https://youtu.be/E6H6Nqe6qJo?si=ZiYEEjVgNf5uhZ-E

    PIDController controllerRight;
    PIDController controllerLeft;
    Gamepad gamepad2;
    Telemetry telemetry;
    Hardware robot = new Hardware();
    public static double pR = 0.022, iR= 0, dR= 0.0001, fR= 0.05;
    public static double pL = 0.022, iL= 0, dL= 0.0001, fL= 0.05;
    public static int target = 0;


    public static double ticks_in_degrees = 145.1/360;

    public PidController(Telemetry telemetry, Gamepad gamepad2) {
        this.telemetry = telemetry;
        this.gamepad2 = gamepad2;
    }


    public void init(HardwareMap hardwareMap) {
        controllerRight =  new PIDController(pR,iR,dR);
        controllerLeft =  new PIDController(pL,iL,dL);
        robot.init(hardwareMap);
    }


    public void run() {
      slidePID();
      telemetry.update();
    }

    private void slidePID() {
        slideLeftPID();
        slideRightPID();
    }

    private void slideRightPID() {
        controllerRight.setPID(pR,iR,dR);
        int slideRightPos = robot.slideRight.getCurrentPosition();
        double pidRight = controllerRight.calculate(slideRightPos, -target);
        double ffRight = Math.cos(Math.toRadians(target/ticks_in_degrees)) * fR;

        double powerRight = pidRight + ffRight;
        if (gamepad2.left_stick_y != 0) {
            robot.slideRight.setPower(-gamepad2.left_stick_y);
            target = slideRightPos;
        } else {
            robot.slideRight.setPower(-powerRight * .975);
        }

        telemetry.addData("posRight ", slideRightPos);
        telemetry.addData("target ", target);
    }

    private void slideLeftPID() {
        controllerLeft.setPID(pL,iL,dL);
        int slideLeftPos = robot.slideLeft.getCurrentPosition();
        double pidLeft = controllerLeft.calculate(slideLeftPos, target);
        double ffLeft = Math.cos(Math.toRadians(target/ticks_in_degrees)) * fL;

        double powerLeft = pidLeft + ffLeft;

        if (gamepad2.left_stick_y != 0) {
            robot.slideRight.setPower(-gamepad2.left_stick_y);
        } else {
            robot.slideRight.setPower(powerLeft);
        }

        telemetry.addData("posLeft ", slideLeftPos);
    }
}

