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

    PIDController controllerRight;
    PIDController controllerLeft;
    Gamepad gamepad2;
    Telemetry telemetry;
    Hardware robot = new Hardware();
    public static double pR = 0.022, iR= 0, dR= 0.0001, fR= 0.05;
    public static double pL = 0.022, iL= 0, dL= 0.0001, fL= 0.05;
    public static int target = 0;


    public static double ticks_in_degrees = 145.1/360;
    
    @Override
    public void init() {
        controllerRight =  new PIDController(pR,iR,dR);
        controllerLeft =  new PIDController(pL,iL,dL);
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
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
        robot.slideRight.setPower(powerRight);

        telemetry.addData("posRight ", slideRightPos);
        telemetry.addData("target ", target);
    }

    private void slideLeftPID() {
        controllerLeft.setPID(pL,iL,dL);
        int slideLeftPos = robot.slideLeft.getCurrentPosition();
        double pidLeft = controllerLeft.calculate(slideLeftPos, target);
        double ffLeft = Math.cos(Math.toRadians(target/ticks_in_degrees)) * fL;

        double powerLeft = pidLeft + ffLeft;
        robot.slideLeft.setPower(powerLeft);

        telemetry.addData("posLeft ", slideLeftPos);
        telemetry.addData("target ", target);
    }


}

