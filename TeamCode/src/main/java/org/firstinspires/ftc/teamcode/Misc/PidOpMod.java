package org.firstinspires.ftc.teamcode.Misc;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@TeleOp
public class PidOpMod extends OpMode {
    PIDController controllerRight;
    PIDController controllerLeft;
    Hardware robot = new Hardware();
    public static double pR = 0.022, iR= 0, dR= 0.0001, fR= 0.05;
    public static double pL = 0.022, iL= 0, dL= 0.0001, fL= 0.05;
    public static int target = 0;
    public static double ticks_in_degrees = 145.1/360;

    @Override
    public void init() {
        robot.init(hardwareMap);
//        controllerRight =  new PIDController(pR,iR,dR);
//        controllerLeft =  new PIDController(pL,iL,dL);
    }

    @Override
    public void loop() {
        telemetry.addData("Gamepad 2 Y", -gamepad2.left_stick_y);
//        controllerRight.setPID(pR,iR,dR);
//        int slideRightPos = robot.slideRight.getCurrentPosition();
//        double pidRight = controllerRight.calculate(slideRightPos, -target);
//        double ffRight = Math.cos(Math.toRadians(-target/ticks_in_degrees)) * fR;
//
//        double powerRight = pidRight + ffRight;
//
//        controllerLeft.setPID(pL,iL,dL);
//        int slideLeftPos = robot.slideLeft.getCurrentPosition();
//        double pidLeft = controllerLeft.calculate(slideLeftPos, target);
//        double ffLeft = Math.cos(Math.toRadians(target/ticks_in_degrees)) * fL;
//
//        double powerLeft = pidLeft + ffLeft;
//
//        if (gamepad2.left_stick_y != 0) {
//            robot.slideRight.setPower(gamepad2.left_stick_y);
//            robot.slideLeft.setPower(-gamepad2.left_stick_y);
//            target = slideRightPos;
//        } else {
//            robot.slideRight.setPower(powerRight * .975);
//            robot.slideLeft.setPower(powerLeft);
//        }

//        robot.slideRight.setPower(gamepad2.left_stick_y * .975);
        robot.slideLeft.setPower(-gamepad2.left_stick_y);

//        telemetry.addData("posRight ", slideRightPos);
//        telemetry.addData("posLeft ", slideLeftPos);
//        telemetry.addData("target ", target);

        telemetry.update();
    }
}
