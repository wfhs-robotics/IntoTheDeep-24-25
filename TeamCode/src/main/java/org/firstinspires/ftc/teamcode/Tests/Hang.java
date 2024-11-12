package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Hang", group="Test")
public class Hang extends LinearOpMode {
    DcMotor slideLeft;
    DcMotor slideRight;
    DcMotor linearActuator;
    @Override
    public void runOpMode() throws InterruptedException {
//       slideLeft = hardwareMap.get(DcMotor.class, "slideLeft");
//       slideRight = hardwareMap.get(DcMotor.class, "slideRight");
       linearActuator = hardwareMap.get(DcMotor.class, "linearActuator");

       waitForStart();
        while (opModeIsActive()) {
//            if (gamepad1.left_stick_y != 0) {
//                slideLeft.setPower(-gamepad1.left_stick_y);
//                slideRight.setPower(gamepad1.left_stick_y *.975);
//            } else {
//                slideLeft.setPower(0);
//                slideRight.setPower(0);
//            }

//            if (gamepad1.right_stick_y != 0) {
                linearActuator.setPower(Range.clip(-gamepad1.right_stick_y, -0.5, 0.5));
//            } else {
//                linearActuator.setPower(0);
//            }



        }
    }
}
