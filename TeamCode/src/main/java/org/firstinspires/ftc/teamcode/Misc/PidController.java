package org.firstinspires.ftc.teamcode.Misc;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.analysis.function.Abs;


@Config
@TeleOp
public class PidController extends OpMode {

    PIDController controller;
    Hardware robot = new Hardware();
    public static double p = 0, i= 0, d= 0, f= 0;
    public static int target= 0;

    public static double ticks_in_degrees = 145.1/360;


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        controller =  new PIDController(p,i,d);
        robot.init(hardwareMap);
    }


    @Override
    public void loop() {
        controller.setPID(p,i,d);
        int actuatorPos = robot.linearActuator.getCurrentPosition();
        double pid = controller.calculate(actuatorPos, target);
        double ff = Math.cos(Math.toRadians(target/ticks_in_degrees)) * f;

        double power = pid + ff;

        robot.linearActuator.setPower(power);

        telemetry.addData("pos ", actuatorPos);
        telemetry.addData("target ", target);
    }
}

