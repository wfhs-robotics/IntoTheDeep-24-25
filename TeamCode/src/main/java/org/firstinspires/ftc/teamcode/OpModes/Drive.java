package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.Misc.PidController;

@TeleOp(name = "Drive", group = "Robot")
public class Drive extends LinearOpMode {
    PidController pid;
    @Override
    public void runOpMode() {
        waitForStart();
        if (opModeIsActive()) {
            // Telemetry that also goes on Dashboard
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            pid = new PidController(telemetry, gamepad2);
            pid.init(hardwareMap);


            while (opModeIsActive()) {
                pid.run();
            }
        }
    }
}
