package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Misc.Actions;
import org.firstinspires.ftc.teamcode.Misc.Hardware;
import org.firstinspires.ftc.teamcode.Misc.PidController;
import org.firstinspires.ftc.teamcode.Misc.PoseStorage;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.ThreeDeadWheelLocalizer;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Drive")
public class Drive extends OpMode {
    PidController pid;
    Hardware robot  = new Hardware();
    Pose2d pose;
    MecanumDrive drive;
    private List<Action> runningActions = new ArrayList<>();
    private FtcDashboard dash = FtcDashboard.getInstance();
    Actions actions;
    boolean prevA = false;
    boolean prevY = false;
    boolean prevStart = false;

    // Runs ONCE when a person hits INIT
    @Override
    public void init() {
        pose = PoseStorage.currentPose;

        robot.init(hardwareMap); //Init Hardware, /Misc/Hardware
        pid = new PidController(telemetry, gamepad2);
        actions = new Actions(pid, hardwareMap);
        drive = new MecanumDrive(hardwareMap, pose); // Create our driving class, giving it our current pose after autonomous
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // Telemetry that allows for dashboard and phone

        // Start the PID
        pid.init(hardwareMap, robot);
    }
    @Override // Runs ONCE when a person hits START
    public void start() {
        // Init servos
        robot.wrist.setPosition(0);
        robot.slideClawRight.setPosition(0);
        robot.slideClawLeft.setPosition(0);
    };

    // LOOPS while the until the player hits STOP
    @Override
    public void loop() {
        pose = drive.pose;
        List<Action> newActions = new ArrayList<>();
        actionLogic(telemetry, newActions);

        if(gamepad1.a && gamepad1.a != prevA) // Precision Mode
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(-gamepad1.left_stick_y * .5, -gamepad1.left_stick_x * .5),
                -gamepad1.right_stick_x * .5
        ));
        else // Normal Driving
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(-gamepad1.left_stick_y * .9, -gamepad1.left_stick_x * .9),
                    -gamepad1.right_stick_x * .9
            ));

        prevA = gamepad2.a;
        prevY = gamepad2.y;
        prevStart = gamepad2.start;

        pid.run(true, prevStart);

        pickupLogic();

        drive.updatePoseEstimate();
    }

    public void pickupLogic() {
        if(gamepad2.right_bumper) {
            robot.slideClawRight.setPosition(1);
            robot.slideClawLeft.setPosition(0);
        }
        if(gamepad2.left_bumper) {
            robot.slideClawRight.setPosition(0);
            robot.slideClawLeft.setPosition(1);
        }

        if(gamepad2.right_trigger != 0) {
            robot.intake.setPower(gamepad2.right_trigger);
        }

        if(gamepad2.left_trigger != 0) {
            robot.intake.setPower(-gamepad2.left_trigger);
        }



        if(gamepad2.y && gamepad2.y != prevY) {
            runningActions.add(new SequentialAction(
                    actions.stackHighBucket()
            ));
        }

        if(gamepad2.a && gamepad2.a != prevA) {
            runningActions.add(new SequentialAction(
                    actions.stackLowBucket()
            ));
        }
    }
    public void actionLogic(Telemetry telemetry, List<Action> newActions) {
        TelemetryPacket packet = new TelemetryPacket();

        // updated based on gamepads

        // update running actions

        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        dash.sendTelemetryPacket(packet);
    }
}
