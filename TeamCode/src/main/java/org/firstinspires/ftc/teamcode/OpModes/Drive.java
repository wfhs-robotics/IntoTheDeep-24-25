package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Misc.Hardware;
import org.firstinspires.ftc.teamcode.Misc.PoseStorage;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Drive")
public class Drive extends OpMode {
    Hardware robot  = new Hardware();
    Pose2d pose;
    MecanumDrive drive;
    private List<Action> runningActions = new ArrayList<>();
    private FtcDashboard dash = FtcDashboard.getInstance();
    boolean prevA = false;
    boolean prevY = false;
    int slideTarget = 0; // Default to 0




    // Runs ONCE when a person hits INIT
    @Override
    public void init() {
        pose = PoseStorage.currentPose;
        robot.init(hardwareMap); //Init Hardware, /Misc/Hardware
        drive = new MecanumDrive(hardwareMap, pose); // Create our driving class, giving it our current pose after autonomous
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // Telemetry that allows for dashboard and phone

    }
    @Override // Runs ONCE when a person hits START
    public void start() {
        // Init servos
        robot.wrist.setPosition(.3);
        robot.clawRight.setPosition(1);
        robot.clawLeft.setPosition(0);
    };

    // LOOPS while the until the player hits STOP
    @Override
    public void loop() {
        pose = drive.pose;

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

        pickupLogic();

        drive.updatePoseEstimate();
    }

    public void pickupLogic() {
        // Manual or automatic arm control
        if (gamepad2.left_stick_y != 0) {
            // Manual arm movement
            robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.arm.setPower(-gamepad2.left_stick_y);
        } else {
            // Automatic arm control to hold position
            robot.arm.setTargetPosition(robot.arm.getCurrentPosition());
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.arm.setVelocity(2100);
        }



        // Slide Positions
        if (gamepad2.dpad_up) slideTarget = 3500;
        else if (gamepad2.dpad_right) slideTarget = 1500;
        else if (gamepad2.dpad_down) slideTarget = 0;

        if (gamepad2.dpad_up || gamepad2.dpad_right || gamepad2.dpad_down) {
            robot.slide.setTargetPosition(-slideTarget);
            robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.slide.setVelocity(2100);
        }

        // Slide Arm Control with toggling positions
        if(gamepad2.y && gamepad2.y != prevY) {
            if (robot.slideArm.getTargetPosition() == 2300) {
                robot.slideArm.setTargetPosition(0);
                robot.slideArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.slideArm.setVelocity(2100);
            } else {
                robot.slideArm.setTargetPosition(2300);
                robot.slideArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.slideArm.setVelocity(2100);
            }
        }
        if(gamepad2.a && gamepad2.a != prevA) {
            if (robot.slideArm.getTargetPosition() == 1000) {
                robot.slideArm.setTargetPosition(0);
                robot.slideArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.slideArm.setVelocity(2100);
            } else {
                robot.slideArm.setTargetPosition(1000);
                robot.slideArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.slideArm.setVelocity(2100);
            }
        }

        // Claw Control
        if(gamepad2.right_bumper) {
            robot.clawRight.setPosition(1);
            robot.clawLeft.setPosition(0);
        }
        if(gamepad2.left_bumper) {
            robot.clawRight.setPosition(0);
            robot.clawLeft.setPosition(1);
        }

        if(gamepad2.right_trigger != 0) {
            robot.intake.setPower(gamepad2.right_trigger);
        }

        if(gamepad2.left_trigger != 0) {
            robot.intake.setPower(-gamepad2.left_trigger);
        }

        if(gamepad2.left_trigger == 0 && gamepad2.right_trigger ==0 ) {
            robot.intake.setPower(0);
        }

        prevY = gamepad2.y;
        prevA = gamepad2.a;
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
