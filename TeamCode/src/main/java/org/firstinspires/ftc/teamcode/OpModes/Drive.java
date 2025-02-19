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
import org.firstinspires.ftc.teamcode.Misc.ArmPosStorage;
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
    boolean prevStick = false;
    boolean prevX = false;
    boolean wristToggled = false;
    boolean horizontal = false;
    boolean armSlow = false;
    int slideTarget = 0; // Default to 0
    private int lastPressedYPosition = 0;
    private int lastPressedAPosition = 0;
    private boolean isMovingToPosition = false;



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
        robot.wrist.setPosition(.65);
        robot.slideClawRight.setPosition(1);
        robot.slideClawLeft.setPosition(0);
        robot.armClawRight.setPosition(0);
        robot.armClawLeft.setPosition(1);
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
        ArmPosStorage.armPos = robot.arm.getCurrentPosition();
    }

    public void pickupLogic() {
        // Arm
        if (gamepad2.left_stick_button && !prevStick) armSlow =  !armSlow;

        if (gamepad2.left_stick_y != 0) {
            boolean wasHorizontal = horizontal;
            horizontal = robot.arm.getCurrentPosition() > 3000;

            if (wasHorizontal && !horizontal && robot.slideArm.getCurrentPosition() > 2750) {
                robot.slideArm.setTargetPosition(2400);
                robot.slideArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.slideArm.setVelocity(2100);
            }

            if(armSlow) {
                if(gamepad2.left_stick_y < 0) {
                    robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.arm.setPower(-gamepad2.left_stick_y * .2);
                } else {
                    robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.arm.setPower(-gamepad2.left_stick_y * .4);
                }

            } else {
                robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.arm.setPower(-gamepad2.left_stick_y);
            }
        } else {
            // Automatic arm control to hold position
            robot.arm.setTargetPosition(robot.arm.getCurrentPosition());
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.arm.setVelocity(3000);
        }


        if(gamepad2.y && gamepad2.y != prevY) {
            if (Math.abs(robot.slideArm.getCurrentPosition() - 2400) < 50 && lastPressedYPosition == 2400) {
                // If we're close to 2050 and that was our last Y press target, go to 0
                robot.slideArm.setTargetPosition(0);
                lastPressedYPosition = 0;
            } else {
                // Otherwise, go to 2400
                robot.slideArm.setTargetPosition(2400);
                lastPressedYPosition = 2400;
            }
            isMovingToPosition = true;
            robot.slideArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.slideArm.setVelocity(2100);
        }
        else if(gamepad2.a && gamepad2.a != prevA) {
            if (Math.abs(robot.slideArm.getCurrentPosition() - 1000) < 50 && lastPressedAPosition == 1000) {
                // If we're close to 1000 and that was our last A press target, go to 0
                robot.slideArm.setTargetPosition(0);
                lastPressedAPosition = 0;
            } else {
                // Otherwise, go to 1000
                robot.slideArm.setTargetPosition(1000);
                lastPressedAPosition = 1000;
            }
            isMovingToPosition = true;
            robot.slideArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.slideArm.setVelocity(2100);
        }

            else if (gamepad2.right_stick_y != 0) {
            horizontal = robot.arm.getCurrentPosition() > 3000;
            if (horizontal) {
                robot.slideArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.slideArm.setPower(-gamepad2.right_stick_y);
            } else {
                if (robot.slideArm.getCurrentPosition() <= 2450) {
                    robot.slideArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.slideArm.setPower(-gamepad2.right_stick_y);
                } else {
                    robot.slideArm.setTargetPosition(2400);
                    robot.slideArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.slideArm.setVelocity(2100);
                }
            }
        }
        else {
            // If we're moving to a position, check if we've reached it
            if (isMovingToPosition) {
                int currentPos = robot.slideArm.getCurrentPosition();
                int targetPos = robot.slideArm.getTargetPosition();
                if (Math.abs(currentPos - targetPos) < 50) {  // Within 50 encoder ticks
                    isMovingToPosition = false;
                }
            }

            // Hold position
            if (!isMovingToPosition) {
                robot.slideArm.setTargetPosition(robot.slideArm.getCurrentPosition());
                robot.slideArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.slideArm.setVelocity(3000);
            }
        }




        // Slide Positions
        if (gamepad2.dpad_up) slideTarget = 1800;
        else if (gamepad2.dpad_right) slideTarget = 300;
        else if (gamepad2.dpad_left) slideTarget = slideTarget - 100;
        else if (gamepad2.dpad_down) slideTarget = 0;

        if (gamepad2.dpad_up || gamepad2.dpad_right || gamepad2.dpad_down || gamepad2.dpad_left) {
            robot.slide.setTargetPosition(-slideTarget);
            robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.slide.setVelocity(2100);
        }

        // Claw Control
        if(gamepad2.right_bumper) {
            robot.slideClawRight.setPosition(1);
            robot.slideClawLeft.setPosition(0);
            robot.armClawRight.setPosition(0);
            robot.armClawLeft.setPosition(1);
        }
        if(gamepad2.left_bumper) {
            robot.slideClawRight.setPosition(0);
            robot.slideClawLeft.setPosition(1);
            robot.armClawRight.setPosition(1);
            robot.armClawLeft.setPosition(0);
        }

        if (gamepad2.x && !prevX)  {
            wristToggled = !wristToggled;
            robot.wrist.setPosition(wristToggled ? 1 : .65);
            prevX = gamepad2.x;
        }

        if(gamepad2.left_trigger == 0 && gamepad2.right_trigger ==0 ) {
            robot.intake.setPower(0);
        }

        prevY = gamepad2.y;
        prevA = gamepad2.a;
        prevX = gamepad2.x;
        prevStick = gamepad2.left_stick_button;
        telemetry.addData("armPos", robot.arm.getCurrentPosition());
        telemetry.addData("slideArmPos", robot.slideArm.getCurrentPosition());
        telemetry.update();
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

