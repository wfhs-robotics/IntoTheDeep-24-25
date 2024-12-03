package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Misc.Hardware;
import org.firstinspires.ftc.teamcode.Misc.PidController;
import org.firstinspires.ftc.teamcode.Misc.PoseStorage;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.ThreeDeadWheelLocalizer;

@TeleOp(name = "Drive")
public class Drive extends OpMode {
    PidController pid;
    Hardware robot  = new Hardware();
    Pose2d pose;
    MecanumDrive drive;
    ThreeDeadWheelLocalizer localizer;
    public static double arm1PosOpen = 0;
    public static double arm2PosOpen = 0;
    public static double arm1PosClosed = 0;
    public static double arm2PosClosed = 0;
    public static double clawOpen = 0;
    public static double clawClosed = 0;
    public static double clawRotate;

    // Runs ONCE when a person hits INIT
    @Override
    public void init() {
        pose = PoseStorage.currentPose;
        robot.init(hardwareMap); //Init Hardware, /Misc/Hardware

        drive = new MecanumDrive(hardwareMap, pose); // Create our driving class, giving it our current pose after autonomous
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // Telemetry that allows for dashboard and phone

        // Start the PID
        pid = new PidController(telemetry, gamepad2);
        pid.init(hardwareMap, robot);
    }
    @Override // Runs ONCE when a person hits START
    public void start() {
        // Init servos
    };

    // LOOPS while the until the player hits STOP
    @Override
    public void loop() {
        pose = drive.pose;


        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(-gamepad1.left_stick_y * .9, -gamepad1.left_stick_x * .9),
                -gamepad1.right_stick_x * .9
        ));


        pid.run();
//        pickupLogic();


        robot.linearActuator.setPower(Range.clip(-gamepad1.right_stick_y, -0.8, 0.8));

        drive.updatePoseEstimate();
    }

    public void pickupLogic() {
//        if(gamepad2.y) { // Open Arm
//            robot.armLeft.setPosition(arm1PosOpen);
//            robot.armRight.setPosition(arm1PosOpen);
//            robot.arm2Right.setPosition(arm2PosOpen);
//            robot.arm2Left.setPosition(arm2PosOpen);
//        }
//        if(gamepad2.a) { // Close Arm
//            robot.armLeft.setPosition(arm1PosClosed);
//            robot.armRight.setPosition(arm1PosClosed);
//            robot.arm2Left.setPosition(arm2PosClosed);
//            robot.arm2Right.setPosition(arm2PosClosed);
//        }
//        if(gamepad2.b) {
//            robot.claw.setPosition(clawOpen);
//        }
//        if(gamepad2.x) {
//            robot.claw.setPosition(clawClosed);
//        }
    }
}
