package org.firstinspires.ftc.teamcode.Misc;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Hardware {
//     Define Motor and Servo objects
    public DcMotorEx leftFront;
    public DcMotorEx rightFront;
    public DcMotorEx rightBack;
    public DcMotorEx leftBack;
    public DcMotorEx arm;
    public DcMotorEx slide;
    public Servo armClaw;
    public Servo wrist;
    public Servo slideClaw;

    private HardwareMap hwMap;

    public Hardware() {

    }

    public void init(HardwareMap aHwMap)    {
       // HardwareMap is null until the OpMode is run, so it needs to be passed into the method
        hwMap = aHwMap;

        leftFront = hwMap.get(DcMotorEx.class, "leftFront");
        leftBack = hwMap.get(DcMotorEx.class, "leftBack");
        rightFront = hwMap.get(DcMotorEx.class, "rightFront");
        rightBack = hwMap.get(DcMotorEx.class, "rightBack");
        slide = hwMap.get(DcMotorEx.class, "slide");
        arm = hwMap.get(DcMotorEx.class, "arm");
        armClaw = hwMap.get(Servo.class, "armClaw");
        wrist = hwMap.get(Servo.class, "wrist");
        slideClaw = hwMap.get(Servo.class, "slideClaw");


        // RUN_WITHOUT_ENCODER for PID looped motors
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
