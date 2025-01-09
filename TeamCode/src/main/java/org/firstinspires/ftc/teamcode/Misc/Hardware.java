package org.firstinspires.ftc.teamcode.Misc;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
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
    public DcMotorEx slideArm;
    public Servo wrist;
    public Servo slideClawLeft;
    public Servo slideClawRight;
    public CRServo intake;

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
        slideArm = hwMap.get(DcMotorEx.class, "slideArm");
        arm = hwMap.get(DcMotorEx.class, "arm");
        wrist = hwMap.get(Servo.class, "wrist");
        slideClawRight = hwMap.get(Servo.class, "slideClawRight");
        slideClawLeft = hwMap.get(Servo.class, "slideClawLeft");
        intake = hwMap.get(CRServo.class, "intake");


        // Reset encoder values for the PID looped motors so that they start at 0
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // RUN_WITHOUT_ENCODER for PID looped motors
        slide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }



}
