package org.firstinspires.ftc.teamcode.Misc;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Hardware {

    // Define Motor and Servo objects
    public DcMotorEx leftFront;
    public DcMotorEx rightFront;
    public DcMotorEx rightBack;
    public DcMotorEx leftBack;
    public DcMotorEx linearActuator;
    public DcMotorEx slideLeft;
    public DcMotorEx slideRight;
    public Servo armLeft;
    public Servo armRight;
    public Servo arm2Right;
    public Servo arm2Left;
    public Servo claw;
    public Servo rotate;


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
        linearActuator = hwMap.get(DcMotorEx.class, "linearActuator");
        slideRight = hwMap.get(DcMotorEx.class, "slideRight");
        slideLeft = hwMap.get(DcMotorEx.class, "slideLeft");
//        armLeft = hwMap.get(Servo.class, "armLeft");
//        armRight = hwMap.get(Servo.class, "armRight");
//        arm2Left = hwMap.get(Servo.class, "arm2Left");
//        arm2Right = hwMap.get(Servo.class, "arm2Right");
//        claw = hwMap.get(Servo.class, "claw");
//        rotate = hwMap.get(Servo.class, "rotate");

        // TODO: Set Direction of Motors

        // RUN_WITHOUT_ENCODER for PID looped motors
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        slideRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        slideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////        linearActuator.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

    }
}
