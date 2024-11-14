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


    private HardwareMap hwMap;

    public Hardware() {

    }

    public void init(HardwareMap aHwMap)    {

       // HardwareMap is null until the OpMode is run, so it needs to be passed into the method
        hwMap = aHwMap;

//        leftFront = hwMap.get(DcMotorEx.class, "leftFront");
//        leftBack = hwMap.get(DcMotorEx.class, "leftBack");
//        rightFront = hwMap.get(DcMotorEx.class, "rightFront");
//        rightBack = hwMap.get(DcMotorEx.class, "rightBack");
//        linearActuator = hwMap.get(DcMotorEx.class, "linearActuator");
        slideRight = hwMap.get(DcMotorEx.class, "slideRight");
        slideLeft = hwMap.get(DcMotorEx.class, "slideLeft");

       // TODO: Set Direction of Motors

        // RUN_WITHOUT_ENCODER for PID looped motors
        slideRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        linearActuator.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

    }
}
