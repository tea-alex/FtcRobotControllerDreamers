package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Config_robot {
    public DcMotorEx leftFrontDrive = null;
    public DcMotorEx leftBackDrive = null;
    public DcMotorEx rightFrontDrive = null;
    public DcMotorEx rightBackDrive = null;

    HardwareMap hardM = null;

    public Config_robot(){

    }

    public void init_Tele(HardwareMap hMap){
        hardM = hMap;
        leftFrontDrive  = hardM.get(DcMotorEx.class, "leftFront");
        leftBackDrive  = hardM.get(DcMotorEx.class, "leftRear");
        rightFrontDrive = hardM.get(DcMotorEx.class, "rightFront");
        rightBackDrive = hardM.get(DcMotorEx.class, "rightRear");

        rightFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorEx.Direction.REVERSE);

        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void init_Auto(HardwareMap hMap){
        hardM = hMap;
    }
}
