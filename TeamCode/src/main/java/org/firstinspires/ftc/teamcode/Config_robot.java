package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Config_robot {
    private BNO055IMU imu = null;      // Control/Expansion Hub IMU
    public DcMotorEx leftFrontDrive = null;
    public DcMotorEx leftBackDrive = null;
    public DcMotorEx rightFrontDrive = null;
    public DcMotorEx rightBackDrive = null;
    public DcMotorEx rightLift = null;
    public DcMotorEx leftLift = null;
    public DcMotorEx zahvat = null;
    public Servo servoP = null;
    public Servo servoS = null;

    HardwareMap hardM = null;

    public Config_robot(){

    }

    public void init_Tele(HardwareMap hMap){
        hardM = hMap;

        servoP = hMap.get(Servo.class,"servoP");
        servoS = hMap.get(Servo.class, "servoS");

        leftFrontDrive  = hardM.get(DcMotorEx.class, "leftFront");
        leftBackDrive  = hardM.get(DcMotorEx.class, "leftRear");
        rightFrontDrive = hardM.get(DcMotorEx.class, "rightFront");
        rightBackDrive = hardM.get(DcMotorEx.class, "rightRear");

        rightLift = hardM.get(DcMotorEx.class, "rightL");
//        leftLift = hardM.get(DcMotorEx.class, "leftL");
        zahvat = hardM.get(DcMotorEx.class, "zahvat");

        rightFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorEx.Direction.REVERSE);

        zahvat.setDirection(DcMotorSimple.Direction.FORWARD);

        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void init_Auto(HardwareMap hMap){

        hardM = hMap;
        servoP = hMap.get(Servo.class,"servo");

        leftFrontDrive  = hardM.get(DcMotorEx.class, "leftFront");
        leftBackDrive  = hardM.get(DcMotorEx.class, "leftRear");
        rightFrontDrive = hardM.get(DcMotorEx.class, "rightFront");
        rightBackDrive = hardM.get(DcMotorEx.class, "rightRear");

        rightLift = hardM.get(DcMotorEx.class, "rightL");
        leftLift = hardM.get(DcMotorEx.class, "leftL");
        zahvat = hardM.get(DcMotorEx.class, "zahvat");

        rightFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorEx.Direction.REVERSE);

        zahvat.setDirection(DcMotorSimple.Direction.FORWARD);

        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
