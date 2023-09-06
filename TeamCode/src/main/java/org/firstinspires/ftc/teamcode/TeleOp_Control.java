package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "TeleOp")
@Config

public class TeleOp_Control extends OpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Config_robot robot = new Config_robot();
    public void init(){
        robot.init_Tele(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    public void start(){
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    public void loop(){
        double max;

        double axial   = -gamepad1.left_stick_y; // вперед
        double lateral =  gamepad1.left_stick_x; // вправо влево
        double yaw     =  gamepad1.right_stick_x; // поворот

        double leftFrontPower  = lateral + yaw; //axial
        double rightFrontPower = axial - yaw; // - lateral
        double leftBackPower   = axial + yaw; // - lateral
        double rightBackPower  = lateral - yaw; // axial


        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }
        robot.leftFrontDrive.setPower(leftFrontPower);
        robot.rightFrontDrive.setPower(rightFrontPower);
        robot.leftBackDrive.setPower(leftBackPower);
        robot.rightBackDrive.setPower(rightBackPower);
    }

    public void stop(){}
}
