package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
public class AutoPark extends LinearOpMode {
    Config_robot robot = new Config_robot();
    public void runOpMode() {
        robot.init_Auto(hardwareMap);
        waitForStart();
        robot.leftBackDrive.setPower(0.3);
        robot.rightBackDrive.setPower(0.3);
        robot.rightFrontDrive.setPower(0.3);
        robot.leftFrontDrive.setPower(0.3);
        sleep(400);
        robot.zahvat.setPower(1);
        sleep(1030);
        robot.leftBackDrive.setPower(0);
        robot.rightBackDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.leftFrontDrive.setPower(0);
        sleep(7000);
        robot.zahvat.setPower(0);
    }

}
