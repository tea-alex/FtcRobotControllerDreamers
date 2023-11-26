package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOp")
@Config

public class TeleOp_Control extends OpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Config_robot robot = new Config_robot();
//    public OnServo launcher = new OnServo();
    public static int up_pos = 3200;
    public static int middle_pos = 1600;
    public static int low_pos = 200;
    public static int zero_pos = 10;

    public void init(){
        robot.init_Tele(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        robot.servoP.setPosition(0.5);
        robot.servoS.setPosition(0);
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

        double leftFrontPower  = axial + lateral + yaw; //axial
        double rightFrontPower = axial - lateral - yaw; // - lateral
        double leftBackPower   = axial - lateral + yaw; // - lateral
        double rightBackPower  = axial + lateral - yaw; // axial


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

        robot.zahvat.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
        robot.conv.setPower(-gamepad1.right_trigger);

        if(gamepad1.dpad_down) {
            robot.rightLift.setTargetPosition(robot.rightLift.getTargetPosition() - 30);
            robot.leftLift.setTargetPosition(robot.leftLift.getTargetPosition() - 30);
        }
        if (gamepad1.dpad_up) {
            robot.rightLift.setTargetPosition(robot.rightLift.getTargetPosition() + 30);
            robot.leftLift.setTargetPosition(robot.leftLift.getTargetPosition() + 30);
        }
/*
        if(gamepad1.y){
            high_position();
        }

        if(gamepad1.b){
            mid_position();
        }

        if(gamepad1.a){
            low_position();
        }

        if(gamepad1.x){
            zero_position();
        }

 */

//        if(gamepad1.dpad_left){
//            launcher.launch = true;
//        }
        if (gamepad1.right_bumper) {
            robot.servoP.setPosition(1);
        }
        if (gamepad1.left_bumper) {
            robot.servoP.setPosition(0);
        }
        if (gamepad1.right_stick_button) {
            robot.servoS.setPosition(1);
        }
        if (gamepad1.left_stick_button) {
            robot.rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        telemetry.addData("Position rightL: ", robot.rightLift.getCurrentPosition());
        telemetry.addData("Target rightL:",robot.rightLift.getTargetPosition());
        telemetry.addData("Position leftL: ", robot.leftLift.getCurrentPosition());
        telemetry.addData("Target leftL:",robot.leftLift.getTargetPosition());
//        telemetry.addData("Position up: ", robot.leftLift.getCurrentPosition());
        telemetry.update();

    }

    public void stop(){
        robot.rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    void left_lift_regulate(){
        int pos_down = robot.leftLift.getCurrentPosition() + 10;
        robot.leftLift.setVelocity(500);
        robot.leftLift.setTargetPosition(pos_down);
    }

    void right_lift_regulate(){
        int pos_up = robot.rightLift.getCurrentPosition() + 10;
        robot.rightLift.setVelocity(500);
        robot.rightLift.setTargetPosition(pos_up);
    }

    void high_position(){
        robot.rightLift.setVelocity(700);
        robot.rightLift.setTargetPosition(up_pos);
    }

    void mid_position(){
        robot.rightLift.setVelocity(700);
        robot.rightLift.setTargetPosition(middle_pos);
    }

    void low_position(){
        robot.rightLift.setVelocity(700);
        robot.rightLift.setTargetPosition(low_pos);
    }

    void zero_position(){
        robot.rightLift.setVelocity(700);
        robot.rightLift.setTargetPosition(zero_pos);
    }

//    class OnServo extends  Thread{
//        boolean launch = false;
//        public void run(){
//            while (!isInterrupted()){
//                if(launch){
//                    if (gamepad1.right_bumper) {
//                        robot.servoP.setPosition(0.5);
//                        telemetry.addData(">", "close");
//                        telemetry.update();
//                    }
//                    if (gamepad1.left_bumper){
//                        robot.servoP.setPosition(1);
//                        telemetry.addData(">", "open");
//                        telemetry.update();
//                    }
//                    if(gamepad1.dpad_right){
//                        robot.servoS.setPosition(0.5);
//                    }
//                }
//            }
//        }
//}
}
