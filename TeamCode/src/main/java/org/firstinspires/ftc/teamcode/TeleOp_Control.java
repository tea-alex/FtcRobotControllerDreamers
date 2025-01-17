package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp")
@Config

public class TeleOp_Control extends OpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Config_robot robot = new Config_robot();
    public OnServo launcher = new OnServo();
    public static int up_pos = 800;
    public static int middle_pos = 400;
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

        robot.zahvat.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

        if(gamepad1.y){
            hight_position();
        }

        if(gamepad1.b){
            midle_position();
        }

        if(gamepad1.a){
            low_position();
        }

        if(gamepad1.x){
            zero_position();
        }

        if(gamepad1.dpad_left){
            launcher.launch = true;
        }

        telemetry.addData("Position rightL: ", robot.rightLift.getCurrentPosition());
//        telemetry.addData("Position up: ", robot.leftLift.getCurrentPosition());
        telemetry.update();

    }

    public void stop(){}

//    void left_lift_regulate(){
//        int pos_down = robot.leftLift.getCurrentPosition() + 10;
//        robot.leftLift.setVelocity(500);
 //       robot.leftLift.setTargetPosition(pos_down);
//    }

    void right_lift_regulate(){
        int pos_up = robot.rightLift.getCurrentPosition() + 10;
        robot.rightLift.setVelocity(500);
        robot.rightLift.setTargetPosition(pos_up);
    }

    void hight_position(){
        robot.rightLift.setVelocity(700);
        robot.rightLift.setTargetPosition(up_pos);
    }

    void midle_position(){
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

    class OnServo extends  Thread{
        boolean launch = false;
        public void run(){
            while (!isInterrupted()){
                if(launch){
                    if (gamepad1.right_bumper) {
                        robot.servoP.setPosition(0.5);
                        telemetry.addData(">", "close");
                        telemetry.update();
                    }
                    if (gamepad1.left_bumper){
                        robot.servoP.setPosition(1);
                        telemetry.addData(">", "open");
                        telemetry.update();
                    }
                    if(gamepad1.dpad_right){
                        robot.servoS.setPosition(0.5);
                    }
                }
            }
        }
}
}
