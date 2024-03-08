package org.firstinspires.ftc.teamcode.VersionItaliano;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class AutoPark extends LinearOpMode {
    Config_robot robot = new Config_robot();

    static final double     COUNTS_PER_MOTOR_REV    = 223.0;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;
    static final double     WHEEL_DIAMETER_INCHES   = 3.93701 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);


    private ElapsedTime runtime = new ElapsedTime();
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

    void servo_close(){
        robot.servoP.setPosition(0);
    }

    void servo_open(){
        robot.servoP.setPosition(1);
    }



    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;

        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFrontDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            newLeftBackTarget = robot.leftFrontDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightFrontDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            robot.leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            robot.rightFrontDrive.setTargetPosition(newRightFrontTarget);

            robot.leftBackDrive.setTargetPosition(newLeftBackTarget);
            robot.rightBackDrive.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            robot.rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            robot.leftBackDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            robot.rightBackDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftFrontDrive.setPower(Math.abs(speed));
            robot.rightFrontDrive.setPower(Math.abs(speed));

            robot.leftBackDrive.setPower(Math.abs(speed));
            robot.rightBackDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFrontDrive.isBusy() && robot.rightFrontDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        robot.leftFrontDrive.getCurrentPosition(), robot.rightFrontDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftFrontDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);

            robot.leftBackDrive.setPower(0);
            robot.rightBackDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            robot.rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            robot.leftBackDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            robot.rightBackDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

}
