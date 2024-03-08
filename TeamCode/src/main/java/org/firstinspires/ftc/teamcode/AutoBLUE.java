package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


public class AutoBLUE extends LinearOpMode {
    Config_robot robot = new Config_robot();
    private OpenCvCamera webcam;

    private static final int CAMERA_WIDTH  = 1280;
    private static final int CAMERA_HEIGHT = 720;

    private double CrLowerUpdate = 160;
    private double CbLowerUpdate = 100;
    private double CrUpperUpdate = 255;
    private double CbUpperUpdate = 255;

    public static double borderLeftX    = 0.0;   //fraction of pixels from the left side of the cam to skip
    public static double borderRightX   = 0.0;   //fraction of pixels from the right of the cam to skip
    public static double borderTopY     = 240.0;   //fraction of pixels from the top of the cam to skip
    public static double borderBottomY  = 240.0;   //fraction of pixels from the bottom of the cam to skip

    private double lowerruntime = 0;
    private double upperruntime = 0;

    public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 50.0, 150.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 120.0, 200.0);

    static final double FEET_PER_METER = 3.28084;
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 10.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.04;     // Larger is more responsive, but also less stable
//    private BNO055IMU imu = null;      // Control/Expansion Hub IMU
    private double  robotHeading  = 0;
    private double  headingOffset = 0;
    private double  headingError  = 0;
    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double  leftSpeed     = 0;
    private double  rightSpeed    = 0;
    private int     leftTarget    = 0;
    private int     rightTarget   = 0;
    public static double MULTIPLIER = 0.5;

    private int BleftTarget = 0;
    private int BrightTarget = 0;



    @Override
    public void runOpMode() {
        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline
        ContourPipeline myPipeline;
        webcam.setPipeline(myPipeline = new ContourPipeline(borderLeftX, borderRightX, borderTopY, borderBottomY));
        // Configuration of Pipeline
        myPipeline.configureScalarLower(scalarLowerYCrCb.val[0], scalarLowerYCrCb.val[1], scalarLowerYCrCb.val[2]);
        myPipeline.configureScalarUpper(scalarUpperYCrCb.val[0], scalarUpperYCrCb.val[1], scalarUpperYCrCb.val[2]);
        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * Это будет вызвано, если камеру невозможно открыть
                 */
            }
        });

        telemetry.update();
        robot.init_Auto(hardwareMap);
        waitForStart();

        //      while (opModeIsActive()) {
        myPipeline.configureBorders(borderLeftX, borderRightX, borderTopY, borderBottomY);
        if (myPipeline.error) {
            telemetry.addData("Exception: ", myPipeline.debug);
        }
        // Используйте эту строку кода только тогда, когда вы хотите найти нижнее и верхнее значения
//        testing(myPipeline);

        telemetry.addData("RectArea: ", myPipeline.getRectArea());
        telemetry.update();
        robot.init_Auto(hardwareMap);
        waitForStart();

        //      while (opModeIsActive()) {
        myPipeline.configureBorders(borderLeftX, borderRightX, borderTopY, borderBottomY);
        if (myPipeline.error) {
            telemetry.addData("Exception: ", myPipeline.debug);
        }
        // Only use this line of the code when you want to find the lower and upper values
        testing(myPipeline);

        telemetry.addData("RectArea: ", myPipeline.getRectArea());
        telemetry.update();

        if (myPipeline.getRectArea() > 2000) {
            if (myPipeline.getRectMidpointX() > 1066) {
                AUTONOMOUS_RIGHT(); // right
            } else if (myPipeline.getRectMidpointX() < 213) {
                AUTONOMOUS_LEFT(); // left
            } else {
                AUTONOMOUS_CENTER();
            }
        }
        //       }
    }

    public void testing(ContourPipeline myPipeline) {
        if (lowerruntime + 0.05 < getRuntime()) {
            CrLowerUpdate += -gamepad1.left_stick_y;
            CbLowerUpdate += gamepad1.left_stick_x;
            lowerruntime = getRuntime();
        }
        if (upperruntime + 0.05 < getRuntime()) {
            CrUpperUpdate += -gamepad1.right_stick_y;
            CbUpperUpdate += gamepad1.right_stick_x;
            upperruntime = getRuntime();
        }

        CrLowerUpdate = inValues(CrLowerUpdate, 0, 255);
        CrUpperUpdate = inValues(CrUpperUpdate, 0, 255);
        CbLowerUpdate = inValues(CbLowerUpdate, 0, 255);
        CbUpperUpdate = inValues(CbUpperUpdate, 0, 255);

        myPipeline.configureScalarLower(0.0, CrLowerUpdate, CbLowerUpdate);
        myPipeline.configureScalarUpper(255.0, CrUpperUpdate, CbUpperUpdate);

        telemetry.addData("lowerCr ", (int) CrLowerUpdate);
        telemetry.addData("lowerCb ", (int) CbLowerUpdate);
        telemetry.addData("UpperCr ", (int) CrUpperUpdate);
        telemetry.addData("UpperCb ", (int) CbUpperUpdate);
    }

    public Double inValues(double value, double min, double max) {
        if (value < min) {
            value = min;
        }
        if (value > max) {
            value = max;
        }
        return value;
    }

    public void AUTONOMOUS_CENTER() {
        telemetry.addLine("Center");
        telemetry.update();

        driveStraight(0.5, -12, 0);
        sleep(500);
        servoOpen();
        sleep(500);
//        driveStraight(0.5,5,0);
        driveStraight(0.5,10,0);
        sleep(500);
        turnToHeadingMecanum(0.3, 90);
        sleep(500);
        driveStraight(0.5, -22,0);
        off_motor();
    }

    public void AUTONOMOUS_LEFT() {
        telemetry.addLine("Auto Left");
        telemetry.update();

        driveStraight(0.5, -12, 0);
        sleep(500);
        turnToHeadingMecanum(0.5,-75);
        sleep(500);
        driveStraight(0.3,-1.5,0);
        sleep(500);
        servoOpen();
        sleep(500);
        driveStraight(0.3,3,0);
        sleep(500);
        turnToHeadingMecanum(0.3, 0);
        sleep(500);
        driveStraight(0.5,10,0);
        sleep(500);
        turnToHeadingMecanum(0.3, 90);
        sleep(500);
        driveStraight(0.5, -18,0);
        off_motor();

    }

    public void AUTONOMOUS_RIGHT() {
        telemetry.addLine("Auto Right");
        telemetry.update();

        driveStraight(0.5, -12, 0);
        sleep(500);
        turnToHeadingMecanum(0.5,75);
        sleep(500);
        driveStraight(0.5, -1.5, 0);
        servoOpen();
        sleep(500);
        driveStraight(0.3,3,0);
        sleep(500);
        turnToHeadingMecanum(0.3, 0);
        sleep(500);
        driveStraight(0.5,10,0);
        sleep(500);
        turnToHeadingMecanum(0.3, 90);
        sleep(500);
        driveStraight(0.5, -16,0);
        off_motor();
    }

    public void driveStraight(double maxDriveSpeed, double distance, double heading) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            leftTarget = robot.FrontLeftDriverMotor.getCurrentPosition() + moveCounts;
            rightTarget = robot.FrontRightDriverMotor.getCurrentPosition() + moveCounts;
            BleftTarget = robot.BackLeftDriverMotor.getCurrentPosition() + moveCounts;
            BrightTarget = robot.BackRightDriverMotor.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            robot.FrontLeftDriverMotor.setTargetPosition(leftTarget);
            robot.FrontRightDriverMotor.setTargetPosition(rightTarget);
            robot.BackLeftDriverMotor.setTargetPosition(BleftTarget);
            robot.BackRightDriverMotor.setTargetPosition(BrightTarget);

            robot.FrontLeftDriverMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FrontRightDriverMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BackLeftDriverMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BackRightDriverMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.FrontLeftDriverMotor.isBusy() && robot.FrontRightDriverMotor.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            robot.FrontLeftDriverMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FrontRightDriverMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BackLeftDriverMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BackRightDriverMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        robot.FrontLeftDriverMotor.setPower(leftSpeed * MULTIPLIER);
        robot.FrontRightDriverMotor.setPower(rightSpeed * MULTIPLIER);
        robot.BackRightDriverMotor.setPower(rightSpeed * MULTIPLIER);
        robot.BackLeftDriverMotor.setPower(leftSpeed * MULTIPLIER);
    }

    public double getRawHeading() {
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    void off_motor(){
        robot.BackLeftDriverMotor.setPower(0);
        robot.BackRightDriverMotor.setPower(0);
        robot.FrontRightDriverMotor.setPower(0);
        robot.FrontLeftDriverMotor.setPower(0);
    }

    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R", "%7d:%7d", leftTarget, rightTarget);
            telemetry.addData("Actual Pos L:R", "%7d:%7d", robot.FrontLeftDriverMotor.getCurrentPosition(), robot.FrontRightDriverMotor.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
        telemetry.addData("Error:Steer", "%5.1f:%5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L:R.", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }

    public void turnMecanum(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        robot.FrontLeftDriverMotor.setPower(leftSpeed * MULTIPLIER);
        robot.FrontRightDriverMotor.setPower(rightSpeed * MULTIPLIER);
        robot.BackLeftDriverMotor.setPower(leftSpeed * MULTIPLIER);
        robot.BackRightDriverMotor.setPower(rightSpeed * MULTIPLIER);
    }

    public void turnToHeadingMecanum(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            turnMecanum(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        turnMecanum(0, 0.0);
    }

    public void servoOpen(){
        robot.Box.setPosition(0.8);
    }
}
