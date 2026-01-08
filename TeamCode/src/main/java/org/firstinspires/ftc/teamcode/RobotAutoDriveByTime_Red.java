/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.BasicOmniOpMode_Linear_BB_25_26.LAUNCH_POWER_LESS;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Goal Shooting Red", group="Robot")
public class RobotAutoDriveByTime_Red extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    //telescoping arm
    //private DcMotor armDrive = null;

    /*
    servo claw

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position
    */

    // Define class members
    Servo gateServo;
    //Servo clawServo;
    //Servo hingeServo;

    // TODO define static constants for gate up and down positions.

    double gatePosition = 0.0;
    //double  clawPosition = 0.50;  //guessing middle is 0.50
    //double  hingePosition = 0.0; // Standard servo TODO: Find right values through testing (value should be up)

    // Adjust these numbers to suit your robot.
    // final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)

    //  * Setup for drive by April Tag

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.3  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0) = 0.02
    final double STRAFE_GAIN =  0.02 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   =  0.02  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.3;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    final double APRIL_DISTANCE_THRESHOLD = 0.5;    // Current distance from april tag is under this threshold distance from desired distance.

    final int LEFT_TICK_THRESHOLD = 100; //TODO: tune amount

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = 24;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    private IMU imu         = null;      // Control/Expansion Hub IMU

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  targetHeading = 0;
    private double  turnSpeed     = 0;

    // * Setup for turn by Gyro
    private double          headingError  = 0;

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     TURN_SPEED              = 0.3;     // Max turn speed to limit turn rate.
    static final double     HEADING_THRESHOLD       = 0.5 ;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.

    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not correct strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.5;     // Larger is more responsive, but also less stable.
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable.

    //Constants for the gate
    static final double GATE_UP = 0.6;
    static final double GATE_DOWN = 0;

    // Set constants for launch speed
    static final double LAUNCH_SPEED = 0.56; // TODO: test value

    // * Setup for Drive by Time
    private final ElapsedTime     runtime = new ElapsedTime();

    // static final double     FORWARD_SPEED = 0.6;
    // static final double     TURN_SPEED    = 0.5;

    @Override
    public void runOpMode() {

        // Initialize the Apriltag Detection process
        initAprilTag();


        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        //set up encoder for measurement
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotor rightLaunchDrive = hardwareMap.get(DcMotor.class, "right_launch_drive"); // Port 0
        // Launchers
        DcMotor leftLaunchDrive = hardwareMap.get(DcMotor.class, "left_launch_drive");

        // Set up launch motors
        rightLaunchDrive.setDirection(DcMotor.Direction.REVERSE);
        leftLaunchDrive.setDirection(DcMotor.Direction.FORWARD);

        //add config for new motor (telescoping arm)
        /*armDrive = hardwareMap.get(DcMotor.class, "arm_drive");
        armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armDrive.setTargetPosition(0);
        armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armDrive.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        // Set direction for the telescoping arm
        armDrive.setDirection(DcMotor.Direction.REVERSE);
        */

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         */
        //TODO: Fix gyro
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Connect to servo
        //clawServo = hardwareMap.get(Servo.class, "claw_hand");

        // Connect to servo
        //hingeServo = hardwareMap.get(Servo.class, "arm_hinge");

        // Connect to servo
        gateServo = hardwareMap.get(Servo.class, "gate_servo");

        // Wait for the game to start (Display Gyro value while waiting)
        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.update();
        }

        imu.resetYaw();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        // Send telemetry message to indicate successful Encoder reset
        /* telemetry.addData("Arm location starting at",  "%7d",
            armDrive.getCurrentPosition());
         */
        telemetry.update();


        /* if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
         */

        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();


        /*
        //int targetTicks = 0;

        // Step through each leg of the path, ensuring that the OpMode has not been stopped along the way.
        //          holdHeading() is used after turns to let the heading stabilize
        //          Add a sleep(2000) after any step to keep the telemetry data visible for review

        //calculate arm power based on joystick from gamepad2
        double armPower = 0.8;

        // Declare new target value for specimen hanging height
        int hangingSpecimenTicks = 3200; //TODO: test value

        // Declare new target value for arm bottom height
        int armDownTicks = 1500;

        // Declare new target value for specimen pick up height
        int specimenGrabTicks = 3900;

        // Declare new target value for specimen lift-off height
        int specimenLiftTicks = 4200; //TODO: test value

        // Declare new target value for specimen lift-off height
        double hingeDown = 0.01; //TODO: test value

        // Declare new target value for specimen lift-off height
        double hingeUp = 0.29; //TODO: test value


        // Step 1: Close claw
        clawPosition = 0.50;
        clawServo.setPosition(clawPosition);

        // Step: Raise hinge
        hingePosition = hingeUp;
        hingeServo.setPosition(hingePosition);

        // Step 2: Lift arm
        encoderArm(armPower, hangingSpecimenTicks, 3);

        // Step 3: Drive forward for x amount of time and stop
        driveByTime(0.5, 0, 0, 2,2000);

        // Step 4: Pull arm down
        encoderArm(armPower, armDownTicks, 2);


        // Step: Open claw
        clawPosition = 0.90;
        clawServo.setPosition(clawPosition);

        // Step: Drive back halfway
        //driveByTime(-0.5,0,0,0.5,0);
        driveByTime(-0.4,0,0,2,-800);

        // Step: Rotate 90 degrees clockwise
        turnToHeading( TURN_SPEED, -90.0);

        // Step: Raise arm
        encoderArm(armPower, specimenGrabTicks, 2);

        // Step: Drive forward TODO: drive with april tags
        driveToAprilTag(27);
        //driveByTime(0.6,0 , 0, 1.8);

        // Step: Rotate an additional 90 degrees clockwise
        turnToHeading( TURN_SPEED, -180.0);

        // Step: Lower arm hinge to specimen pick - up height
        hingePosition = hingeDown;                       // TODO: Change values through testing
        hingeServo.setPosition(hingePosition);

        // Step: Drive towards from observation zone
        //driveByTime(0.4,0,0,0.04,0); // TODO: Change values through testing

       // driveByTime(-0.15,0,0,1,-10);
        driveByTime(0.2,0,0,1,30);
        sleep(2500);

        // Step: Close claw
        clawPosition = 0.50;
        clawServo.setPosition(clawPosition);
        sleep(500);

        // Step: Drive forward into observation zone
        //driveByTime(0.4,0,0,0.01,2); // TODO: Change values through testing
        driveByTime(0.15,0,0,1,10);

        // Step: Lift arm slightly to disengage from wall
        encoderArm(armPower, specimenLiftTicks, 1);

        // Step: Raise arm hinge
        hingePosition = hingeUp;                       // TODO: Change values through testing
        hingeServo.setPosition(hingePosition);
        sleep(500);

        // Step: Back up
        //driveByTime(-0.15,0,0,1,-60); // TODO: Change values through testing
        //driveByTime(-0.4,0,0,0.5,8);

        // Step: Rotate 90 degrees CCW
        turnToHeading( TURN_SPEED, -90);

        // Step: drive backwards back to submersible
        driveToAprilTag(58);
        //driveByTime(0.6,0 , 0, 1.55);

        // Step: Raise arm
        encoderArm(armPower, hangingSpecimenTicks, 3);

        // Step: Rotate 90 degrees CW
        turnToHeading( TURN_SPEED, 0);

        // Step: Drive forwards
        //driveByTime(0.5,0,0,0.5,0);
        driveByTime(0.5,0,0,1,3700);

        // Step: Pull arm down
        encoderArm(armPower, armDownTicks, 1);

        // Step: Open claw
        clawPosition = 0.90;
        clawServo.setPosition(clawPosition);

        // Step: Strafe back and to the right to observation zone
        driveByTime(-0.25, 0.5, 0, 2,0);
        */
        //Step: Wait for other team shooting
        sleep(15000);

        //Find April Tag
        driveToAprilTag(27,40);

        //Step: Launch wheels rolling
        leftLaunchDrive.setPower(LAUNCH_SPEED);
        rightLaunchDrive.setPower(LAUNCH_SPEED);
        telemetry.addData("Step", "2");
        telemetry.update();

        //Step: Wait for wheel momentum
        sleep(2000);
        telemetry.addData("Step","3");
        telemetry.update();

        //Step: Servo push the ball up 1
        gateServo.setPosition(GATE_UP);
        telemetry.addData("Step", "4");
        telemetry.update();

        //Step: sleep
        sleep(1000);

        //Step: put the gate down
        gateServo.setPosition(GATE_DOWN);
        telemetry.addData("Gate", "down");
        telemetry.update();

        //Step: Pause
        sleep(750);

        //Step: Servo push ball 2
        gateServo.setPosition(GATE_UP);
        telemetry.addData("Gate", "Up");
        telemetry.update();

        //Step: sleep
        sleep(1000);

        //Step: put the gate down
        gateServo.setPosition(GATE_DOWN);
        telemetry.addData("Gate", "down");
        telemetry.update();

        //Step: Pause
        sleep(750);

        //Step: Servo push ball 3
        gateServo.setPosition(GATE_UP);
        telemetry.addData("Gate", "Up");
        telemetry.update();

        //Step: sleep
        sleep(1000);

        //Step: put the gate down
        gateServo.setPosition(GATE_DOWN);
        telemetry.addData("Gate", "down");
        telemetry.update();

        //Step: back up
        driveByTime(-0.3, 0,0,1,0);

        //Step: drive forward
        driveByTime(0.6,0,0,1,0);

        // Step: Park and wait for TeleOp
        // TODO

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000); //TODO: Change value w/ testing
    }

    /*
     *  Method to perform a move to an exact position, based on encoder counts.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    /*
    public void encoderArm(double speed,
                             int targetTicks,
                             double timeoutS) {
        // if you're already at the target do nothing
        if (armDrive.getCurrentPosition() == targetTicks) {
            return;
        }
        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            armDrive.setTargetPosition(targetTicks);

            // reset the timeout time and start motion.
            runtime.reset();
            armDrive.setPower(Math.abs(speed));

            //TODO: figure out how to include the while (opModeISActive() loop in code above
            //the encoder resolution is 537.7 PPR, this is for a 19.2:1 gear ratio
            //initial guess for fully extending the slide: about 15,000 - WAY TO MUCH

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    armDrive.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Running telescoping arm to", " %7d", targetTicks);
                telemetry.addData("Currently at", " at %7d",
                        armDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            armDrive.setPower(0.05); // to prevent drift... ZeroPowerBehavior.BRAKE doesn't work

        }
    }

    */

    /*
     *  Method to perform a move to an exact position, based on encoder counts.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     *
     *  TODO: Make a new copy of this that takes both timeout and encoder ticks.
     */
    public void driveByTime(double axial,
                            double lateral,
                            double yaw,
                            double timeout,
                            int ticks) {

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max;
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send calculated power to wheels to go
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        int leftPosition = leftFrontDrive.getCurrentPosition();
        int threshold = LEFT_TICK_THRESHOLD;

        if (Math.abs(ticks) < LEFT_TICK_THRESHOLD) {
            threshold = 1;
        }

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < timeout) &&
                ( (ticks == 0) || (Math.abs(leftPosition - ticks) > threshold))) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.addData("Left front wheel target", "at %7d");
            telemetry.addData("Left front wheel currently at", " at %7d",
                    leftPosition);
            telemetry.update();
            leftPosition = leftFrontDrive.getCurrentPosition();
        }

        // Send calculated power to wheels to stop
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = -1 * turnSpeed;
            double rightFrontPower = turnSpeed;
            double leftBackPower = -1 * turnSpeed;
            double rightBackPower = turnSpeed;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            double max;
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Send calculated power to wheels to go
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }


    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        double steeringCorrection = 0;

        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        // Adjust gain according to severity of error.  When error is low, turn slower.
        if (headingError > 20) {
            steeringCorrection = Range.clip(headingError * proportionalGain, -1, 1);
        } else {
            steeringCorrection = Range.clip(headingError * proportionalGain * 0.2, -1, 1);
        }

        return steeringCorrection;
    }
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.addData("Threshold :",  "%5.1f", HEADING_THRESHOLD);
        telemetry.update();
    }
    public void driveToAprilTag(double desiredDistance, double desiredHeading) {

        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)
        List<AprilTagDetection> currentDetections;

        desiredTag  = null;

        runtime.reset();
        while (opModeIsActive() && !targetFound && runtime.seconds() < 0.5) {
            // Step through the list of detected tags and look for a matching tag
            currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }
        }

        // Tell the driver what we see, and what to do.
        if (targetFound) {
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);

            // TODO: Wrap this in a while loop.  Keep looping while
            // absolute value of (distance from april tag - desiredDistance) is more than 0.5 inch.
            while (opModeIsActive() && targetFound &&
                    (Math.abs(desiredTag.ftcPose.range - desiredDistance) > APRIL_DISTANCE_THRESHOLD) &&
                    (Math.abs(desiredTag.ftcPose.bearing - desiredHeading) > APRIL_DISTANCE_THRESHOLD) ) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (desiredTag.ftcPose.range - desiredDistance);
                double headingError = desiredTag.ftcPose.bearing - desiredHeading;
                double yawError = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                telemetry.update();

                // Apply desired axes motions to the drivetrain.
                moveRobot(drive, strafe, turn);
                sleep(10);

                // refresh the camera data
                currentDetections = aprilTag.getDetections();
                targetFound = false;
                for (AprilTagDetection detection : currentDetections) {
                    // Look to see if we have size info on this tag.
                    if (detection.metadata != null) {
                        //  Check to see if we want to track towards this tag.
                        if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                            // Yes, we want to use this tag.
                            targetFound = true;
                            desiredTag = detection;
                            break;  // don't look any further.
                        } else {
                            // This tag is in the library, but we do not want to track it right now.
                            telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                        }
                    } else {
                        // This tag is NOT in the library, so we don't have enough information to track to it.
                        telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                    }
                }
            }

            // Stop all motion;
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);

        } else {
            // Error case. Could not find an April Tag.
            telemetry.addData("\n>","NO TARGET FOUND\n");
            telemetry.update();

            sleep(2000);
        }
    }

    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    /* private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);


        }

    }

     */
}