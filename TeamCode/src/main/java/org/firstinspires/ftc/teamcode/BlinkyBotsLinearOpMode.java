/* Copyright (c) 2021 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * This is a base class meant to be extended by other OpModes
 * This assumes the robot has mecanum drive, a ramp launcher with 2 motors, and a "gate" servo
 * TODO: We want to have a webcam and odometry
 * TODO: gyro driving for hard turn 180
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

/*
 * TODO: 2025-26 Blinky Bots main requirements
 * 4 motors for the wheels
 * 2 motors for launcher
 * Servo for gate
 * Roadrunner odometry pods (3?)
 */

/*
 * TODO: Shoot-By-Velo
 *
 * - Switch to DcMotorEx.  Initialize runmode.
 *
 * - Add a Test op mode to get PIDF values
 *   https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit?tab=t.0#heading=h.k1p1szg5soey
 *
 * - Add a shooting mode variable (power? velo? hybrid?)
 *
 * - Add option to DRIVE_BY_ENCODER if in velo or hybrid mode
 *
 * - modify autoshoot
 *      - drive by velo for target velo
 *      - get the power level
 *      - switch to drive by power
 * -
 */

@TeleOp(name="Basic: Omni Linear OpMode (normal servo) 25-26", group="Linear OpMode")
@Disabled
public abstract class BlinkyBotsLinearOpMode extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private final ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;

    // Declare OpMode members for the launch motors
    public DcMotorEx rightLaunchDrive = null;
    public DcMotorEx leftLaunchDrive = null;

    // Set constant power level for the launch motors
    // 13.8 V: .56 and .5
    // 12.3 V: .6 and .55
    static final double LAUNCH_POWER_LESS = 0.5; //TODO: Tune value (between 0 and 1)
    static final double LAUNCH_POWER_MORE = 0.56;
    // Set up a variable for each launch wheel to set power level
    private double launchPower = 0;
    private double launchTrim = 0; // TODO: This is buggy - always runs

    public double leftFrontPower = 0;
    public double rightFrontPower = 0;
    public double leftBackPower = 0;
    public double rightBackPower = 0;

    private final ElapsedTime automatedShootTimer = new ElapsedTime();
    private boolean automatedShootRunning = false;

    // Servo for release mechanism 
    private Servo gateServo;
    static final double GATE_UP = 0.4; // 0.4 is at 90
    static final double GATE_DOWN = 0;
    double gatePosition = GATE_DOWN; // TODO: Change if need be
    // Camera stuff
    final double TURN_GAIN   =  0.1  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    final double APRIL_BEARING_THRESHOLD = 1;    // Current bearing from april tag is under this threshold distance from desired distance.
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    public static final int RED_DESIRED_TAG_ID = 24;   // April tag ID for the red launch goal
    public static final int BLUE_DESIRED_TAG_ID = 20;     // April tag ID for the blue launch goal
    private int desiredTagId = 0;                    // Holds the desired tag ID for shooting
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    public long CYCLE_MS = 0;

    void addTelemetry() {
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.addData("Launcher Left/Right", "%4.2f", launchPower);
        telemetry.addData("gatePosition", "%4.2f", gatePosition);
        telemetry.addData("Trim amount", "Trim amount", launchTrim);
        telemetry.addData(">", "Press Stop to end test.");
        telemetry.update();
    }

    void sendMotorValues() {
        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        // Set calculated power to launcher
        leftLaunchDrive.setPower(launchPower);
        rightLaunchDrive.setPower(launchPower);

        //Set the gate servo to new position and pause
        gateServo.setPosition(gatePosition);
    }

    void readControllerInputs() {

        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        //gamepad2 - control launch mechanism - right bumper pressed for release
        boolean rightBumperPressed = gamepad2.right_bumper;
        boolean leftBumperPressed = gamepad2.left_bumper;

        //dpad up pressed for addition
        boolean dpadUpPressed = gamepad2.dpad_up;

        //dpad down pressed for subtraction
        boolean dpadDownPressed = gamepad2.dpad_down;

        //gamepad2 - control gate movement - x pressed for open
        boolean buttonXPressed = gamepad2.x;

        //gamepad2 - control gate movement - y pressed for open
        boolean buttonYPressed = gamepad2.y;

        //control launch velocity trim through bumpers
        if (dpadUpPressed) {
            launchTrim -= 0.01;
        } else if (dpadDownPressed) {
            launchTrim += 0.01;
        }

        //Control gate movement through buttons
        if (!automatedShootRunning) {
            if (buttonXPressed) { //down position
                gatePosition = GATE_DOWN;
            } else if (buttonYPressed) { //up position
                gatePosition = GATE_UP;
            }

            // Control launcher movement through bumper
            if (leftBumperPressed) {
                launchPower = LAUNCH_POWER_MORE;
            } else if (rightBumperPressed) {
                launchPower = LAUNCH_POWER_LESS;
            } else {
                launchPower = 0;
            }
        }

        //Button pressed for automated shooting
        if (gamepad2.left_trigger > 0.2) {
            yaw = automatedShoot(LAUNCH_POWER_MORE);

        } else if (gamepad2.right_trigger > 0.2) {
            yaw = automatedShoot(LAUNCH_POWER_LESS);

        } else {
            endAutomatedShoot();
        }

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        leftFrontPower = axial + lateral + yaw;
        rightFrontPower = axial - lateral - yaw;
        leftBackPower = axial - lateral + yaw;
        rightBackPower = axial + lateral - yaw;


        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        int divider = 3;
        double divider2 = 1.5;

        // if button LB is pressed, the speed will increase
        boolean buttonLBPressed = gamepad1.left_bumper;  // B gamepad 1
        boolean buttonLTPressed = gamepad2.left_trigger > 0.1;
        boolean buttonRTPressed = gamepad2.right_trigger > 0.1;

        if (buttonLBPressed) {
            leftFrontPower /= divider;
            rightFrontPower /= divider;
            leftBackPower /= divider;
            rightBackPower /= divider;
        }

        if (buttonLTPressed || buttonRTPressed) {
            leftFrontPower /= divider2;
            rightFrontPower /= divider2;
            leftBackPower /= divider2;
            rightBackPower /= divider2;
        }
    }

    void initializeHardware() {

        // Initialize the Apriltag Detection process
        initAprilTag();

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        //Add config for new motors (launch wheels)
        rightLaunchDrive = hardwareMap.get(DcMotorEx.class, "right_launch_drive"); // Port 0
        leftLaunchDrive = hardwareMap.get(DcMotorEx.class, "left_launch_drive");

        // Connect to servo
        gateServo = hardwareMap.get(Servo.class, "gate_servo");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // TODO: Set direction for the launch mechanism
        leftLaunchDrive.setDirection(DcMotor.Direction.FORWARD);
        rightLaunchDrive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }


    public double automatedShoot(double targetLaunchPower) {

        double turn = 0;

        // Track whether we're mid-shoot.  Only reset the timer on first entry
        if (!automatedShootRunning) {
            automatedShootTimer.reset();
            automatedShootRunning = true;
        }

        // Save the timer value so we don't keep re-reading the timer.
        // Re-reading the timer on every 'if' evaluation would introduce time gaps.
        double elapsedTime = automatedShootTimer.seconds();
        telemetry.addData("Elapsed Time", "%4.2f", elapsedTime);
        if (elapsedTime < 2.0) {
            // First 2 seconds

            // Step: Launch wheels rolling and wait for wheel momentum
            // Set the variable. Main loop will send this power level to motors.
            launchPower = targetLaunchPower;

            turn = rotateToAprilTag();

            // Don't send the telemetry yet. Just add lines.
            telemetry.addData("Step 1", "Setting power");

        } else if (elapsedTime < 2.25) {
            // Do this in seconds 2-3

            // Step: Send gate up to push ball 1
            gatePosition = GATE_UP;

            // Don't send the telemetry yet. Just add lines.
            telemetry.addData("Step 2 Gate", "Up");

        } else if (elapsedTime < 3.25) {
            // Do this in seconds 3.0 - 3.5

            //Step: put the gate down
            gatePosition = GATE_DOWN;
            telemetry.addData("Gate", "down");

        } else if (elapsedTime < 3.5) {
            // Step: Send gate up to push ball 2
            gatePosition = GATE_UP;
            telemetry.addData("Gate", "Up");

        } else if (elapsedTime < 4.5) {
            //Step: put the gate down
            gatePosition = GATE_DOWN;
            telemetry.addData("Gate", "down");

        } else if (elapsedTime < 5.0) {
            // Step: Send gate up to push ball 3
            gatePosition = GATE_UP;
            telemetry.addData("Gate", "Up");

        }

        telemetry.addData("Automated Shoot", "True");
        return turn;
    }

    /*
     * Method to end an automated shoot and reset any motors and servos.
     */
    public void endAutomatedShoot() {
        telemetry.addData("Starting Automated Shoot", "False");

        if (automatedShootRunning) {
            automatedShootRunning = false;

            // Put the gate down
            gatePosition = GATE_DOWN;

            // Turn off launch motors
            launchPower = 0;
        }

        telemetry.addData("Automated Shoot", "False");
    }
    public double rotateToAprilTag() {

        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)
        List<AprilTagDetection> currentDetections;

        desiredTag  = null;

        // Step through the list of detected tags and look for a matching tag
        currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((desiredTagId < 0) || (detection.id == desiredTagId)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }


        // Tell the driver what we see, and what to do.
        if (targetFound) {
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);

            // absolute value of (distance from april tag - desiredDistance) is more than 0.5 inch.
            if ( (desiredTagId == RED_DESIRED_TAG_ID) && (Math.abs(desiredTag.ftcPose.bearing) > APRIL_BEARING_THRESHOLD) ){

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double headingError = -desiredTag.ftcPose.bearing;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }else if ( (desiredTagId == BLUE_DESIRED_TAG_ID) && (Math.abs(desiredTag.ftcPose.bearing - 5) > APRIL_BEARING_THRESHOLD) ){

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double headingError = -desiredTag.ftcPose.bearing - 5;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }



        } else {
            // Error case. Could not find an April Tag.
            telemetry.addData("\n>","NO TARGET FOUND\n");
            telemetry.update();
        }
        return turn;
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
    public void setDesiredTagId(int desiredTagId) {
        this.desiredTagId = desiredTagId;
    }
}


