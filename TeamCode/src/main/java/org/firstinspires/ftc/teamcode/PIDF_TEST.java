package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Max Velocity Test", group="Linear OpMode")
@Disabled
public class PIDF_TEST extends LinearOpMode {
    DcMotorEx leftLaunchDrive;
    DcMotorEx rightLaunchDrive;
    double leftCurrentVelocity;
    double rightCurrentVelocity;
    double rightMaxVelocity= 0.0;
    double leftMaxVelocity = 0.0;
    double launchPower = 1;

    @Override
    public void runOpMode() {
        leftLaunchDrive = hardwareMap.get(DcMotorEx.class, "left_launch_drive");
        rightLaunchDrive = hardwareMap.get(DcMotorEx.class, "right_launch_drive");
        waitForStart();

        leftLaunchDrive.setPower(launchPower);
        rightLaunchDrive.setPower(launchPower);

        while (opModeIsActive()) {
            leftCurrentVelocity = leftLaunchDrive.getVelocity();
            rightCurrentVelocity = rightLaunchDrive.getVelocity();

            if (leftCurrentVelocity > leftMaxVelocity) {
                leftMaxVelocity = leftCurrentVelocity;
            }
            if (rightCurrentVelocity > rightMaxVelocity) {
                rightMaxVelocity = rightCurrentVelocity;
            }

            telemetry.addData("left current velocity", leftCurrentVelocity);
            telemetry.addData("right current velocity", rightCurrentVelocity);
            telemetry.addData("left maximum velocity", leftMaxVelocity);
            telemetry.addData("right maximum velocity", rightMaxVelocity);
            telemetry.update();
        }
    }
}
