package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;


public class InteractiveCalibration extends CommandBase {

    private Drivetrain drivetrain;
    private Gamepad gamepad = null;
    private Telemetry telemetry;

    public MotorEx leftDriveMotor;
    public CRServo leftAzimuthServo;
    public Motor.Encoder leftAzimuthEncoder;

    public MotorEx rightDriveMotor;
    public CRServo rightAzimuthServo;
    public Motor.Encoder rightAzimuthEncoder;

    private boolean stopPlz = false;

    public InteractiveCalibration(Drivetrain drivetrain, Gamepad gamepad, HardwareMap hardwareMap, Telemetry telemetry) {
        this.drivetrain = drivetrain;
        this.gamepad = gamepad;
        this.telemetry = telemetry;
        addRequirements(this.drivetrain);

        leftDriveMotor = drivetrain.leftModule.driveMotor;
        leftAzimuthServo = drivetrain.leftModule.azimuthServo;
        leftAzimuthEncoder = drivetrain.leftModule.azimuthEncoder;

        rightDriveMotor = drivetrain.rightModule.driveMotor;
        rightAzimuthServo = drivetrain.rightModule.azimuthServo;
        rightAzimuthEncoder = drivetrain.rightModule.azimuthEncoder;
    }

    // Executed on startup of the command running
    @Override
    public void initialize() {
    }


    private String state = "";

    // Continuously ran during command execution
    @Override
    public void execute() {
        if (gamepad.left_bumper) {
            leftAzimuthEncoder.reset();
        }

        if (gamepad.right_bumper) {
            rightAzimuthEncoder.reset();
        }

        if (gamepad.start)
            stopPlz = true;

        switch (state) {
            case "rotate":
                if (leftAzimuthEncoder.getPosition() > Constants.DrivetrainConstants.ticksPerModuleRotation){
                    leftAzimuthServo.set(0.0);
                }
                if (rightAzimuthEncoder.getPosition() > Constants.DrivetrainConstants.ticksPerModuleRotation){
                    rightAzimuthServo.set(0.0);
                }
                if (gamepad.b) {
                    state = "";
                    leftAzimuthServo.set(0.0);
                    rightAzimuthServo.set(0.0);
                }
                break;
            default:
                if (gamepad.a) {
                    state = "rotate";
                    leftAzimuthServo.set(-1);
                    rightAzimuthServo.set(-0.5);
                }
                break;
        }


        telemetry.addData("Interactive Calibration", "");
        telemetry.addData("Left Azimuth", leftAzimuthEncoder.getPosition());
        telemetry.addData("Left Swerve Angle", drivetrain.getLeftDriveAngle().getDegrees());
        telemetry.addLine();
        telemetry.addData("Right Azimuth", rightAzimuthEncoder.getPosition());
        telemetry.addData("Right Swerve Angle", drivetrain.getRightDriveAngle().getDegrees());
        telemetry.addLine();
        telemetry.addLine("Press start to end calibration");

        telemetry.addLine();
        telemetry.addData("left_stick_x", gamepad.left_stick_x);
        telemetry.addData("left_stick_y", gamepad.left_stick_y);
        telemetry.addData("right_stick_x", gamepad.right_stick_x);
        telemetry.addData("right_stick_y", gamepad.right_stick_y);
        telemetry.addData("left_trigger", gamepad.left_trigger);
        telemetry.addData("right_trigger", gamepad.right_trigger);
        telemetry.update();

    }

    // run once at the end of the command
    @Override
    public void end(boolean interrupted) {

    }

    // a continuous check to see if the command should still be executed
    @Override
    public boolean isFinished() {
        return stopPlz;
    }

}
