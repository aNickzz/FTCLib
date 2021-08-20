package org.firstinspires.ftc.teamcode.commands;

import android.media.MediaPlayer;
import android.provider.MediaStore;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import java.io.File;


public class ControllerDrive extends CommandBase {

    private Drivetrain drivetrain;
    private Gamepad gamepad = null;
    private Telemetry telemetry;

    private DriveMode driveMode;
    private HardwareMap hardwareMap;

    private int hornSoundID;
    private boolean hornFound = false;
    private String lastError = null;

    public enum DriveMode {
        /**
         * Movement inputs are relative to the robot
         */
        RELATIVE(false),
        /**
         * Movement inputs are relative to the arena
         */
        ABSOLUTE(true);

        public boolean fieldOriented;

        DriveMode(boolean fieldOriented) {
            this.fieldOriented = fieldOriented;
        }
    }

    public ControllerDrive(Drivetrain drivetrain, Gamepad gamepad, HardwareMap hardwareMap, Telemetry telemetry) {
        this.drivetrain = drivetrain;
        this.gamepad = gamepad;
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        driveMode = DriveMode.RELATIVE;

        addRequirements(this.drivetrain);


        hornSoundID = hardwareMap.appContext.getResources().getIdentifier("horn", "raw", hardwareMap.appContext.getPackageName());

        if (hornSoundID != 0)
            hornFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, hornSoundID);
    }

    // Executed on startup of the command running
    @Override
    public void initialize() {
    }

    private boolean rightBumperWasPressed = false;
    private boolean rightThumbstickWasPressed = false;

    // Continuously ran during command execution
    @Override
    public void execute() {
        double xVelocity = gamepad.left_stick_x;
        double yVelocity = gamepad.left_stick_y;
        double angularVelocityInput = 0*gamepad.right_stick_x;

        boolean rightBumper = gamepad.back;
        boolean toggleDriveMode = (rightBumper && !rightBumperWasPressed);
        rightBumperWasPressed = rightBumper;

        boolean precisionMode = gamepad.left_bumper;
        boolean rightThumbstick = gamepad.right_stick_button;

        boolean hornButton = rightThumbstick && !rightThumbstickWasPressed;
        rightThumbstickWasPressed = rightThumbstick;

        if (toggleDriveMode) {
            driveMode = driveMode == DriveMode.RELATIVE ? DriveMode.ABSOLUTE : DriveMode.RELATIVE;
        }

        double rotationDeadZone = Constants.ControllerDriveConstants.rotationDeadzone;
        if (Math.abs(angularVelocityInput) < rotationDeadZone) {
            angularVelocityInput = 0;
        } else {
            angularVelocityInput = (angularVelocityInput - Math.copySign(rotationDeadZone, angularVelocityInput)) * (1 - rotationDeadZone);
        }
        double angularVelocity = angularVelocityInput;

        if (precisionMode) {
            xVelocity *= 0.25;
            yVelocity *= 0.25;
            angularVelocity *= 0.25;
        }

        drivetrain.drive(xVelocity, yVelocity, angularVelocity, driveMode.fieldOriented, true);

        if (hornButton) {
            if (hornFound)
                audioPlayer(hornSoundID);
            else
                lastError = "Couldn't find horn.wav";
        }


        telemetry.addData("Drive Mode", driveMode.name());
        telemetry.addLine("Left bumper for precision mode");

        telemetry.addData("Left azimuth", drivetrain.leftModule.getAzimuthAngle().getDegrees());
        telemetry.addData("Left target", drivetrain.leftModule.getStateAngle());
        telemetry.addData("Left delta", drivetrain.leftModule.calculatePController());

        telemetry.addData("Right azimuth", drivetrain.rightModule.getAzimuthAngle().getDegrees());
        telemetry.addData("Right target", drivetrain.rightModule.getStateAngle());
        telemetry.addData("Right delta", drivetrain.rightModule.calculatePController());

        telemetry.addData("left_stick_x", gamepad.left_stick_x);
        telemetry.addData("left_stick_y", gamepad.left_stick_y);
        telemetry.addData("right_stick_x", gamepad.right_stick_x);
        telemetry.addData("left_trigger", gamepad.left_trigger);

        if (lastError != null) {
            telemetry.addData("ERR", lastError);
        }
        telemetry.update();
    }

    public void audioPlayer(int resourceID) {
        SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, resourceID);

    }

    // run once at the end of the command
    @Override
    public void end(boolean interrupted) {
        SoundPlayer.getInstance().stopPlayingAll();
    }

    // a continuous check to see if the command should still be executed
    @Override
    public boolean isFinished() {
        return false;
    }

}
