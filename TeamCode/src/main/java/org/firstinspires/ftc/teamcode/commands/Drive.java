package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;


public class Drive extends CommandBase {

    private Drivetrain drivetrain;
    private double x;
    private double y;
    private double rotation;
    private boolean fieldOriented;

    public Drive(Drivetrain drivetrain, double x, double y, double rotation, boolean fieldOriented) {
        this.drivetrain = drivetrain;
        addRequirements(this.drivetrain);
        this.x = x;
        this.y = y;
        this.rotation = rotation;
        this.fieldOriented = fieldOriented;
    }

    // Executed on startup of the command running
    @Override
    public void initialize() {
    }

    // Continuously ran during command execution
    @Override
    public void execute() {
        drivetrain.drive(x,y,rotation,true, true);
    }

    // run once at the end of the command
    @Override
    public void end(boolean interrupted) {

    }

    // a continuous check to see if the command should still be executed
    @Override
    public boolean isFinished() {
        return false;
    }

}
