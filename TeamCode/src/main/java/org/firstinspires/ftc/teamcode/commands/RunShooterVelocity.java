package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class RunShooterVelocity extends CommandBase {

    Shooter shooter;

    public RunShooterVelocity(Intake Shooter) {
        this.shooter = shooter;
        addRequirements(this.shooter);
    }

    // Executed on startup of the command running
    @Override
    public void initialize() {
    }

    // Continuously ran during command execution
    @Override
    public void execute() {

        this.shooter.runShooterVelocity(Constants.ShooterConstants.frontShooterRPM);

        if(shooter.isReadyToShoot()) {
            shooter.feedToShooter();
        }

    }

    // run once at the end of the command
    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

    // a continuous check to see if the command should still be executed
    @Override
    public boolean isFinished() {
        return false;
    }
}

