package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;

public class RunIntake extends CommandBase {

    private Intake intake;
    private boolean reverseDirection;

    public RunIntake(Intake intake, boolean reverseDirection) {
        this.intake = intake;
        this.reverseDirection = reverseDirection;
        addRequirements(this.intake);
    }

    // Executed on startup of the command running
    @Override
    public void initialize() {
    }

    // Continuously ran during command execution
    @Override
    public void execute() {
        if(!reverseDirection) {
            intake.runIntakeMotor(IntakeConstants.intakePower);
        } else {
            intake.runIntakeMotor(-IntakeConstants.intakePower);
        }
    }

    // run once at the end of the command
    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

    // a continuous check to see if the command should still be executed
    @Override
    public boolean isFinished() {
        return false;
    }
}