package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants.MagazineConstants;
import org.firstinspires.ftc.teamcode.subsystems.Magazine;

public class RunMagazine extends CommandBase {

    private Magazine magazine;
    private int state;

    public RunMagazine(Magazine magazine) {
        this.magazine = magazine;
        state = 0;

        addRequirements(this.magazine);
    }

    // Executed on startup of the command running
    @Override
    public void initialize() {

    }

    // Continuously ran during command execution
    @Override
    public void execute() {
        switch (state) {
            case 0:
                // raising the magazine
                magazine.runLiftMotor(MagazineConstants.liftPower);
                if(magazine.isTopSwitchActivated()) {
                    // check if the magazine is at the top
                    state = 1;
                    magazine.stop();
                }
                break;
            case 1:
                // lowering the magazine
                magazine.runLiftMotor(-MagazineConstants.liftPower);
                if(magazine.isBottomSwitchActivated()) {
                    // check if the magazine is at the top
                    state = 2;
                    magazine.stop();
                }
                break;
        }
    }

    // run once at the end of the command
    @Override
    public void end(boolean interrupted) {
        magazine.stop();
    }

    // a continuous check to see if the command should still be executed
    @Override
    public boolean isFinished() {
        if(state == 2) {
            return true;
        }
        return false;
    }

}