package org.firstinspires.ftc.teamcode.opModes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.teamcode.commands.RunIntake;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Magazine;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp(name = "TeleOp")
public class TeleOpMode extends CommandOpMode {

    // subsystems
    private Drivetrain m_drivetrain;
    private Intake m_intake;
    private Magazine m_magazine;
    private Shooter m_shooter;

    // commands
    RunIntake m_runIntake;

    @Override
    public void initialize() {
        m_drivetrain = new Drivetrain(hardwareMap, telemetry);
        m_intake = new Intake(hardwareMap, telemetry);
        m_magazine = new Magazine(hardwareMap, telemetry);
        m_shooter = new Shooter(hardwareMap, telemetry);

        register(m_drivetrain, m_intake, m_magazine, m_shooter);
    }

}
