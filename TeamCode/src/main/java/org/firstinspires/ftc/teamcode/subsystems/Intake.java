package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake extends SubsystemBase {

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private Motor intakeMotor;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        intakeMotor = new Motor(hardwareMap, Constants.IntakeConstants.intakeMotorName);
    }

    public void runIntakeMotor(double power) {
        intakeMotor.set(power);
    }

    public void stop() {
        intakeMotor.set(0.0);
    }

    private void initMotors() {
        intakeMotor.setInverted(false);
        intakeMotor.setRunMode(Motor.RunMode.RawPower);
        intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

}
