package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Constants.MagazineConstants;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Magazine extends SubsystemBase {

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private Motor liftMotor;

    public Magazine(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.liftMotor = new Motor(this.hardwareMap, MagazineConstants.liftMotorName);
    }

    private void initHardware() {
        liftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        liftMotor.setRunMode(Motor.RunMode.RawPower);
        liftMotor.setInverted(false);
    }

    public void runLiftMotor(double power) {
        liftMotor.set(power);
    }

    public boolean isTopSwitchActivated() {
        return true;
    }

    public boolean isBottomSwitchActivated() {
        return true;
    }

    public void stop() {
        liftMotor.set(0.0);
    }

}
