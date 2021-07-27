package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;


public class Shooter extends SubsystemBase {

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private MotorEx frontShooterMotor;
    private MotorEx backShooterMotor;
    private ServoEx feederServo;
    private int frontShooterMotorTargetVelocity;
    private int backShooterMotorTargetVelocity;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        frontShooterMotor = new MotorEx(hardwareMap, ShooterConstants.frontMotorName);
        backShooterMotor = new MotorEx(hardwareMap, ShooterConstants.backMotorName);
        feederServo = new SimpleServo(
                hardwareMap,
                ShooterConstants.feederServoName,
                ShooterConstants.feederServoMinAngle,
                ShooterConstants.feederServoMaxAngle,
                AngleUnit.DEGREES
        );

        frontShooterMotorTargetVelocity = 0;
        backShooterMotorTargetVelocity = 0;
        initHardware();
    }

    private void initHardware() {
        frontShooterMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        frontShooterMotor.setInverted(false);
        backShooterMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        backShooterMotor.setInverted(false);
    }

    /**
     * Run the shooter based on percent power with independent control of each flywheel.
     * @param backPower (double) The percentage power to be sent to the back flywheel between -1.0 - 1.0.
     * @param frontPower (double) The percentage power to be sent to the front flywheel between -1.0 - 1.0.
     */
    public void runShooterPower(double backPower, double frontPower) {
        frontShooterMotor.setRunMode(Motor.RunMode.RawPower);
        backShooterMotor.setRunMode(Motor.RunMode.RawPower);
        frontShooterMotor.set(frontPower);
        backShooterMotor.set(backPower);
    }

    /**
     * Run the shooter based on velocity with independent control of each flywheel.
     * @param backVelocity (int) The RPM the back flywheel is set to.
     * @param frontVelocity (int) The RPM the front flywheel is set to.
     */
    public void runShooterVelocity(int backVelocity, int frontVelocity) {
        frontShooterMotor.setRunMode(Motor.RunMode.VelocityControl);
        backShooterMotor.setRunMode(Motor.RunMode.VelocityControl);
        frontShooterMotorTargetVelocity = frontVelocity;
        backShooterMotorTargetVelocity = backVelocity;
        frontShooterMotor.set(rpmToTicksPerSec(frontVelocity));
        backShooterMotor.set(rpmToTicksPerSec(backVelocity));
    }

    /**
     * Run the shooter based on percent power.
     * @param power The percentage power to be sent to both flywheels between -1.0 - 1.0.
     */
    public void runShooterPower(double power) {
        frontShooterMotor.setRunMode(Motor.RunMode.RawPower);
        backShooterMotor.setRunMode(Motor.RunMode.RawPower);
        frontShooterMotor.set(power);
        backShooterMotor.set(power);
    }

    /**
     * Run the shooter based on velocity in RPM.
     * @param velocity (int) The RPM both flywheels are set to.
     */
    public void runShooterVelocity(int velocity) {
        frontShooterMotor.setRunMode(Motor.RunMode.VelocityControl);
        backShooterMotor.setRunMode(Motor.RunMode.VelocityControl);
        frontShooterMotorTargetVelocity = velocity;
        backShooterMotorTargetVelocity = velocity;
        double encoderVelocity = rpmToTicksPerSec(velocity);
        frontShooterMotor.set(encoderVelocity);
        backShooterMotor.set(encoderVelocity);
    }

    public void stop() {
        frontShooterMotor.set(0);
        backShooterMotor.set(0);
    }

    public void feedToShooter() {

    }

    public boolean isReadyToShoot() {
        if(
                ticksPerSecToRpm(frontShooterMotor.getVelocity()) >= frontShooterMotorTargetVelocity-10 &&
                ticksPerSecToRpm(backShooterMotor.getVelocity()) >= backShooterMotorTargetVelocity-10
        ) {
            return true;
        }
        return false;
    }

    private double rpmToTicksPerSec(int rpm) {
        return (rpm*ShooterConstants.shooterMotorTicksPerRev) / 60.0;
    }

    private double ticksPerSecToRpm(double ticksPerSec) {
        return (ticksPerSec*60) / ShooterConstants.shooterMotorTicksPerRev;
    }

}
