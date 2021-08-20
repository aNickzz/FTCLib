package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import org.firstinspires.ftc.teamcode.Constants.DrivetrainConstants;


public class SwerveModule {

    public MotorEx driveMotor;
    public CRServo azimuthServo;
    public Motor.Encoder azimuthEncoder;
    private SwerveModuleState moduleState;

    public SwerveModule(MotorEx driveMotor, CRServo azimuthServo) {
        this.driveMotor = driveMotor;
        this.azimuthServo = azimuthServo;
        this.azimuthEncoder = driveMotor.encoder;
        this.azimuthEncoder.setDirection(Motor.Direction.REVERSE);
    }

    public void updateState(SwerveModuleState moduleState) {
        this.moduleState = moduleState;
        runModuleState();
    }

    public Rotation2d getAzimuthAngle() {
        // get the value from the encoder
        int moduleLocation = azimuthEncoder.getPosition() % DrivetrainConstants.ticksPerModuleRotation;

        double moduleDegrees = (360.0 / (double)DrivetrainConstants.ticksPerModuleRotation) * (double)moduleLocation;


        return Rotation2d.fromDegrees(moduleDegrees);
    }

    public double getStateAngle() {
        return moduleState.angle.getDegrees();
    }

    public double calculatePController() {
        PController ctrl = new PController(0.01);

        double currentDegrees = getAzimuthAngle().getDegrees();
        double targetDegrees = moduleState.angle.getDegrees();

        if (currentDegrees - targetDegrees > 180) {
            targetDegrees += 360;
        }

        if (currentDegrees - targetDegrees < -180) {
            targetDegrees -= 360;
        }


        double ctrlResult = ctrl.calculate(currentDegrees, targetDegrees);

        return ctrlResult;
    }

    private void runModuleState() {
        // set the drive motor to drive
        driveMotor.set(moduleState.speedMetersPerSecond);

        azimuthServo.set(calculatePController());
    }

    private void runModuleStateOptimised() {
        double speed = moduleState.speedMetersPerSecond;
        double targetAngle = moduleState.angle.getDegrees();

        // find the optimised angle to rotate the module
        if(
                Math.abs(moduleState.angle.getDegrees() - getAzimuthAngle().getDegrees()) > 90 &&
                Math.abs(moduleState.angle.getDegrees() - getAzimuthAngle().getDegrees()) < 270
        ) {
            targetAngle = ((int)targetAngle + 180) % 360;
            speed = -speed;
        }

        // rotate the module
        if(targetAngle+1.5 < getAzimuthAngle().getDegrees()) {
            // rotate the module ccw
            azimuthServo.set(-0.8);
        } else if(targetAngle-1.5 > getAzimuthAngle().getDegrees()) {
            // rotate the module cw
            azimuthServo.set(0.8);
        }

        driveMotor.set(speed);
    }

}
