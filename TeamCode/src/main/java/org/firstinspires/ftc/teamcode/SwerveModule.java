package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import org.firstinspires.ftc.teamcode.Constants.DrivetrainConstants;


public class SwerveModule {

    private MotorEx driveMotor;
    private CRServo azimuthServo;
    private Motor.Encoder azimuthEncoder;
    private SwerveModuleState moduleState;

    public SwerveModule(MotorEx driveMotor, CRServo azimuthServo) {
        this.driveMotor = driveMotor;
        this.azimuthServo = azimuthServo;
        this.azimuthEncoder = driveMotor.encoder;
    }

    public void updateState(SwerveModuleState moduleState) {
        this.moduleState = moduleState;
        runModuleState();
    }

    private Rotation2d getAzimuthAngle() {
        // get the value from the encoder
        int moduleLocation = azimuthEncoder.getPosition() % DrivetrainConstants.ticksPerModuleRotation;
        double moduleDegrees = (360.0 / (double)DrivetrainConstants.ticksPerModuleRotation) * (double)moduleLocation;
        return Rotation2d.fromDegrees(moduleDegrees);
    }

    private void runModuleState() {
        // set the drive motor to drive
        driveMotor.set(moduleState.speedMetersPerSecond);

        if(moduleState.angle.getDegrees()+1.5 < getAzimuthAngle().getDegrees()) {
            // rotate the module ccw
            azimuthServo.set(-0.8);
        } else if(moduleState.angle.getDegrees()-1.5 > getAzimuthAngle().getDegrees()) {
            // rotate the module cw
            azimuthServo.set(0.8);
        }
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
