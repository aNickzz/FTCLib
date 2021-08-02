package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SwerveModule {

    private MotorEx driveMotor;
    private CRServo azimuthServo;
    private SwerveModuleState moduleState;

    public SwerveModule(MotorEx driveMotor, CRServo azimuthServo) {
        this.driveMotor = driveMotor;
        this.azimuthServo = azimuthServo;
    }

    public void updateState(SwerveModuleState moduleState) {
        this.moduleState = moduleState;
        runModuleState();
    }

    private double getAzimuthAngle() {
        double val = 0.0;
        return val;
    }

    private void runModuleState() {
        driveMotor.set(moduleState.speedMetersPerSecond);

        if(moduleState.angle.getDegrees()+1.5 < getAzimuthAngle()) {
            azimuthServo.set(-0.8);
        } else if(moduleState.angle.getDegrees()-1.5 > getAzimuthAngle()) {
            azimuthServo.set(0.8);
        }
    }
}
