package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Constants.DrivetrainConstants;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SwerveModule;

public class Drivetrain extends SubsystemBase {

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private SwerveDriveKinematics swerveKinematics;
    private RevIMU gyro;
    private SwerveModule leftModule;
    private SwerveModule rightModule;

    public Drivetrain(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gyro = new RevIMU(hardwareMap);
        leftModule = new SwerveModule(
                new MotorEx(hardwareMap, DrivetrainConstants.leftDriveMotorName),
                new CRServo(hardwareMap, DrivetrainConstants.leftAzimuthServoName)
        );
        rightModule = new SwerveModule(
                new MotorEx(hardwareMap, DrivetrainConstants.rightDriveMotorName),
                new CRServo(hardwareMap, DrivetrainConstants.rightAzimuthServoName)
        );
        swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(-0.3, 0), //left module location relative to robot center
                new Translation2d(0.3,0)    //right module location
        );
    }

    public double getAngle() {
        return gyro.getRotation2d().getDegrees();
    }

    public void drive(double x, double y, double rotation, boolean fieldOriented, boolean squareInputs) {

        if(squareInputs) {
            x = Math.pow(x, 2) * Math.signum(x);
            y = Math.pow(y, 2) * Math.signum(y);
            rotation = Math.pow(rotation, 2) * Math.signum(rotation);
        }

        ChassisSpeeds speeds;
        // pi is half a radian/180 degrees
        if(fieldOriented) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rotation * Math.PI, Rotation2d.fromDegrees(getAngle()));
        } else {
            speeds = new ChassisSpeeds(x, y, rotation * Math.PI);
        }

        SwerveModuleState[] moduleStates = swerveKinematics.toSwerveModuleStates(speeds);
        leftModule.updateState(moduleStates[0]);
        rightModule.updateState(moduleStates[1]);
    }

}
