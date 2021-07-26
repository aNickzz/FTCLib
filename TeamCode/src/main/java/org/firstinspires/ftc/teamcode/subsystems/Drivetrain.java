package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drivetrain extends SubsystemBase {

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private SwerveDriveKinematics swerveKinematics;

    public Drivetrain(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(-0.3, 0), //left module location
                new Translation2d(0.3,0)    //right module location
        );

    }

}
