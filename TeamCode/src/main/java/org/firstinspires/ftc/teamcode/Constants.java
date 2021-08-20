package org.firstinspires.ftc.teamcode;

public final class Constants {

    public static final class DrivetrainConstants {
        public static final String leftDriveMotorName = "leftDrive";
        public static final String rightDriveMotorName = "rightDrive";
        public static final String leftAzimuthServoName = "leftAzimuth";
        public static final String rightAzimuthServoName = "rightAzimuth";
        public static final double maxSpeed = 1.0;
        public static final int ticksPerModuleRotation = 4600;
    }

    public static final class ShooterConstants {
        public static final String  frontMotorName = "shooterFront";
        public static final String backMotorName = "shooterBack";
        public static final String feederServoName = "shooterFeed";
        public static final int feederServoMinAngle = 0;
        public static final int feederServoMaxAngle = 180;
        public static final int frontShooterRPM = 60;
        public static final int backShooterRPM = 60;
        public static final double frontShooterPower = 0.3;
        public static final double backShooterPower = 0.3;
        public static final double shooterMotorTicksPerRev = 28.0;
    }

    public static final class IntakeConstants {
        public static final String intakeMotorName = "intake";
        public static final double intakePower = 0.3;
    }

    public static final class MagazineConstants {
        public static final String liftMotorName = "lift";
        public static final double liftPower = 0.2;
    }

    public static final class ControllerDriveConstants {
        /**
         * How far you have to push the thumbstick before the robot will start turning
         */
        public static final double rotationDeadzone = 0.2;
    }
}
