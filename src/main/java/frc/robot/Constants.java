package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;

public class Constants {
        // Both sets of gains need to be tuned to your individual robot.

        // The steer motor uses any SwerveModule.SteerRequestType control request with
        // the
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        public static final Slot0Configs steerGains = new Slot0Configs()
                        .withKP(100).withKI(0).withKD(0.2)
                        .withKS(0.13973).withKV(2.58862).withKA(0);
        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        public static final Slot0Configs driveGains = new Slot0Configs()
                        .withKP(3).withKI(0).withKD(0)
                        .withKS(0.14774).withKV(0.11330).withKA(0);

        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        public static final double kSlipCurrentA = 75;

        // Theoretical free speed (m/s) at 12v applied output;
        // This needs to be tuned to your individual robot
        public static final double kSpeedAt12VoltsMps = 5.21;

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        // This may need to be tuned to your individual robot
        public static final double kCoupleRatio = 3.5714285714285716;

        public static final double kDriveGearRatio = 6.122448979591837;
        public static final double kSteerGearRatio = 21.428571428571427;
        public static final double kWheelRadiusInches = 2;

        public static final boolean kSteerMotorReversed = true;
        public static final boolean kInvertLeftSide = false;
        public static final boolean kInvertRightSide = true;

        public static final int kPigeonId = 0;

        // These are only used for simulation
        public static final double kSteerInertia = 0.00001;
        public static final double kDriveInertia = 0.001;
        // Simulated voltage necessary to overcome friction
        public static final double kSteerFrictionVoltage = 0.25;
        public static final double kDriveFrictionVoltage = 0.25;

        // Front Left
        public static final int kFrontLeftDriveMotorId = 3;
        public static final int kFrontLeftSteerMotorId = 4;
        public static final int kFrontLeftEncoderId = 51;
        public static final double kFrontLeftEncoderOffset = 0.308837890625;

        public static final double kFrontLeftXPosInches = 10.625;
        public static final double kFrontLeftYPosInches = 10.625;

        // Front Right
        public static final int kFrontRightDriveMotorId = 5;
        public static final int kFrontRightSteerMotorId = 6;
        public static final int kFrontRightEncoderId = 52;
        public static final double kFrontRightEncoderOffset = 0.306640625;

        public static final double kFrontRightXPosInches = 10.625;
        public static final double kFrontRightYPosInches = -10.625;

        // Back Left
        public static final int kBackLeftDriveMotorId = 7;
        public static final int kBackLeftSteerMotorId = 8;
        public static final int kBackLeftEncoderId = 54;
        public static final double kBackLeftEncoderOffset = -0.3603515625;

        public static final double kBackLeftXPosInches = -10.625;
        public static final double kBackLeftYPosInches = 10.625;

        // Back Right
        public static final int kBackRightDriveMotorId = 1;
        public static final int kBackRightSteerMotorId = 2;
        public static final int kBackRightEncoderId = 53;
        public static final double kBackRightEncoderOffset = 0.357421875;

        public static final double kBackRightXPosInches = -10.625;
        public static final double kBackRightYPosInches = -10.625;
}