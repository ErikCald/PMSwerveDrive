// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Config {
    public static final class DriveConstants {

        public static final boolean kFrontLeftDriveReversed = false;
        public static final boolean kRearLeftDriveReversed = true;
        public static final boolean kFrontRightDriveReversed = false;
        public static final boolean kRearRightDriveReversed = true;

        public static final boolean kFrontLeftSteeringReversed = false;
        public static final boolean kRearLeftSteeringReversed = true;
        public static final boolean kFrontRightSteeringReversed = false;
        public static final boolean kRearRightSteeringReversed = true;

        public static final boolean kFrontLeftSteeringEncoderReversed = false;
        public static final boolean kRearLeftSteeringEncoderReversed = true;
        public static final boolean kFrontRightSteeringEncoderReversed = false;
        public static final boolean kRearRightSteeringEncoderReversed = true;
        
        public static final double kTrackWidth = 0.5;
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = 0.7;
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2), 
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final boolean kGyroReversed = false;

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or
        // theoretically
        // for *your* robot's drive.
        // The RobotPy Characterization Toolsuite provides a convenient tool for
        // obtaining these
        // values for your robot.
        public static final double ksVolts = 1;
        public static final double kvVoltSecondsPerMeter = 0.8;
        public static final double kaVoltSecondsSquaredPerMeter = 0.15;

        public static final double kMaxSpeedMetersPerSecond = 3;
    }

    public static final class ModuleConstants {

        public static final int kEncoderCPR = 4096;
        public static final double kWheelDiameterMeters = 0.1015;

        public static final double kDriveGearing = 0;

        public static final double kSteeringP = 0;
        public static final double kSteeringI = 0;
        public static final double kSteeringD = 0;
        public static final double kSteeringIz = 0;
        public static final double kSteeringF = 0;
        public static final double kSteeringAllowableError = 0;
        public static final boolean kSteeringFeedbackNotContinuous = true;
        public static final double kSteeringMMVel = 0;
        public static final double kSteeringMMAccel = 0;
        public static final int kSteeringMMCurve = 0;

        public static final double kDriveP = 0;
        public static final double kDriveI = 0;
        public static final double kDriveD = 0;
        public static final double kDriveIz = 0;
        public static final double kDriveF = 0;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 1;
    }

    public static final class AutoConstants {
        // Necessary for SwerveControllerCommand
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Traj Config for generating trajectories in code
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;

        // 
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class HardwareConstants {
        public static final int SRXEncoder_CPR = 4096;
    }

    public static final class CanConstants {
        public static final int kFrontLeftDriveCanID = 0;
        public static final int kRearLeftDriveCanID = 2;
        public static final int kFrontRightDriveCanID = 4;
        public static final int kRearRightDriveCanID = 6;

        public static final int kFrontLeftSteeringCanID = 1;
        public static final int kRearLeftSteeringCanID = 3;
        public static final int kFrontRightSteeringCanID = 5;
        public static final int kRearRightSteeringCanID = 7;

        public static final int kPigeonCanID = 15;

        public static final int TIMEOUT_SHORT = 10;
        public static final int TIMEOUT_LONG = 100;
    }
}
