// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import frc.robot.Config.CanConstants;
import frc.robot.Config.DriveConstants;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings("PMD.ExcessiveImports")
public class DriveSubsystem extends SubsystemBase {
    /** Creates a new DriveSubsystem. */
    private DriveSubsystem() {

    }
    private static DriveSubsystem m_driveSubsystemInstance;
    public static DriveSubsystem getInstance() {
        if (m_driveSubsystemInstance != null) {
            m_driveSubsystemInstance = new DriveSubsystem();
        }
        return m_driveSubsystemInstance;
    }

    // Robot swerve modules
    private final SwerveModule m_frontLeft = new SwerveModule(
        CanConstants.kFrontLeftDriveCanID,
        CanConstants.kFrontLeftSteeringCanID, 
        DriveConstants.kFrontLeftDriveReversed,
        DriveConstants.kFrontLeftSteeringEncoderReversed, 
        DriveConstants.kFrontLeftSteeringEncoderReversed);

    private final SwerveModule m_rearLeft = new SwerveModule(
        CanConstants.kRearLeftDriveCanID,
        CanConstants.kRearLeftSteeringCanID, 
        DriveConstants.kRearLeftDriveReversed,
        DriveConstants.kRearLeftSteeringEncoderReversed, 
        DriveConstants.kRearLeftSteeringEncoderReversed);

    private final SwerveModule m_frontRight = new SwerveModule(
        CanConstants.kFrontRightDriveCanID,
        CanConstants.kFrontRightSteeringCanID, 
        DriveConstants.kFrontRightDriveReversed,
        DriveConstants.kFrontRightSteeringEncoderReversed, 
        DriveConstants.kFrontRightSteeringEncoderReversed);

    private final SwerveModule m_rearRight = new SwerveModule(
        CanConstants.kRearRightDriveCanID,
        CanConstants.kRearRightSteeringCanID, 
        DriveConstants.kRearRightDriveReversed,
        DriveConstants.kRearRightSteeringEncoderReversed, 
        DriveConstants.kRearRightSteeringEncoderReversed);

    // The gyro sensor
    private final PigeonIMU m_gyro = new PigeonIMU(CanConstants.kPigeonCanID);

    // Odometry class for tracking robot pose
    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, Rotation2d.fromDegrees(m_gyro.getFusedHeading()));

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        m_odometry.update(Rotation2d.fromDegrees(m_gyro.getFusedHeading()), m_frontLeft.getState(), m_rearLeft.getState(),
                m_frontRight.getState(), m_rearRight.getState());
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(m_gyro.getFusedHeading()));
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }


    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        m_gyro.setFusedHeading(0);
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public Rotation2d getHeading() {
        return m_odometry.getPoseMeters().getRotation();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        double[] xyz_dps = new double[3];
        m_gyro.getRawGyro(xyz_dps);

        return xyz_dps[2];
    }
}
