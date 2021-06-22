// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Config;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;


public class SwerveModule {
    private final CANSparkMax m_driveMotor; 
    private final CANSparkMax m_steeringMotor;

    private final CANPIDController m_drivePidController; 
    private final CANPIDController m_steeringPidController;

    /**
     * Constructs a SwerveModule.
     *
     * @param driveMotorChannel   ID for the drive motor.
     * @param steeringMotorChannel ID for the steering motor.
     */
    public SwerveModule(int driveMotorCANID, int steeringMotorCANID, boolean invertDrive, boolean invertSteering, boolean steeringEncoderReversed) {

        m_driveMotor = new CANSparkMax(driveMotorCANID, MotorType.kBrushless); // <----- SUPER IMPORTANT BRUSHLESS FOR NEOs
        m_steeringMotor = new CANSparkMax(steeringMotorCANID, MotorType.kBrushless);

        revError("Set IdleMode to Coast", m_driveMotor.setIdleMode(IdleMode.kCoast));
        revError("Set IdleMode to Coast", m_steeringMotor.setIdleMode(IdleMode.kCoast));

        // Config Drive motor (neo+spark max)
        revError("Factory Defaults", m_driveMotor.restoreFactoryDefaults());
        revError("Factory Defaults", m_steeringMotor.restoreFactoryDefaults());

        m_drivePidController = m_driveMotor.getPIDController();

        // set PID coefficients
        revError("Config P", m_drivePidController.setP(Config.ModuleConstants.kDriveP));
        revError("Config I", m_drivePidController.setI(Config.ModuleConstants.kDriveI));
        revError("Config D", m_drivePidController.setD(Config.ModuleConstants.kDriveD));
        revError("Config Izone", m_drivePidController.setIZone(Config.ModuleConstants.kDriveIz));
        revError("Config FF", m_drivePidController.setFF(Config.ModuleConstants.kDriveF));

        m_driveMotor.setInverted(invertDrive);


        // Config Steering motor (minicim+talon)
        m_steeringPidController = m_driveMotor.getPIDController();

        // set PID coefficients
        revError("Config P", m_steeringPidController.setP(Config.ModuleConstants.kDriveP));
        revError("Config I", m_steeringPidController.setI(Config.ModuleConstants.kDriveI));
        revError("Config D", m_steeringPidController.setD(Config.ModuleConstants.kDriveD));
        revError("Config Izone", m_steeringPidController.setIZone(Config.ModuleConstants.kDriveIz));
        revError("Config FF", m_steeringPidController.setFF(Config.ModuleConstants.kDriveF));
    
        

    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getSteeringAngle());
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, getSteeringAngle());

        // Set Velocity of wheel
        double RPM = state.speedMetersPerSecond * 1; // CONVERT m/s to RPM
        m_driveMotor.getPIDController().setReference(RPM, ControlType.kVelocity);


        // Calculate pos of steering motor and set
        double currentPos = getSteeringPosition();

        double currentAngle = (currentPos / Config.ModuleConstants.kSteerGearing) * Math.PI * 2;
        Rotation2d currentPrincipleAngle = new Rotation2d(currentAngle);

        Rotation2d angleError = desiredState.angle.minus(currentPrincipleAngle);

        double posError = (angleError.getRadians() / (Math.PI * 2)) * Config.ModuleConstants.kSteerGearing;

        double desiredPos = currentPos + posError;


        m_steeringPidController.setReference(desiredPos, ControlType.kSmartMotion);

    }

    /** Set the angle of the steering motor*/
    public void setSteeringEncoder(Rotation2d angle) {
        double pos = (angle.getRadians()/(Math.PI*2)) * Config.ModuleConstants.kSteerGearing;
        m_steeringMotor.getEncoder().setPosition(pos);
    }

    /** Get the current velocity of the drive wheel */
    private double getDriveVelocity() {
        double kMetersPerRev = Config.ModuleConstants.kWheelDiameterMeters * Math.PI;

        double neoRPM = m_driveMotor.getEncoder().getVelocity();
        double wheelRPM = neoRPM / Config.ModuleConstants.kDriveGearing;
        double metersPerSecond = wheelRPM * kMetersPerRev / 60;

        return metersPerSecond;
    }


    /** Get the current angle of the steering of the swerve module */
    private Rotation2d getSteeringAngle() {
        return new Rotation2d((m_steeringMotor.getEncoder().getPosition() * Config.ModuleConstants.kSteerGearing) * Math.PI*2);
    }

    /** Get the current encoder position of the steering of the swerve module */
    private double getSteeringPosition() {
        return m_steeringMotor.getEncoder().getPosition();
    }

    private void revError(String action, CANError e) {

    }

}
