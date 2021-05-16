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
    private final WPI_TalonSRX m_steeringMotor;

    private final CANPIDController m_pidController; 

    /**
     * Constructs a SwerveModule.
     *
     * @param driveMotorChannel   ID for the drive motor.
     * @param steeringMotorChannel ID for the steering motor.
     */
    public SwerveModule(int driveMotorCANID, int steeringMotorCANID, boolean invertDrive, boolean invertSteering, boolean steeringEncoderReversed) {

        m_driveMotor = new CANSparkMax(driveMotorCANID, MotorType.kBrushless); // <----- SUPER IMPORTANT BRUSHLESS FOR NEOs
        m_steeringMotor = new WPI_TalonSRX(steeringMotorCANID);

        revError("Set IdleMode to Coast", m_driveMotor.setIdleMode(IdleMode.kCoast));
        m_steeringMotor.setNeutralMode(NeutralMode.Coast); 

        // Config Drive motor (neo+spark max)
        revError("Factory Defaults", m_driveMotor.restoreFactoryDefaults());

        m_pidController = m_driveMotor.getPIDController();

        // set PID coefficients
        revError("Config P", m_pidController.setP(Config.ModuleConstants.kDriveP));
        revError("Config I", m_pidController.setI(Config.ModuleConstants.kDriveI));
        revError("Config D", m_pidController.setD(Config.ModuleConstants.kDriveD));
        revError("Config Izone", m_pidController.setIZone(Config.ModuleConstants.kDriveIz));
        revError("Config FF", m_pidController.setFF(Config.ModuleConstants.kDriveF));

        m_driveMotor.setInverted(invertDrive);


        // Config Steering motor (minicim+talon)
        ctreError("Config Encoder", m_steeringMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, Config.CanConstants.TIMEOUT_SHORT));

        m_steeringMotor.setInverted(invertSteering);
        m_steeringMotor.setSensorPhase(steeringEncoderReversed);

        TalonSRXConfiguration talonConfig = new TalonSRXConfiguration();

        talonConfig.slot0.kP = Config.ModuleConstants.kSteeringP;
        talonConfig.slot0.kI = Config.ModuleConstants.kSteeringI;
        talonConfig.slot0.kD = Config.ModuleConstants.kSteeringD;
        talonConfig.slot0.integralZone = Config.ModuleConstants.kSteeringIz;
        talonConfig.slot0.kF = Config.ModuleConstants.kSteeringF;
        talonConfig.slot0.allowableClosedloopError = Config.ModuleConstants.kSteeringAllowableError;

        talonConfig.feedbackNotContinuous = Config.ModuleConstants.kSteeringEncoderWrap;

        talonConfig.motionCruiseVelocity = Config.ModuleConstants.kSteeringMMVel;
        talonConfig.motionAcceleration = Config.ModuleConstants.kSteeringMMAccel;
        talonConfig.motionCurveStrength = Config.ModuleConstants.kSteeringMMCurve;

        ctreError("ConfigAllSettings", m_steeringMotor.configAllSettings(talonConfig));
    
        

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

        m_driveMotor.getPIDController().setReference(state.speedMetersPerSecond, ControlType.kVelocity);

        m_steeringMotor.set(ControlMode.MotionMagic, convertAngleToCTREUnits(state.angle));
    }

    /** Zero steering encoder (ONLY USE ONCE TO SET ABSOLUTE VALUE OF ENCODER) */
    public void resetDriveEncoder() {
        m_steeringMotor.setSelectedSensorPosition(0);
    }

    /** Get the current velocity of the drive wheel */
    private double getDriveVelocity() {
        double kMetersPerRev = Config.ModuleConstants.kWheelDiameterMeters * Math.PI;

        double neoRPM = m_driveMotor.getEncoder().getVelocity();
        double wheelRPM = neoRPM / Config.ModuleConstants.kDriveGearing;
        double metersPerSecond = wheelRPM * kMetersPerRev / 60;

        return metersPerSecond;
    }

    /** Convert angle (Rotation2d) to  CTRE SRX mag encoder units */
    private double convertAngleToCTREUnits(Rotation2d angle) {
        return (angle.getRadians() / (Math.PI*2)) * Config.HardwareConstants.SRXEncoder_CPR;
    }

    /** Get the current angle of the steering of the swerve module */
    private Rotation2d getSteeringAngle() {
        return new Rotation2d((m_steeringMotor.getSelectedSensorPosition() / Config.HardwareConstants.SRXEncoder_CPR) * Math.PI*2);
    }



    private void revError(String tag, CANError e) {

    }

    private void ctreError(String tag, ErrorCode e) { 

    }



}
