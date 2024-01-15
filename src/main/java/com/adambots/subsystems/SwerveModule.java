// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.Constants.DriveConstants;
import com.adambots.Constants.DriveConstants.ModulePosition;
import com.adambots.Constants.ModuleConstants;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class SwerveModule {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final CANcoder m_absoluteEncoder;
  private final RelativeEncoder m_driveEncoder;
  // private final CANCoderConfiguration m_canCoderConfig = new CANCoderConfiguration();

  private final PIDController m_drivePIDController =
      new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          ModuleConstants.kPModuleTurningController,
          ModuleConstants.kIModuleTurningController,
          ModuleConstants.kDModuleTurningController,
          new TrapezoidProfile.Constraints(
              ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
              ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  private ModulePosition m_position;

  public void setPIDValues(double kP, double kI, double kD) {
    m_turningPIDController.setP(kP);
    m_turningPIDController.setI(kI);
    m_turningPIDController.setD(kD);
  }

  /**
   * Constructs a SwerveModule.
   *
   * @param position The position of this module (front or back, right or left)
   * @param driveMotorChannel The channel of the drive motor.
   * @param turningMotorChannel The channel of the turning motor.
   * @param driveEncoderChannels The channels of the drive encoder.
   * @param turningEncoderChannels The channels of the turning encoder.
   * @param driveEncoderReversed Whether the drive encoder is reversed.
   * @param turningEncoderReversed Whether the turning encoder is reversed.
   */
  public SwerveModule(
      ModulePosition position,
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel,
      boolean driveEncoderReversed,
      boolean turningEncoderReversed) {
    
    this.m_position = position; // Use position.name() to get the name of the position as a String
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
    m_driveMotor.setIdleMode(IdleMode.kBrake);
    m_turningMotor.setIdleMode(IdleMode.kBrake);

    m_driveMotor.setSmartCurrentLimit(32); //32
    m_turningMotor.setSmartCurrentLimit(21); //25 - 15
    m_driveMotor.enableVoltageCompensation(12.6);
    m_turningMotor.enableVoltageCompensation(12.6);
    // m_turningMotor.setOpenLoopRampRate(0.1);

    m_absoluteEncoder = new CANcoder(turningEncoderChannel);
    m_driveEncoder = m_driveMotor.getEncoder();

    //TODO: Utilize driveEncoder and turningEncoder Reversed flags - instead of negating Joystick values in RobotContainer
    // m_driveMotor.setInverted(driveEncoderReversed);
    
    // m_encoder.configAllSettings(m_canCoderConfig);
    m_absoluteEncoder.clearStickyFaults();
    m_driveMotor.clearFaults();
    m_turningMotor.clearFaults();

    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    resetEncoders();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    double speedMetersPerSecond = m_driveEncoder.getVelocity() / 60.0; //TODO: Fix this complete lack of sensible units
    double turningRadians = Units.degreesToRadians(m_absoluteEncoder.getAbsolutePosition().getValueAsDouble());
    return new SwerveModuleState(speedMetersPerSecond, new Rotation2d(turningRadians));
  }

  /**
   * Sets the desired state for the module.
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    if (Math.abs(desiredState.speedMetersPerSecond) < DriveConstants.kSpeedThreshold){
      stop();
      return;
    }

    double speedMetersPerSecond = ModuleConstants.kDriveEncoderDistancePerPulse * m_driveEncoder.getVelocity(); //TODO: Fix this complete lack of sensible units
    double turningRadians = Units.degreesToRadians(m_absoluteEncoder.getAbsolutePosition().getValueAsDouble());

    // System.out.printf("Speed: %f, Turn: %f\n", speedMetersPerSecond, turningRadians);
    // System.out.printf("Absolute Encoder: %f\n", m_absoluteEncoder.getAbsolutePosition());
    
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(turningRadians));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(speedMetersPerSecond, state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(turningRadians, state.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.

    m_driveMotor.set(driveOutput);
    m_turningMotor.set(turnOutput);

    // SmartDashboard.putNumber("device id" + m_driveMotor.getDeviceId(), m_absoluteEncoder.getAbsolutePosition());
    // SmartDashboard.putNumber("device id" + m_driveMotor.getDeviceId(), driveOutput);

    // If using motor's PID Controller, then enable this instead of the set above.
    // m_drivePIDController.setReference(state.speedMetersPerSecond, ControlType.kVelocity, 1);

    // System.out.printf("Drive Output: %f\n", driveOutput);
    // System.out.printf("Turn Output: %f\n", turnOutput);
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    double distance = m_driveEncoder.getPosition() * ModuleConstants.kDriveEncoderScale;
    double turningDistance = Units.degreesToRadians(m_absoluteEncoder.getAbsolutePosition().getValueAsDouble());
    
    // System.out.printf("Distance: %f | Turn: %f \n", m_driveEncoder.getPosition(), turningDistance);
    // SmartDashboard.putNumber("Turningdistance " + m_position.name(), m_turningEncoder.getPosition());

    return new SwerveModulePosition(
        distance, new Rotation2d(turningDistance));
  }

  /**
   * Turn this module wheel by this angle in degrees
   * @param angle in degrees
   */
  public void turn(double angle, double turningSpeed) { //TODO: Test and get this function working

    double currentModuleAngleInRadians = Units.degreesToRadians(m_absoluteEncoder.getAbsolutePosition().getValueAsDouble()); 
    var radAngle = Units.degreesToRadians(angle);
    double turnAngleError = Math.abs(radAngle - currentModuleAngleInRadians);
    
    var swerveModuleState = new SwerveModuleState(turningSpeed, new Rotation2d(radAngle));
    var desiredState = SwerveModuleState.optimize(swerveModuleState, new Rotation2d(currentModuleAngleInRadians));
    // m_turningPIDController.reset(currentModuleAngleInRadians);
    double pidOut = m_turningPIDController.calculate(currentModuleAngleInRadians, desiredState.angle.getRadians());

    // if robot is not moving, stop the turn motor oscillating
    if (turnAngleError < Units.degreesToRadians(0.5) && Math.abs(getState().speedMetersPerSecond) <= DriveConstants.kSpeedThreshold) {
      pidOut = 0;
    }

    m_turningMotor.set(pidOut);
  }

  public void stop(){
    m_driveMotor.set(0);
    m_turningMotor.set(0);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
  }
}