// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.adambots.Constants;
import com.adambots.Constants.AutoConstants;
import com.adambots.Constants.DriveConstants;
import com.adambots.Constants.DriveConstants.ModulePosition;
import com.adambots.utils.ModuleMap;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {
  // The gyro sensor
  private final Gyro m_gyro;

  // Odometry class for tracking robot pose
  private SwerveDriveOdometry m_odometry;

  // Field details that can be viewed in Glass
  HashMap<ModulePosition, SwerveModule> swerveModules;
  private BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();
  private LinearFilter yLinearFilter = LinearFilter.movingAverage(10);

  private boolean bumped = false;

  public DrivetrainSubsystem(HashMap<ModulePosition, SwerveModule> modules, Gyro gyro) {
    this.swerveModules = modules;
    m_gyro = gyro;

    AutoBuilder.configureHolonomic(
      this::getPose, // Robot pose supplier
      this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      new HolonomicPathFollowerConfig(
          new PIDConstants(AutoConstants.kPXController, 0, AutoConstants.kDXController), 
          new PIDConstants(AutoConstants.kPThetaController, 0, AutoConstants.kDThetaController),
          4.5, // Max module speed, in m/s
          0.43, // Drive base radius in meters. Distance from robot center to furthest module.
          new ReplanningConfig() // Default path replanning config. See the API for the options here
      ), 
      () -> false, //TODO: CREATE PATH FLIP SUPPLIER
      this // Reference to this subsystem to set requirements
  );

    m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, m_gyro.getRotation2d(), ModuleMap.orderedModulePositions(swerveModules));
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(),
        ModuleMap.orderedModulePositions(swerveModules)
    );

    Constants.DriveConstants.field.setRobotPose(getPose());

    double yAcceleration = yLinearFilter.calculate(accelerometer.getY());
    // SmartDashboard.putNumber("Yacceleration", yAcceleration);
    bumped = Math.abs(yAcceleration) > 0.5;
  }

  public Map<ModulePosition, SwerveModuleState> getModuleStates() {
    Map<ModulePosition, SwerveModuleState> map = new HashMap<>();
    for (ModulePosition i : swerveModules.keySet()) {
      map.put(i, swerveModules.get(i).getState());
    }
    return map;
  }

  public SwerveModuleState[] getModuleStatesArray() {
    SwerveModuleState[] arr = new SwerveModuleState[4];
    int j = 0;
    for (ModulePosition i : swerveModules.keySet()) {
      arr[j++] = swerveModules.get(i).getState();
    }
    return arr;
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
    m_odometry.resetPosition(m_gyro.getRotation2d(), ModuleMap.orderedModulePositions(swerveModules), pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed
   *          Speed of the robot in the x direction (forward).
   * @param ySpeed
   *          Speed of the robot in the y direction (sideways).
   * @param rot
   *          Angular rate of the robot.
   * @param fieldRelative
   *          Whether the provided x and y speeds are relative to the
   *          field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    ModuleMap.setDesiredState(swerveModules, swerveModuleStates);

    // swerveModuleStates[0].
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    ModuleMap.setDesiredState(swerveModules, swerveModuleStates);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStatesArray());
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);

    ModuleMap.setDesiredState(swerveModules, desiredStates);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 0 to 360
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
    // return Math.IEEEremainder(m_gyro.getAngle(), 360); // Constrain to 360
    // return MathUtil.inputModulus(m_gyro.getAngle(), -180, 180); // use this if
    // you want to constrain by -180 to 180
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate();
  }

  public void setPIDValues(double kP, double kI, double kD) {
    swerveModules.get(ModulePosition.FRONT_LEFT).setPIDValues(kP, kI, kD);
    swerveModules.get(ModulePosition.FRONT_RIGHT).setPIDValues(kP, kI, kD);
    swerveModules.get(ModulePosition.REAR_LEFT).setPIDValues(kP, kI, kD);
    swerveModules.get(ModulePosition.REAR_RIGHT).setPIDValues(kP, kI, kD);
  }

  public void stop() {
    drive(0, 0, 0, false);
  }

  public boolean getBumped() {
    return bumped;
  }
}