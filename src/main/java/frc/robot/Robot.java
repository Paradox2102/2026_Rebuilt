// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private PowerDistribution m_pd = new PowerDistribution(1, ModuleType.kRev);
  private StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault().getStructTopic("Robot Pose", Pose2d.struct).publish();
  private StructPublisher<Pose2d> futurePosePublisher = NetworkTableInstance.getDefault().getStructTopic("Est Future Robot Pose", Pose2d.struct).publish();
  private StructPublisher<Pose2d> alignPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Align Pose", Pose2d.struct).publish();
  // private StructArrayPublisher<Pose3d> zeroedComponentPoses = NetworkTableInstance.getDefault()
  // .getStructArrayTopic("Zeroed Poses", Pose3d.struct).publish();
  // private StructArrayPublisher<Pose3d> finalComponentPoses = NetworkTableInstance.getDefault()
  // .getStructArrayTopic("Component Poses", Pose3d.struct).publish();

  public Robot() {
    m_robotContainer = new RobotContainer();
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    posePublisher.set(m_robotContainer.m_swerveSubsystem.getPose());
    futurePosePublisher.set(m_robotContainer.m_swerveSubsystem.sotmLookAhead(3));
    alignPosePublisher.set(new Pose2d(m_robotContainer.m_swerveSubsystem.getPose().getTranslation(), Rotation2d.fromDegrees(m_robotContainer.m_swerveSubsystem.getHubAngle())));
    SmartDashboard.putBoolean("Align On", m_robotContainer.shouldAutoAlign.getAsBoolean());
    SmartDashboard.putNumber("current", m_pd.getTotalCurrent());
    SmartDashboard.putNumber("bl current (6)", m_pd.getCurrent(6));
    SmartDashboard.putNumber("fl current (15)", m_pd.getCurrent(15));
    SmartDashboard.putNumber("br current (9)", m_pd.getCurrent(9));
    SmartDashboard.putNumber("fr current (1)", m_pd.getCurrent(1));
    SmartDashboard.putNumber("current 0", m_pd.getCurrent(0));
    SmartDashboard.putNumber("current 2", m_pd.getCurrent(2));
    SmartDashboard.putNumber("current 3", m_pd.getCurrent(3));
    SmartDashboard.putNumber("current 4", m_pd.getCurrent(4));
    SmartDashboard.putNumber("current 5", m_pd.getCurrent(5));
    SmartDashboard.putNumber("current 7", m_pd.getCurrent(7));
    SmartDashboard.putNumber("current 8", m_pd.getCurrent(8));
    SmartDashboard.putNumber("current 10", m_pd.getCurrent(10));
    SmartDashboard.putNumber("current 11", m_pd.getCurrent(11));
    SmartDashboard.putNumber("current 12", m_pd.getCurrent(12));
    SmartDashboard.putNumber("current 13", m_pd.getCurrent(13));
    SmartDashboard.putNumber("current 14", m_pd.getCurrent(14));
    SmartDashboard.putNumber("current 16", m_pd.getCurrent(16));
    SmartDashboard.putNumber("current 17", m_pd.getCurrent(17));
    SmartDashboard.putNumber("current 18", m_pd.getCurrent(18));
    SmartDashboard.putNumber("current 19", m_pd.getCurrent(19));
    SmartDashboard.putNumber("current 20", m_pd.getCurrent(20));
    SmartDashboard.putNumber("volts", m_pd.getVoltage());
    SmartDashboard.putBoolean("brownout", RobotController.isBrownedOut());
    SmartDashboard.putNumber("canbus utilization", RobotController.getCANStatus().percentBusUtilization);
    SmartDashboard.putNumber("roborio brownout volatage", RobotController.getBrownoutVoltage());
    // zeroedComponentPoses.set(new Pose3d[] {new Pose3d(), new Pose3d(), new Pose3d()});
    // finalComponentPoses.set(new Pose3d[] {
    //   new Pose3d(-0.184, 0.0, 0.158, new Rotation3d(0, Math.toRadians(m_robotContainer.m_pivotSubsystem.getPosition()) - IntakeConstants.k_pivotMaxRotation,0)),
    //   new Pose3d(0.298,0, 0.488, new Rotation3d(0, Math.toRadians(m_robotContainer.m_hoodSubsystem.getHoodAngle()),0)),
    //   new Pose3d(Math.cos(0.814527)*m_robotContainer.m_climberSubsystem.getHeight(), 0, Math.sin(0.814527)*m_robotContainer.m_climberSubsystem.getHeight(), new Rotation3d(0,0,0))});
    // SmartDashboard.putNumber("fuel sim fuel amount", m_robotContainer.m_fuelLaunchSim.getFuelStored());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }

    m_robotContainer.switchAuto(true);
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.switchAuto(false);
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {
    FuelSim.getInstance().updateSim();
  }
}
