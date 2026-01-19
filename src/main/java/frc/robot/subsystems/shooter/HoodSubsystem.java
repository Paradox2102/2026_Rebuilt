// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.drive.SwerveSubsystem;

public class HoodSubsystem extends SubsystemBase {
  private final SparkFlex m_hoodMotor = new SparkFlex(CANIDConstants.shooter_hood, MotorType.kBrushless);
  private final AbsoluteEncoder m_hoodEncoder = m_hoodMotor.getAbsoluteEncoder();
  private SparkClosedLoopController m_pid = m_hoodMotor.getClosedLoopController();

  private final double hoodAngleDeadzone = 0.1;

  private final InterpolatingDoubleTreeMap m_hoodLerpTable = new InterpolatingDoubleTreeMap();
  
  private final SwerveSubsystem m_driveSubsystem;
  public HoodSubsystem(SwerveSubsystem driveSubsystem) {
    m_hoodMotor.configure(ShooterConstants.k_hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_hoodLerpTable.put(0.0, 0.0); // add values later
    m_driveSubsystem = driveSubsystem;
  }

  @Override
  public void periodic() {
    
  }
  public final class pitchHoodCommand extends Command {
    private final Timer m_timer = new Timer();
    // get dimensions later.
    private Translation2d goalTranslation = new Translation2d();
    private double distanceToGoal = m_driveSubsystem.getPose().getTranslation().getDistance(goalTranslation);
    private final double m_endTime = 0.1;
    public pitchHoodCommand(){
      m_timer.start();
    }
    @Override
    public void execute() {
      m_pid.setSetpoint(m_hoodLerpTable.get(distanceToGoal), ControlType.kPosition);
      if (Math.abs(m_hoodEncoder.getPosition() - m_hoodLerpTable.get(distanceToGoal)) > hoodAngleDeadzone){
        m_timer.reset();
      }
    }
    @Override
    public boolean isFinished() {
        return m_timer.get() > m_endTime;
    }
    @Override
    public void end(boolean interrupted) {
      m_hoodMotor.set(0.0);
    }
  }
}
