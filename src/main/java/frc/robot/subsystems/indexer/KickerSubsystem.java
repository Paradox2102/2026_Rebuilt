// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class KickerSubsystem extends SubsystemBase {
  private SparkFlex m_kickerMotor = new SparkFlex(Constants.CANIDConstants.kicker, MotorType.kBrushless);
  private SparkClosedLoopController m_pid = m_kickerMotor.getClosedLoopController();
  private RelativeEncoder m_encoder = m_kickerMotor.getEncoder();

  private FlywheelSim m_kickerSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNeoVortex(1), Constants.IndexerConstants.k_kickerMOI, Constants.IndexerConstants.k_kickerReduction), DCMotor.getNeoVortex(1));
  private SparkSim m_kickerMotorSim = new SparkSim(m_kickerMotor, DCMotor.getNeoVortex(1));
  
  private double m_simVelocity = 0;

  /** Creates a new kickerSubsystem. */
  public KickerSubsystem() {
    m_kickerMotor.configure(Constants.IndexerConstants.k_kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command run(boolean in){
    return Commands.runEnd(() -> {
      m_pid.setSetpoint(in ? Constants.IndexerConstants.k_kickerInSpeed : Constants.IndexerConstants.k_kickerOutSpeed , ControlType.kVelocity);
    }, () -> {
      m_pid.setSetpoint(0, ControlType.kVelocity);
    }, this);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Flywheel Velocity", RobotBase.isReal() ? m_encoder.getVelocity() : m_simVelocity);
  }

  public void simulationPeriodic() {
    m_kickerSim.setInput(m_kickerMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    m_kickerSim.update(0.02);
    m_kickerMotorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(m_kickerSim.getAngularVelocityRadPerSec()),
        RoboRioSim.getVInVoltage(),0.02);
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_kickerSim.getCurrentDrawAmps()));
    m_simVelocity = Units.radiansPerSecondToRotationsPerMinute(m_kickerSim.getAngularVelocityRadPerSec());
  }
}
