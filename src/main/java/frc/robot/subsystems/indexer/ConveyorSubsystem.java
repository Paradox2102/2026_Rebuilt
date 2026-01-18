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

public class ConveyorSubsystem extends SubsystemBase {
  private SparkFlex m_conveyorMotor = new SparkFlex(Constants.CANIDConstants.conveyor, MotorType.kBrushless);
  private SparkClosedLoopController m_pid = m_conveyorMotor.getClosedLoopController();
  private RelativeEncoder m_encoder = m_conveyorMotor.getEncoder();

  private FlywheelSim m_conveyorSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNeoVortex(1), Constants.IndexerConstants.k_conveyorMOI, Constants.IndexerConstants.k_conveyorReduction), DCMotor.getNeoVortex(1));
  private SparkSim m_conveyorMotorSim = new SparkSim(m_conveyorMotor, DCMotor.getNeoVortex(1));
  
  private double m_simVelocity = 0;

  /** Creates a new ConveyorSubsystem. */
  public ConveyorSubsystem() {
    m_conveyorMotor.configure(Constants.IndexerConstants.k_conveyorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command run(boolean in){
    return Commands.runEnd(() -> {
      m_pid.setSetpoint(in ? Constants.IndexerConstants.k_conveyorInSpeed : Constants.IndexerConstants.k_conveyorOutSpeed , ControlType.kVelocity);
    }, () -> {
      m_pid.setSetpoint(0, ControlType.kVelocity);
    }, this);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Flywheel Velocity", RobotBase.isReal() ? m_encoder.getVelocity() : m_simVelocity);
  }

  public void simulationPeriodic() {
    m_conveyorSim.setInput(m_conveyorMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    m_conveyorSim.update(0.02);
    m_conveyorMotorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(m_conveyorSim.getAngularVelocityRadPerSec()),
        RoboRioSim.getVInVoltage(),0.02);
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_conveyorSim.getCurrentDrawAmps()));
    m_simVelocity = Units.radiansPerSecondToRotationsPerMinute(m_conveyorSim.getAngularVelocityRadPerSec());
  }
}
