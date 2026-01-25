// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

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
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private SparkFlex m_leadMotor = new SparkFlex(CANIDConstants.climber_leader, MotorType.kBrushless);
  private SparkFlex m_followMotor = new SparkFlex(CANIDConstants.climber_follower, MotorType.kBrushless);
  private RelativeEncoder m_encoder = m_leadMotor.getEncoder();
  private SparkClosedLoopController m_pid = m_leadMotor.getClosedLoopController();

  private ElevatorSim m_climberSim = new ElevatorSim(LinearSystemId.createElevatorSystem(DCMotor.getNeoVortex(1), ClimberConstants.k_climberWeight, ClimberConstants.k_climberDrumWidth/2.0, ClimberConstants.k_climberReduction), DCMotor.getNeoVortex(1), 0, ClimberConstants.k_climberMaxHeight, true, 0);
  private SparkSim m_motorSim = new SparkSim(m_leadMotor, DCMotor.getNeoVortex(1));

  private double m_simHeight = 0;

  public Trigger isClimberRetracted = new Trigger(() -> getHeight() < ClimberConstants.k_climberDeadzone);
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    m_leadMotor.configure(ClimberConstants.k_leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_followMotor.configure(ClimberConstants.k_followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber Height", getHeight());
  }

  public double getHeight(){
    return RobotBase.isReal() ? m_encoder.getPosition() : m_simHeight;
  }

  public Command extend(){
    return Commands.runOnce(() -> {
      m_pid.setSetpoint(ClimberConstants.k_climberMaxHeight, ControlType.kPosition);
    }, this);
  }

  public Command retract(){
    return Commands.runOnce(() -> {
      m_pid.setSetpoint(0, ControlType.kPosition);
    }, this);
  }

  public void simulationPeriodic() {
    m_climberSim.setInput(m_motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    m_climberSim.update(0.02);
    m_motorSim.iterate(
      Units.radiansPerSecondToRotationsPerMinute(m_climberSim.getVelocityMetersPerSecond()),
      RoboRioSim.getVInVoltage(), 0.02);
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_climberSim.getCurrentDrawAmps()));
    m_simHeight = m_climberSim.getPositionMeters();
  }
}
