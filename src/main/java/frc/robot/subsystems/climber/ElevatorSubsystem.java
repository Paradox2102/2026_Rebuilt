// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import java.util.function.Supplier;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.climber.ClimberPivotSubsystem.ClimberState;

public class ElevatorSubsystem extends SubsystemBase {
  private SparkFlex m_elevatorMotor = new SparkFlex(CANIDConstants.climber_extension, MotorType.kBrushless);
  private RelativeEncoder m_encoder = m_elevatorMotor.getEncoder();
  private SparkClosedLoopController m_pid = m_elevatorMotor.getClosedLoopController();

  private ElevatorSim m_elevatorSim = new ElevatorSim(LinearSystemId.createElevatorSystem(DCMotor.getNeoVortex(1), ClimberConstants.k_elevatorWeight, ClimberConstants.k_elevatorDrumWidth, ClimberConstants.k_elevatorReduction), DCMotor.getNeoVortex(1), 0, ClimberConstants.k_elevatorMaxHeight, true, 0);
  private SparkSim m_motorSim = new SparkSim(m_elevatorMotor, DCMotor.getNeoVortex(1));

  private Supplier<ClimberState> m_state;
  private double m_simHeight = 0;
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem(Supplier<ClimberState> state) {
    m_state = state;
    m_elevatorMotor.configure(ClimberConstants.k_elevConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_pid.setSetpoint(m_state.get().getExtension(), ControlType.kPosition);
    SmartDashboard.putNumber("Elevator Height", RobotBase.isReal() ? m_encoder.getPosition() : m_simHeight);
  }

  public void simulationPeriodic() {
    m_elevatorSim.setInput(m_motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    m_elevatorSim.update(0.02);
    m_motorSim.iterate(
      Units.radiansPerSecondToRotationsPerMinute(m_elevatorSim.getVelocityMetersPerSecond()),
      RoboRioSim.getVInVoltage(), 0.02);
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
    m_simHeight = m_elevatorSim.getPositionMeters();
  }
}
