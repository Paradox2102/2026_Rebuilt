// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

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
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.CANIDConstants;

public class IntakeRollerSubsystem extends SubsystemBase {
  private SparkFlex m_intakeRollerMotor = new SparkFlex(CANIDConstants.intake_roller, MotorType.kBrushless);
  private SparkClosedLoopController m_pid = m_intakeRollerMotor.getClosedLoopController();
  private RelativeEncoder m_encoder = m_intakeRollerMotor.getEncoder();

  private FlywheelSim m_intakeRollerSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNeoVortex(1), IntakeConstants.k_rollerMOI, IntakeConstants.k_rollerReduction), DCMotor.getNeoVortex(1));
  private SparkSim m_intakeRollerMotorSim = new SparkSim(m_intakeRollerMotor, DCMotor.getNeoVortex(1));
  
  private double m_simVelocity = 0;

  /** Creates a new intakeRollerSubsystem. */
  public IntakeRollerSubsystem() {
    m_intakeRollerMotor.configure(IntakeConstants.k_rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command run(boolean in){
    return Commands.runEnd(() -> {
      m_pid.setSetpoint(in ? IntakeConstants.k_rollerInSpeed : IntakeConstants.k_rollerOutSpeed , ControlType.kVelocity);
    }, () -> {
      m_pid.setSetpoint(0, ControlType.kVelocity);
    }, this);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Roller Velocity", RobotBase.isReal() ? m_encoder.getVelocity() : m_simVelocity);
  }

  public void simulationPeriodic() {
    m_intakeRollerSim.setInput(m_intakeRollerMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    m_intakeRollerSim.update(0.02);
    m_intakeRollerMotorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(m_intakeRollerSim.getAngularVelocityRadPerSec()),
        RoboRioSim.getVInVoltage(),0.02);
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_intakeRollerSim.getCurrentDrawAmps()));
    m_simVelocity = Units.radiansPerSecondToRotationsPerMinute(m_intakeRollerSim.getAngularVelocityRadPerSec());
  }
}
