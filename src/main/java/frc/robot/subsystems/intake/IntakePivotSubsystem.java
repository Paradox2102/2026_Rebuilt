// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakePivotSubsystem extends SubsystemBase {
  private SparkFlex m_pivotMotor = new SparkFlex(Constants.CANIDConstants.intake_pivot, MotorType.kBrushless);
  private AbsoluteEncoder m_encoder = m_pivotMotor.getAbsoluteEncoder();
  private SparkClosedLoopController m_pid = m_pivotMotor.getClosedLoopController();

  private SingleJointedArmSim m_pivotSim = new SingleJointedArmSim(DCMotor.getNeoVortex(1), Constants.IntakeConstants.k_pivotReduction, Constants.IntakeConstants.k_pivotMOI, Constants.IntakeConstants.k_pivotLength, 0, Constants.IntakeConstants.k_pivotMaxRotation, true, Constants.IntakeConstants.k_pivotMaxRotation);
  private SparkSim m_pivotMotorSim = new SparkSim(m_pivotMotor, DCMotor.getNeoVortex(1));
  
  private double m_simAngleDegrees = 0;

  /** Creates a new IntakePivotSubsystem. */
  public IntakePivotSubsystem() {
    m_pivotMotor.configure(Constants.IntakeConstants.k_pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pivot Angle Degrees", RobotBase.isReal() ? m_encoder.getPosition() : m_simAngleDegrees);
  }

  public Command extend() {
    return Commands.runOnce(() -> {
      m_pid.setSetpoint(0, ControlType.kPosition);
    }, this);
  }

  public Command retract() {
    return Commands.runOnce(() -> {
      m_pid.setSetpoint(Constants.IntakeConstants.k_pivotMaxRotation, ControlType.kPosition);
    }, this);
  }

  public void simulationPeriodic() {
    m_pivotSim.setInput(m_pivotMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    m_pivotSim.update(0.02);
    m_pivotMotorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(m_pivotSim.getVelocityRadPerSec()),
        RoboRioSim.getVInVoltage(),0.02);
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_pivotSim.getCurrentDrawAmps()));
    m_simAngleDegrees = Math.toDegrees(m_pivotSim.getAngleRads());
  }
}
