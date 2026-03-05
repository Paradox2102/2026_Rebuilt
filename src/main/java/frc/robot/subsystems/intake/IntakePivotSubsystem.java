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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.CANIDConstants;

public class IntakePivotSubsystem extends SubsystemBase {
  private SparkFlex m_pivotMotor = new SparkFlex(CANIDConstants.intake_pivot, MotorType.kBrushless);
  private RelativeEncoder m_encoder = m_pivotMotor.getEncoder();
  private PIDController m_pid = new PIDController(IntakeConstants.k_pivotP, IntakeConstants.k_pivotI, IntakeConstants.k_pivotD);

  private SingleJointedArmSim m_pivotSim = new SingleJointedArmSim(DCMotor.getNeoVortex(1), IntakeConstants.k_pivotReduction, IntakeConstants.k_pivotMOI, IntakeConstants.k_pivotLength, 0, IntakeConstants.k_pivotMaxRotation, true, 0);
  private SparkSim m_pivotMotorSim = new SparkSim(m_pivotMotor, DCMotor.getNeoVortex(1));
  
  private double m_simAngleDegrees = 0;

  public Trigger isIntakeRetracted = new Trigger(() -> Math.abs(getPosition() - IntakeConstants.k_pivotMaxRotation) < IntakeConstants.k_pivotDeadzone);

  /** Creates a new IntakePivotSubsystem. */
  public IntakePivotSubsystem() {
    m_pivotMotor.configure(IntakeConstants.k_pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_encoder.setPosition(IntakeConstants.k_pivotMaxRotation);
    setPosition(IntakeConstants.k_pivotMaxRotation);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("retracted", isIntakeRetracted.getAsBoolean());
    SmartDashboard.putNumber("intake pos", getPosition());
    SmartDashboard.putNumber("intake motor output", m_pivotMotor.getAppliedOutput());
    m_pivotMotor.set(m_pid.calculate(getPosition()) + (IntakeConstants.k_pivotCosF * Math.cos(Math.toRadians(getPosition()))));
  }

  public Command extend() {
    return Commands.runOnce(() -> {
      setPosition(-1);
    }, this);
  }

  public Command retract() {
    return Commands.run(() -> {
      setPosition(IntakeConstants.k_pivotMaxRotation);
    }, this).until(isIntakeRetracted);
  }

  public Command fullPower() {
    return Commands.run(() -> {
      m_pivotMotor.setVoltage(12);
    }, this);
  }

  public Command quickRaise(){
    return Commands.runOnce(() -> {
      setPosition(IntakeConstants.k_pivotPullInHeight);
    }, this);
  }

  public RepeatCommand agitate(){
    return new SequentialCommandGroup(quickRaise(), new WaitCommand(1), extend(), new WaitCommand(0.5)).repeatedly();
  }

  public void setPosition(double pos){
    m_pid.setSetpoint(pos);
  }

  public double getPosition() {
    return RobotBase.isReal() ? m_encoder.getPosition() : m_simAngleDegrees;
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
