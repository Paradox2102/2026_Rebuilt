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
  private SparkFlex m_leftMotor = new SparkFlex(CANIDConstants.climber_left, MotorType.kBrushless);
  private SparkFlex m_rightMotor = new SparkFlex(CANIDConstants.climber_right, MotorType.kBrushless);
  private RelativeEncoder m_leftEncoder = m_leftMotor.getEncoder();
  private RelativeEncoder m_rightEncoder = m_rightMotor.getEncoder();
  private SparkClosedLoopController m_leftPID = m_leftMotor.getClosedLoopController();
  private SparkClosedLoopController m_rightPID = m_rightMotor.getClosedLoopController();

  private ElevatorSim m_climberSim = new ElevatorSim(LinearSystemId.createElevatorSystem(DCMotor.getNeoVortex(1), ClimberConstants.k_climberWeight/2.0, ClimberConstants.k_climberDrumWidth/2.0, ClimberConstants.k_climberReduction), DCMotor.getNeoVortex(1), 0, ClimberConstants.k_climberMaxHeight, false, 0);
  private SparkSim m_motorSim = new SparkSim(m_rightMotor, DCMotor.getNeoVortex(1));

  private double m_simHeight = 0;

  public Trigger isClimberRetracted = new Trigger(() -> getHeight()[0] <= ClimberConstants.k_climberDeadzone && getHeight()[1] <= ClimberConstants.k_climberDeadzone);
  public Trigger isClimberExtended = new Trigger(() -> getHeight()[0] >= ClimberConstants.k_climberMaxHeight -ClimberConstants.k_climberDeadzone && getHeight()[1] >= ClimberConstants.k_climberMaxHeight - ClimberConstants.k_climberDeadzone);
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    m_leftMotor.configure(ClimberConstants.k_leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightMotor.configure(ClimberConstants.k_rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("climber left height", getHeight()[0]);
    SmartDashboard.putNumber("climber right height", getHeight()[1]);
  }

  public double[] getHeight(){
    return RobotBase.isReal() ? new double[] {m_leftEncoder.getPosition(), m_rightEncoder.getPosition()} : new double[] {m_simHeight, m_simHeight};
  }

  public Command extend(){
    return Commands.runOnce(() -> {
      setHeight(ClimberConstants.k_climberMaxHeight);
    }, this);
  }

  public Command retract(){
    return Commands.run(() -> {
      setHeight(0);
    }, this).until(isClimberRetracted);
  }
  public Command climbingRetract(){
    return Commands.runOnce(() -> {
      setHeight(ClimberConstants.k_climberClimbingStowedHeight);
    }, this);
  }

  public void setHeight(double height){
    m_leftPID.setSetpoint(height, ControlType.kPosition);
    m_rightPID.setSetpoint(height, ControlType.kPosition);
  }

  public Command setPower(double power) {
    return Commands.run(() -> {
      m_leftPID.setSetpoint(power, ControlType.kVoltage);
      m_rightPID.setSetpoint(power, ControlType.kVoltage);
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
