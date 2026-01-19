// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

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
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.ClimberConstants;

public class ClimberPivotSubsystem extends SubsystemBase {
  public enum ClimberState{
    STOW(0,0,"Stow");
    
    private double m_angle;
    private double m_extension;
    private String m_name;

    ClimberState(double angle, double extension, String name){
      m_angle = angle;
      m_extension = extension;
      m_name = name;
    }

    public double getAngle() {
      return m_angle;
    }

    public double getExtension() {
      return m_extension;
    }

    public String getName() {
      return getName();
    }
  }

  private SparkFlex m_leadMotor = new SparkFlex(CANIDConstants.climber_pivot_leader, MotorType.kBrushless);
  private SparkFlex m_followMotor = new SparkFlex(CANIDConstants.climber_pivot_follower, MotorType.kBrushless);
  private AbsoluteEncoder m_pivotEncoder = m_leadMotor.getAbsoluteEncoder();
  private SparkClosedLoopController m_pid = m_leadMotor.getClosedLoopController();

  private SingleJointedArmSim m_pivotSim = new SingleJointedArmSim(DCMotor.getNeoVortex(2), ClimberConstants.k_pivotReduction, ClimberConstants.k_pivotMOI, ClimberConstants.k_pivotLength, ClimberConstants.k_pivotMinRotation, ClimberConstants.k_pivotMaxRotation, true, ClimberState.STOW.getAngle());
  private SparkSim m_pivotMotorSim = new SparkSim(m_leadMotor, DCMotor.getNeoVortex(2));

  private double m_simAngleDegrees;
  private ClimberState m_state = ClimberState.STOW;

  /** Creates a new ClimberPivotSubsystem. */
  public ClimberPivotSubsystem() {
    m_leadMotor.configure(ClimberConstants.k_leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_followMotor.configure(ClimberConstants.k_leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public ClimberState getState() {
    return m_state;
  }

  public Command setPosition(ClimberState pos){
    return Commands.runOnce(() -> {
      m_state = pos;
    }, this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_pid.setSetpoint(m_state.getAngle(), ControlType.kPosition);
    SmartDashboard.putNumber("Climber Angle", RobotBase.isReal() ? m_pivotEncoder.getPosition() : m_simAngleDegrees);
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
