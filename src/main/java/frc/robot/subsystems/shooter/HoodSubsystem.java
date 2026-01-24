// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkSim;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.ShooterConstants;

public class HoodSubsystem extends SubsystemBase {
  private final SparkFlex m_hoodMotor = new SparkFlex(CANIDConstants.shooter_hood, MotorType.kBrushless);
  private final AbsoluteEncoder m_hoodEncoder = m_hoodMotor.getAbsoluteEncoder();
  private SparkClosedLoopController m_pid = m_hoodMotor.getClosedLoopController();

  private SingleJointedArmSim m_hoodSim = new SingleJointedArmSim(DCMotor.getNeoVortex(1), ShooterConstants.k_hoodGearRatio, ShooterConstants.k_hoodMomentOfInertia, ShooterConstants.k_hoodArmLengthMeters, 0, Math.toRadians(0.0), true, Math.toRadians(0.0));
  private SparkSim m_hoodMotorSim = new SparkSim(m_hoodMotor, DCMotor.getNeoVortex(1));

  private double m_hoodSimAngle;

  private double m_autoAlignTrim = 0;
  private double m_staticTrim = 0;

  private final InterpolatingDoubleTreeMap m_hoodLerpTable = new InterpolatingDoubleTreeMap();

  public Trigger isHoodOnTarget = new Trigger(() -> (Math.abs(getHoodAngle() - m_pid.getSetpoint()) <= ShooterConstants.k_hoodDeadzone));
  
  public HoodSubsystem() {
    m_hoodMotor.configure(ShooterConstants.k_hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_hoodLerpTable.put(0.0, 0.0); // add values later
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood Angle", getHoodAngle());
    SmartDashboard.putNumber("Auto Aim Hood Trim", m_autoAlignTrim);
    SmartDashboard.putNumber("Static Hood Trim", m_staticTrim);
  }

  public double getHoodAngle() {
    return RobotBase.isReal() ? m_hoodEncoder.getPosition() : m_hoodSimAngle;
  }

  public Command pitchHood(DoubleSupplier distanceToGoal){
    return Commands.run(() -> {
      m_pid.setSetpoint(m_hoodLerpTable.get(distanceToGoal.getAsDouble()) + m_autoAlignTrim, ControlType.kPosition);
    }, this);
  }

  public Command staticPitch(){
    return Commands.run(() -> {
      m_pid.setSetpoint(ShooterConstants.k_staticHoodAngle + m_staticTrim, ControlType.kPosition);
    }, this);
  }

  public Command returnHood(){
    return Commands.run(() -> {
      m_pid.setSetpoint(0, ControlType.kPosition);
    }, this);
  }

  public Command trimShooterCommand(BooleanSupplier isAutoAim, boolean up){
    return Commands.runOnce(() -> {
      if(isAutoAim.getAsBoolean()){
        m_autoAlignTrim += up ? 2 : -2;
      } else {
        m_staticTrim += up ? 2 : -2;
      }
    });
  }

  @Override
  public void simulationPeriodic() {
      m_hoodSim.setInput(m_hoodMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    m_hoodSim.update(0.02);
    m_hoodMotorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(m_hoodSim.getVelocityRadPerSec()),
        RoboRioSim.getVInVoltage(),0.02);
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_hoodSim.getCurrentDrawAmps()));
    m_hoodSimAngle = Math.toDegrees(m_hoodSim.getAngleRads());
  }
}
