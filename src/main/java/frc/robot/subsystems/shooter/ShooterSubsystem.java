// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;


import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private final SparkFlex m_leadMotor = new SparkFlex(CANIDConstants.shooter_1, MotorType.kBrushless);
  private final SparkFlex m_follow1 = new SparkFlex(CANIDConstants.shooter_2, MotorType.kBrushless);
  private final SparkFlex m_follow2 = new SparkFlex(CANIDConstants.shooter_3, MotorType.kBrushless);
  private final SparkFlex m_follow3 = new SparkFlex(CANIDConstants.shooter_4, MotorType.kBrushless);
  private SparkClosedLoopController  m_pid = m_leadMotor.getClosedLoopController();
  private RelativeEncoder m_encoder = m_leadMotor.getEncoder();

  private FlywheelSim m_shooterSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNeoVortex(1), ShooterConstants.k_shooterMomentOfInertia, ShooterConstants.k_shooterMotorReduction), DCMotor.getNeoVortex(1));
  private SparkSim m_shooterMotorSim = new SparkSim(m_leadMotor, DCMotor.getNeoVortex(4));
  
  private InterpolatingDoubleTreeMap m_shooterPowerLerp = new InterpolatingDoubleTreeMap();

  private double m_simVelocity = 0;

  public Trigger isShooterOnTarget = new Trigger(() -> (Math.abs(getVelocity() - m_pid.getSetpoint()) <= ShooterConstants.k_shooterDeadzone));
  
  public ShooterSubsystem() {
    m_shooterPowerLerp.put(0.0, 0.0); // add values later
    m_leadMotor.configure(ShooterConstants.k_leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_follow1.configure(ShooterConstants.k_follower1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_follow2.configure(ShooterConstants.k_follower23Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_follow3.configure(ShooterConstants.k_follower23Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command shootCommand(DoubleSupplier distanceToHub){
    return Commands.run(() -> {
      m_pid.setSetpoint(m_shooterPowerLerp.get(distanceToHub.getAsDouble()), ControlType.kVelocity);
    }, this).until(() -> true).finallyDo(() -> {
      m_pid.setSetpoint(ShooterConstants.k_shooterRevVel, ControlType.kVelocity );
    }); // set to whatever is decided for stopping shooting
  }

  public Command revCommand(){
    return Commands.run(() -> {
      m_pid.setSetpoint(ShooterConstants.k_shooterRevVel, ControlType.kVelocity);
    }, this);
  }

  public double getVelocity() {
    return RobotBase.isReal() ? m_encoder.getVelocity() : m_simVelocity;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Velocity", getVelocity());
  }

  @Override
  public void simulationPeriodic() {
    m_shooterSim.setInput(m_shooterMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    m_shooterSim.update(0.02);
    m_shooterMotorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(m_shooterSim.getAngularVelocityRadPerSec()),
        RoboRioSim.getVInVoltage(),0.02);
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_shooterSim.getCurrentDrawAmps()));
    m_simVelocity = Units.radiansPerSecondToRotationsPerMinute(m_shooterSim.getAngularVelocityRadPerSec());
  }
}
