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
import edu.wpi.first.wpilibj.DriverStation;
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
  private final SparkFlex m_lead2 = new SparkFlex(CANIDConstants.shooter_3, MotorType.kBrushless);
  private final SparkFlex m_follow2 = new SparkFlex(CANIDConstants.shooter_4, MotorType.kBrushless);
  private SparkClosedLoopController  m_pid = m_leadMotor.getClosedLoopController();
  private SparkClosedLoopController m_pid2 = m_lead2.getClosedLoopController();

  private RelativeEncoder m_encoder = m_leadMotor.getEncoder();

  private FlywheelSim m_shooterSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNeoVortex(4), ShooterConstants.k_shooterMomentOfInertia, ShooterConstants.k_shooterMotorReduction), DCMotor.getNeoVortex(4));
  private SparkSim m_shooterMotorSim = new SparkSim(m_leadMotor, DCMotor.getNeoVortex(4));
  
  private InterpolatingDoubleTreeMap m_shooterPowerLerp = new InterpolatingDoubleTreeMap();

  private double m_simVelocity = 0;
  private double m_RPMSetPoint = 0;
  private boolean m_isShooting = false;

  public Trigger isShooterOnTarget = new Trigger(() -> (getVelocity() - m_RPMSetPoint >= -ShooterConstants.k_shooterDeadzone) && m_isShooting);
  
  public ShooterSubsystem() {
    m_leadMotor.configure(ShooterConstants.k_leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_follow1.configure(ShooterConstants.k_follower1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_lead2.configure(ShooterConstants.k_leader2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_follow2.configure(ShooterConstants.k_follower2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    for(double[] shotSpeed : ShooterConstants.k_shotSpeeds){
      m_shooterPowerLerp.put(shotSpeed[0], shotSpeed[1]);
    }
  }
  public Command shootCommand(DoubleSupplier distanceToHub){
    return Commands.run(() -> {
      bangBang(m_shooterPowerLerp.get(distanceToHub.getAsDouble()));
      m_isShooting = true;
    }, this).until(() -> {
      if (DriverStation.isAutonomous()) {
        return false; // timer to detect time between shots. if it's too high then stop command.
      }else {
        return false;
      }
    }).finallyDo(() -> {
      bangBang(ShooterConstants.k_shooterRevVel);
      m_isShooting = false;
    });
  }

  public Command staticShootCommand(){
    return Commands.run(() ->{
      m_isShooting = true;
      bangBang(ShooterConstants.k_staticShootVel);
    }, this).finallyDo(() -> {
      bangBang(ShooterConstants.k_shooterRevVel);
      m_isShooting = false;
    });
  }

  public Command revCommand(){
    return Commands.run(() -> {
       bangBang(ShooterConstants.k_shooterRevVel);
    }, this);
  }

  public Command setRPM(double rpm){
    return Commands.runEnd(() -> {
      bangBang(rpm);
    }, () -> {
      setVolts(0);
    }, this);
  }

  private void bangBang(double targetRPM){
    m_RPMSetPoint = targetRPM;
    if(m_RPMSetPoint == 0){
      setVolts(0);
    } else {
      if(getVelocity() < m_RPMSetPoint){
        setVolts(12);
      } else {
        setVolts(ShooterConstants.k_shooterKV * m_RPMSetPoint);
      }
    }
  }

  // private void setVelocity(double rpm){
  //   m_pid.setSetpoint(rpm, ControlType.kVelocity);
  //   m_pid2.setSetpoint(-rpm, ControlType.kVelocity);
  // }

  private void setVolts(double volts){
    m_pid.setSetpoint(volts, ControlType.kVoltage);
    m_pid2.setSetpoint(-volts, ControlType.kVoltage);
  }

  public double getVelocity() {
    return RobotBase.isReal() ? m_encoder.getVelocity() : m_simVelocity;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("shooter revved", isShooterOnTarget.getAsBoolean());
    SmartDashboard.putNumber("shooter speed", getVelocity());
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