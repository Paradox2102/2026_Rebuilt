// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.ShooterConstants;

public class HoodSubsystem extends SubsystemBase {
  private final SparkFlex m_hoodMotor = new SparkFlex(CANIDConstants.shooter_hood, MotorType.kBrushless);
  private final AbsoluteEncoder m_hoodEncoder = m_hoodMotor.getAbsoluteEncoder();
  private SparkClosedLoopController m_pid = m_hoodMotor.getClosedLoopController();


  private final InterpolatingDoubleTreeMap m_hoodLerpTable = new InterpolatingDoubleTreeMap();
  
  public HoodSubsystem() {
    m_hoodMotor.configure(ShooterConstants.k_hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_hoodLerpTable.put(0.0, 0.0); // add values later
  }

  @Override
  public void periodic() {
    
  }
  public Command pitchHood(DoubleSupplier distanceToGoal){
    return Commands.run(() -> {
      m_pid.setSetpoint(m_hoodLerpTable.get(distanceToGoal.getAsDouble()), ControlType.kPosition);
    }, this);
  };
  public Command returnHood(){
    return Commands.run(() -> {
      m_pid.setSetpoint(m_hoodLerpTable.get(0.0), ControlType.kPosition);
    }, this);
  }
}
