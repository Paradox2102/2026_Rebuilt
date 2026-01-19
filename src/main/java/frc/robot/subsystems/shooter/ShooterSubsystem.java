// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;


import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;;

public class ShooterSubsystem extends SubsystemBase {

  private final SparkFlex m_topRightMotor = new SparkFlex(CANIDConstants.shooter_1, MotorType.kBrushless);
  private final SparkFlex m_topLeftMotor = new SparkFlex(CANIDConstants.shooter_2, MotorType.kBrushless);
  private final SparkFlex m_bottomRightMotor = new SparkFlex(CANIDConstants.shooter_3, MotorType.kBrushless);
  private final SparkFlex m_bottomLeftMotor = new SparkFlex(CANIDConstants.shooter_4, MotorType.kBrushless);

  private double m_power = 0.0;
  
  public ShooterSubsystem(double power) {
    m_power = power;
  }
  @Override
  public void periodic() {
    m_topLeftMotor.set(m_power);
    m_topRightMotor.set(m_power);
    m_bottomLeftMotor.set(m_power);
    m_bottomRightMotor.set(m_power);
  }
}
