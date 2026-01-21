// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;


import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkSim;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.IndexerConstants;

public class ShooterSubsystem extends SubsystemBase {

  private final SparkFlex m_topRightMotor = new SparkFlex(CANIDConstants.shooter_1, MotorType.kBrushless);
  private final SparkFlex m_topLeftMotor = new SparkFlex(CANIDConstants.shooter_2, MotorType.kBrushless);
  private final SparkFlex m_bottomRightMotor = new SparkFlex(CANIDConstants.shooter_3, MotorType.kBrushless);
  private final SparkFlex m_bottomLeftMotor = new SparkFlex(CANIDConstants.shooter_4, MotorType.kBrushless);

  private FlywheelSim m_shooterSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNeoVortex(1), IndexerConstants.k_conveyorMOI, IndexerConstants.k_conveyorReduction), DCMotor.getNeoVortex(1));
  private SparkSim m_shooterMotorSim = new SparkSim(m_topRightMotor, DCMotor.getNeoVortex(4));
  
  private InterpolatingDoubleTreeMap m_shooterPowerLerp = new InterpolatingDoubleTreeMap();

  private double m_simVelocity = 0;

  private double m_power = 0.0;
  
  public ShooterSubsystem(double power) {
    m_power = power;
    m_shooterPowerLerp.put(0.0, 0.0); // add values later
  }
  public Command shootCommand(double distanceToHub){
    return Commands.run(() -> {
      m_power = m_shooterPowerLerp.get(distanceToHub);
    }, this).until(() -> true).finallyDo(() -> m_power = 0.5); // set to whatever is decided for stopping shooting
  }
  public Command revCommand(){
    return Commands.run(() -> m_power = 0.5, this);
  }
  @Override
  public void periodic() {

    m_topLeftMotor.set(m_power);
    m_topRightMotor.set(m_power);
    m_bottomLeftMotor.set(m_power);
    m_bottomRightMotor.set(m_power);
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
