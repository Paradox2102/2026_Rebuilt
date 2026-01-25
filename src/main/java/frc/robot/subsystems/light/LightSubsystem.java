// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.light;

import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.LightConstants;

public class LightSubsystem extends SubsystemBase {
  /** Creates a new LightSubsystem. */
  private AddressableLED m_led = new AddressableLED(LightConstants.k_lightPort);
  public AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(LightConstants.k_lightAmount);
  private LEDPattern m_pattern;
  private Timer m_timer = new Timer();

  private final double k_shift1Time = 10.0;
  private final double k_shift2Time = 35.0;
  private final double k_shift3Time = 60.0;
  private final double k_shift4Time = 85.0;
  private final double k_endShiftTime = 110.0;
  private final double k_matchEndTime = 140.0;

  private final double k_transitionTime = 7.0;
  private final double k_shiftTime = 22.0;
  private final double k_endTime = 27.0;

  private int m_shift = 0;
  private String gameData;
  private static final Time k_blinkMagnitude = Seconds.of(0.3);

  private final class LedPatterns { // if it doesn't work as third of each team color and scrolling it likely has to do with me not knowing how maps work and it's an issue with k1, 2, and 3
      private static final LEDPattern m_disabledPattern = LEDPattern.steps(Map.of(0, Color.kRed, 0.33, Color.kBlue, 0.66, Color.kGreen)).scrollAtAbsoluteSpeed(LightConstants.k_disabledVelocity, LightConstants.k_ledSpacing);
      
    }  

  private final class ShiftCommand extends Command {
    private final Timer m_blinkTimer = new Timer();

    private final int m_commandShift;
    private final boolean m_odd;
    private final double m_endTime;
    private Color m_color = Color.kBlack;

    private final LightSubsystem m_subsystem;
    public ShiftCommand(boolean odd, double endTime, int shift, LightSubsystem subsystem) {
      m_commandShift = shift;
      m_odd = odd;
      m_endTime = endTime;
      m_subsystem = subsystem;
      new Trigger(() -> m_blinkTimer.get() > m_endTime).onTrue(new SetPattern(m_color, true, m_subsystem));
    }
    
    @Override
    public void initialize() {
      m_timer.start();
      m_blinkTimer.reset();
      m_blinkTimer.start();
      if (m_commandShift != 0 || m_commandShift != 5){
        if (m_odd) {
          m_color = getWonAuto() ? Color.kRed : Color.kGreen;
        }
        else {
          m_color = getWonAuto() ? Color.kGreen : Color.kRed;
        }
      }
      else{
        m_color = Color.kGreen;
      }
      new SetPattern(m_color, false, m_subsystem);
    }
    @Override
    public boolean isFinished() {
      m_shift ++;
      switch(m_commandShift) {
        case 0:
          return m_timer.get() > k_shift1Time;
          
        case 1:
          return m_timer.get() > k_shift2Time;

        case 2:
          return m_timer.get() > k_shift3Time;

        case 3:
          return m_timer.get() > k_shift4Time;

        case 4:
          return m_timer.get() > k_endShiftTime;

        case 5:
          return m_timer.get() > k_matchEndTime;

        default:
          return true;

      }
    }
  }

  public class SetPattern extends Command {
    public SetPattern(Color color, boolean blinking, LightSubsystem subsystem) {
      m_pattern = LEDPattern.solid(color);
      if (blinking) {
        m_pattern.blink(k_blinkMagnitude);
      }
      m_pattern.applyTo(m_ledBuffer);
    }
  }

  public LightSubsystem() {
    m_led.setLength(LightConstants.k_lightAmount);
    m_led.setData(m_ledBuffer);
    m_led.start();
    
    new Trigger(() -> DriverStation.isTeleop() && !DriverStation.isDisabled()).onTrue(new ShiftCommand(false, k_transitionTime, 1, this));
    new Trigger(() -> m_timer.get() > k_shift1Time).onTrue(new ShiftCommand(true, k_shiftTime, 1, this));
    new Trigger(() -> m_timer.get() > k_shift2Time).onTrue(new ShiftCommand(false, k_shiftTime, 2, this));
    new Trigger(() -> m_timer.get() > k_shift3Time).onTrue(new ShiftCommand(true, k_shiftTime, 3, this));
    new Trigger(() -> m_timer.get() > k_shift4Time).onTrue(new ShiftCommand(false, k_shiftTime, 4, this));
    new Trigger(() -> m_timer.get() > k_endShiftTime).onTrue(new ShiftCommand(false, k_endTime, 5, this));
  }

  @Override
  public void periodic() {
    gameData = DriverStation.getGameSpecificMessage();
    if (DriverStation.isDisabled()){
      m_pattern = LedPatterns.m_disabledPattern;
    }
    m_pattern.applyTo(m_ledBuffer);
    
    m_led.setData(m_ledBuffer);
  }
  public boolean hubIsActive(){
    switch (m_shift) {
      case 0:
        return true;
      case 1:
        return getWonAuto() ? false : true;
      case 2:
        return getWonAuto() ? true : false;
      case 3:
        return getWonAuto() ? false : true;
      case 4:
        return getWonAuto() ? true : false;
      case 5:
        return true;
      default:
        return false;
    }
  }
  private boolean getWonAuto() {
    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
        case 'B':
          if (DriverStation.getAlliance().get() == Alliance.Blue) {
            return true;
          }
          else {
            return false;
          }
        case 'R':
          if (DriverStation.getAlliance().get() == Alliance.Red) {
            return true;
          }
          else {
            return false;
          }
        default:
          return true;
      }
    }
    else {
      return true;
    }
  }
}
