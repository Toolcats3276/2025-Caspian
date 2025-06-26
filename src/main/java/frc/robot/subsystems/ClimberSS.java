// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSS extends SubsystemBase {

  private TalonFX m_climberMotor;

  private double speed;

  private DigitalInput ClimbLeftLimitSwitch;
  private DigitalInput ClimbRightLimitSwitch;

  /** Creates a new ClimberSS. */
  public ClimberSS() {

    m_climberMotor = new TalonFX(35);
    m_climberMotor.getConfigurator().apply(new TalonFXConfiguration());
    m_climberMotor.setNeutralMode(NeutralModeValue.Brake);

    ClimbLeftLimitSwitch = new DigitalInput(3);
    ClimbRightLimitSwitch = new DigitalInput(2);

  }

  public enum Mode{
    ManualIn,
    ManualOut,
    ManualStop,
    ManualClimb
  }


  Mode ClimberMode = Mode.ManualStop;

  @Override
  public void periodic() {

    switch(ClimberMode) {

      case ManualIn:{
        if(returnRightClimbLimit() && returnLeftClimbLimit()){
          m_climberMotor.set(0);
          ClimberMode = Mode.ManualStop;
        }

        else{
          m_climberMotor.set(-ClimberConstants.MANUAL_SPEED);
        }
        break;
      }

      case ManualOut:{
          m_climberMotor.set(ClimberConstants.MANUAL_SPEED);
          break;
      }

      case ManualStop:{
          m_climberMotor.set(0);
          break;
      }

      case ManualClimb:{

        m_climberMotor.set(speed);
      }
    }

    SmartDashboard.putBoolean("ClimberRightLimit", returnRightClimbLimit());
    SmartDashboard.putBoolean("ClimberLeftLimit", returnLeftClimbLimit());
  }

  public void ManualIn(){
    ClimberMode = Mode.ManualIn;
  }
  
  public void ManualOut(){
      ClimberMode = Mode.ManualOut;
  }
  
  public void ManualStop(){
      ClimberMode = Mode.ManualStop;
  }

  public void setSpeed(double speed){
    this.speed = speed;
    ClimberMode = Mode.ManualClimb;
  }

  public boolean returnRightClimbLimit(){
    return !ClimbRightLimitSwitch.get();
}
  
  public boolean returnLeftClimbLimit(){
    return !ClimbLeftLimitSwitch.get();
  }
}
