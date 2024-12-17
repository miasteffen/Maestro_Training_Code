// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endEffector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffector extends SubsystemBase {
  
  private final TalonFX motor = new TalonFX(Constants.CandIDs.END_EFFECTOR_CAN_ID);
  
  private final VoltageOut openLoopControl = new VoltageOut(0).withEnableFOC(true);
 
 private final StatusSignal<Double> deviceTemp;
 private final StatusSignal<Double> appliedVolts;
 private final StatusSignal<Double> currentAmps;
 private final StatusSignal<Double> velocityRPS;

  /** Creates a new EndEffector. */
  public EndEffector() {
    TalonFXConfiguration config = new TalonFXConfiguration();
  
    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.SupplyCurrentEnable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = Inverted Value.CounterClockwise_Positive;

    config.Feedback.SensorToMechanismRatio = Constants.EndEffectorConstants.GEAR_RATIO;
    
    motor.getConfigurator().apply(config);_

    deviceTemp = motor.getDeviceTemp();
    appliedVolts = motor.getMotorVoltage();
    currentAmps = motor.getStatorCurrent();
    velocityRPS = motor.getVelocity();
  }


  public void setVoltage(double volt){
      motor.setControl(openLoopControl.withOutput(volts));

    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    }
  }
  
  
    
  