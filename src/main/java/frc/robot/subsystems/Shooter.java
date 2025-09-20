// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  
  /** Creates a new Shooter. */
  public Shooter() {}
  private TalonFX shooterTop = new TalonFX(0);
  private TalonFX shooterBottom = new TalonFX(1);
  public void set(double value) {
    shooterTop.set(value);
    shooterBottom.set(value);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
