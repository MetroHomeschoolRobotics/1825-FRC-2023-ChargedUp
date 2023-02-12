// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  CANSparkMax motor1 = new CANSparkMax(5, MotorType.kBrushless);
  /** Creates a new Arm. */
  public Arm() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Motor Position", motor1.getEncoder().getPosition());
  }

  public void resetEncoders(){
    motor1.getEncoder().setPosition(0);
  }

  public double getDistance(){
    return motor1.getEncoder().getPosition();
  }
  public double getSpeed(){
    return motor1.getEncoder().getVelocity();
  }

  public void moveMotor(double speed){
    motor1.set(speed);
  }

}
