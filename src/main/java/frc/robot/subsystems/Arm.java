// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  CANSparkMax angleMotor = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax telescopingMotor = new CANSparkMax(6, MotorType.kBrushless);
  /** Creates a new Arm. */
  public Arm() {
    angleMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Motor Position", angleMotor.getEncoder().getPosition());
  }

  public void resetAngleEncoders(){
    angleMotor.getEncoder().setPosition(0);
  }

  public double getAngleDistance(){
    return angleMotor.getEncoder().getPosition();
  }
  public double getAngleSpeed(){
    return angleMotor.getEncoder().getVelocity();
  }

  public void setAngleEncoders(double value){
    angleMotor.getEncoder().setPosition(value);
  }

  public void moveAngleMotor(double speed){
    angleMotor.set(speed / 2); //Changed this to half speed to not rip the arm off - Joseph
  }


  public void resetTeleEncoders(){
    telescopingMotor.getEncoder().setPosition(0);
  }
  public double getTeleDistance(){
    return telescopingMotor.getEncoder().getPosition();
  }
  public double getTeleSpeed(){
    return telescopingMotor.getEncoder().getVelocity();
  }
  public void setTeleEncoders(double value){
    telescopingMotor.getEncoder().setPosition(value);
  }
  public void moveTeleMotor(double speed){
    telescopingMotor.set(speed); //Changed this motor from anglemotor to telescoping motor - Joseph
  }
}
