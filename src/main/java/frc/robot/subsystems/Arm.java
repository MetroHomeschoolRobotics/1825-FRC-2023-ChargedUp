// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class Arm extends SubsystemBase {
  CANSparkMax angleMotor = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax telescopingMotor = new CANSparkMax(6, MotorType.kBrushless);
  DutyCycleEncoder rotationEncoder = new DutyCycleEncoder(0);
  /** Creates a new Arm. */
  public Arm() {
    angleMotor.setInverted(true);
    //telescopingMotor.getEncoder().setPositionConversionFactor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Motor Position", angleMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Shaft Encoder Position", rotationEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("ExtensionEncoderValue", telescopingMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("ShoulderSpeed", angleMotor.get() );
    SmartDashboard.putNumber("absolute angle", getAbsoluteAngle());
  }
//The motor encoder for the rotating arm
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
    angleMotor.set(speed); //Change this for better arm controlability.
  }
//The REV Through bore shaft encoder
  public void resetShaftEncoders(){
    rotationEncoder.reset();
  }
  public void getShaftAngle(){
    rotationEncoder.get();
  }
  public double getShaftRotation(){
    return rotationEncoder.getDistance();
  }
  public double getAbsoluteShaftRotation(){
    return rotationEncoder.getAbsolutePosition();
  }
  public double getAbsoluteAngle(){
    return ((rotationEncoder.getAbsolutePosition()-.63)*(360/1)+16); // this encoder used rotations as a position mesurement so we move the degrees to the top and convert it to degrees with this method
  }
  
//The motor encoder for the telescoping arm
  public void resetTeleEncoders(){
    telescopingMotor.getEncoder().setPosition(0);
  }
  public double getTeleEncoderDistance(){
    return telescopingMotor.getEncoder().getPosition();
  }
  public double getTeleDistance(){
    return getTeleEncoderDistance()*-0.15+84.46;  // the conversion from the length of the arm from the encoder 
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

  public double setArmStability(double angle, double radius){ //this equation helps the arm to fight gravity which is affected by the angle and arm extension length
    
    double balancePointDistance = 0.00333*(radius)*(radius)+0.130301*(radius);  // the arm balance point is parabolicly related to the lenght  // this encoder used rotations as a position mesurement, so it can be converted to degrees with this method
    
    double force = Math.sin(angle*(Math.PI/180)) * balancePointDistance * 0.01;  // to get the input needed, we take the sine of the angle from the top and increase the amplitude depending on how extended the arm is times some constant
    return force;
  }
}
