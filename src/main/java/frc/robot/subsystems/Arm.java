// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  CANSparkMax angleMotor = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax telescopingMotor = new CANSparkMax(6, MotorType.kBrushless);
  CANSparkMax telescopingMotor2 = new CANSparkMax(7, MotorType.kBrushless);
  DutyCycleEncoder rotationEncoder = new DutyCycleEncoder(0);
  private DigitalInput BeamBreakSensor;
  /** Creates a new Arm. */
  public Arm() {
    angleMotor.setInverted(true);
    BeamBreakSensor = new DigitalInput(Constants.BeamBreakSensor);

    angleMotor.setSmartCurrentLimit(60);
    telescopingMotor.setSmartCurrentLimit(35);
    telescopingMotor2.setSmartCurrentLimit(35);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // automatically zero the tele encoder when beam break is triggered
    if(!BeamBreakSensor.get())
    {
      resetTeleEncoders();
    }

    SmartDashboard.putNumber("Arm Motor Position", angleMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Shaft Encoder Position", rotationEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("ExtensionEncoderValue", telescopingMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Telescoping distance!!!", getTeleEncoderDistance());
    SmartDashboard.putNumber("Absolute Angle", getAbsoluteAngle());
    SmartDashboard.putNumber("Motor Input", setArmStability(getAbsoluteAngle(), getTeleEncoderDistance()));
    SmartDashboard.putBoolean("Retract Sensor", !BeamBreakSensor.get());
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
    final double discontinuityOffset = 1.25;  // move the encoder's discontinuity where the arm can't reach
    final double calibrationOffset = -.63+.129;  // calibrate so zero degrees is vertical
    return (((rotationEncoder.getAbsolutePosition()+discontinuityOffset) % 1.0+calibrationOffset)*(360.0)); // this encoder used rotations as a position mesurement so we move the degrees to the top and convert it to degrees with this method
  }
  
//The motor encoder for the telescoping arm
  public void resetTeleEncoders(){
    telescopingMotor.getEncoder().setPosition(0);
  }
  public double getTeleEncoderDistance(){
    return telescopingMotor.getEncoder().getPosition();
  }
  public double getTeleDistance(){
    return getTeleEncoderDistance()*-0.15+30;  // the conversion from the length of the arm from the encoder 
  }
  public double getTeleSpeed(){
    return telescopingMotor.getEncoder().getVelocity();
  }
  public void setTeleEncoders(double value){
    telescopingMotor.getEncoder().setPosition(value);
  }
  public void moveTeleMotor(double speed){
    telescopingMotor.set(speed); //Changed this motor from anglemotor to telescoping motor - Joseph
    telescopingMotor2.set(speed);
  }

  public boolean getBeamBreakSensor() {//reads true if triggered for retracting the arm.
    return !BeamBreakSensor.get();
  }
// Insert arm stability here 

public double setArmStability(double angle, double radius){ //this equation helps the arm to fight gravity which is affected by the angle and arm extension length
    
  //Maximum radius = 111.14
  //Desired result at full extension. horizontal is 0.55
  //Desired result at full retraction, horizontal is 0.0859
  //double maxForce = 0.000303*radius + 0.0156;
  final double retractedForce = 0.03693;
  final double extendedForce = 0.234375;
  final double extendedRadius = 110.0;
  final double maxForce = (extendedForce - retractedForce)*radius/extendedRadius + retractedForce;
  
  double force = Math.sin(angle*(Math.PI/180))*maxForce;  // to get the input needed, we take the sine of the angle from the top and increase the amplitude depending on how extended the arm is times some constant
 
  SmartDashboard.putNumber("Arm Stability Force", force);

  return force;
}


}
