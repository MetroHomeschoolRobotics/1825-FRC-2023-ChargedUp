package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmStability extends CommandBase {
    private Arm arm;
    public PIDController armPIDController;
    
    public ArmStability( Arm _arm) {
        arm = _arm;
        addRequirements(_arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }
    @Override
    public void initialize() {
        arm.resetAngleEncoders();
        arm.resetShaftEncoders();
    }

  // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      arm.moveAngleMotor(arm.setArmStability(arm.getAbsoluteAngle(), arm.getTeleDistance()));
    }

  // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

  // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
  }
}


