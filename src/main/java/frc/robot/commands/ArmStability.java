package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;

public class ArmStability extends CommandBase {
    private Arm arm;
    private CommandXboxController controller;
    private double force;
    public PIDController armPIDController;
    private double value;
    
    public ArmStability(CommandXboxController _controller, Arm _arm, double encoderValue) {
        arm = _arm;
        value = encoderValue;
        controller = _controller;
        addRequirements(_arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }
    @Override
    public void initialize() {
        arm.resetAngleEncoders();
        arm.setAngleEncoders(value);
    }

  // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //arm.moveAngleMotor(arm.setArmStability(0, arm.getTeleDistance()*-0.15 + 84.46/*this is the conversion from encoder to actual extension in inches */));
        arm.moveAngleMotor(-0.031);
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


