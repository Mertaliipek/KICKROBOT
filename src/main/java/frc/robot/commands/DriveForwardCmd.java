package frc.robot.commands;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class DriveForwardCmd extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private final double speedLeft;
  private final double speedRight;

  public DriveForwardCmd(DriveSubsystem driveSubsystem,double speedLeft,double speedRight) {
    this.driveSubsystem = driveSubsystem;
    this.speedLeft = speedLeft;
    this.speedRight = speedRight;
    addRequirements(driveSubsystem);
  }



     // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    System.out.print("drive forward started");
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.setMotors(speedLeft,speedRight);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.setMotors(0,0);
    System.out.println("drive forward ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

    
}
