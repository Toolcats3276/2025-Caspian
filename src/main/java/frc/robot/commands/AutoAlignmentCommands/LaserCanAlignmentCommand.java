package frc.robot.commands.AutoAlignmentCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SensorSS;
import frc.robot.subsystems.SwerveSS;

public class LaserCanAlignmentCommand extends Command {

  private SwerveSS s_Swerve;
  private SensorSS s_Sensor;
  private boolean strafeLeft;

  public LaserCanAlignmentCommand(SwerveSS s_Swerve, SensorSS s_Sensor, boolean strafeLeft) {
    this.s_Swerve = s_Swerve;
    this.s_Sensor = s_Sensor;
    this.strafeLeft = strafeLeft;

    addRequirements(s_Swerve);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    double translationVal;
    double strafeVal;

    if(s_Sensor.reefSensed()){
      translationVal = -0.5;
      strafeVal = 0;
    }

    else if(strafeLeft){
      translationVal = 0;
      strafeVal = -0.4;
    }

    else{
      translationVal = 0;
      strafeVal = 0.4;
    }

    s_Swerve.drive(
      new Translation2d(translationVal, strafeVal),
      0,
      false,
      true,
      new Translation2d(0, 0),
      0
    );
  }

  @Override
  public void end(boolean interrupted) {
    this.cancel();
  }

  @Override
  public boolean isFinished() {
    return s_Sensor.reefSensed();
  }
}
