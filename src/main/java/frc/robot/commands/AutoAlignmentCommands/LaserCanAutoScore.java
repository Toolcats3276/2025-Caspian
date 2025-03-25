package frc.robot.commands.AutoAlignmentCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.CompoundCommands.ShootCoCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.ElevatorSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.SensorSS;
import frc.robot.subsystems.SwerveSS;
import frc.robot.subsystems.WristSS;

public class LaserCanAutoScore extends SequentialCommandGroup {

  public LaserCanAutoScore(ArmSS s_Arm, InfeedSS s_Infeed, WristSS s_Wrist, ElevatorSS s_Elevator, SwerveSS s_Swerve, SensorSS s_Sensor, boolean strafeLeft) {

    addCommands(

      new SequentialCommandGroup(
        new LaserCanAlignmentCommand(s_Swerve, s_Sensor, strafeLeft),
        new WaitCommand(0.1),
        new ShootCoCommand(s_Arm, s_Infeed, s_Wrist, s_Elevator, s_Sensor)
      )

    );

    addRequirements(s_Swerve, s_Infeed);
  }
  
}
