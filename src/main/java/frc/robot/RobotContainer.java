package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.InfeedConstants;
import frc.robot.commands.*;
import frc.robot.commands.AutoAlignmentCommands.LaserCanAutoScore;
import frc.robot.commands.BaseCommands.InfeedCommands.InfeedCommand;
import frc.robot.commands.CompoundCommands.CancelCoCommand;
import frc.robot.commands.CompoundCommands.CompCoCommand;
import frc.robot.commands.CompoundCommands.ShootCoCommand;
import frc.robot.commands.CompoundCommands.SuckBack;
import frc.robot.commands.CompoundCommands.SuckIn;
import frc.robot.commands.CompoundCommands.SuckOut;
import frc.robot.commands.CompoundCommands.AlgaeCommands.AlgaeInfeeds.ToggleAlgaeFrontInfeedCoCommand;
import frc.robot.commands.CompoundCommands.AlgaeCommands.AlgaeInfeeds.ToggleAlgaeLollyInfeedCoCommand;
import frc.robot.commands.CompoundCommands.AlgaeCommands.AlgaeInfeeds.ReefInfeedCommands.AlgaeInfeedL1FrontSensorCoCommand;
import frc.robot.commands.CompoundCommands.AlgaeCommands.AlgaeInfeeds.ReefInfeedCommands.AlgaeInfeedL2SensorFrontCoCommand;
import frc.robot.commands.CompoundCommands.AlgaeCommands.AlgaeInfeeds.ToggleAlgaeBackInfeedCoCommand;
import frc.robot.commands.CompoundCommands.AlgaeCommands.ScoringCommands.ToggleAlgaeBargeCoCommand;
import frc.robot.commands.CompoundCommands.AlgaeCommands.ScoringCommands.AutoBargeCoCommand;
import frc.robot.commands.CompoundCommands.AlgaeCommands.ScoringCommands.ProcessorCoCommand;
import frc.robot.commands.CompoundCommands.AutoCommands.AutoCoralInfeedSensorCommand;
import frc.robot.commands.CompoundCommands.Climb.ClimbCoCommand;
import frc.robot.commands.CompoundCommands.CoralCommands.CoralInfeedCommands.CoralSourceInfeedSensorCoCommand;
import frc.robot.commands.CompoundCommands.CoralCommands.CoralInfeedCommands.ToggleCoralInfeedCoCommand;
import frc.robot.commands.CompoundCommands.CoralCommands.CoralInfeedCommands.ToggleCoralInfeedStateCoCommand;
import frc.robot.commands.CompoundCommands.CoralCommands.ScoreingCoCommands.L1CoCommands.L1FrontCoCommand;
import frc.robot.commands.CompoundCommands.CoralCommands.ScoreingCoCommands.L2CoCommands.L2ToggleCoCommand;
import frc.robot.commands.CompoundCommands.CoralCommands.ScoreingCoCommands.L3CoCommands.L3ToggleCoCommand;
import frc.robot.commands.CompoundCommands.CoralCommands.ScoreingCoCommands.L4CoCommands.L4FrontCoCommand;
import frc.robot.commands.CompoundCommands.CoralCommands.ScoreingCoCommands.L4CoCommands.L4BackCoCommand;
import frc.robot.commands.CompoundCommands.CoralCommands.ScoreingCoCommands.L4CoCommands.L4ToggleCoCommand;
import frc.robot.subsystems.*;


public class RobotContainer {

    private final SendableChooser<Command> AutoChooser;

    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final XboxController xboxController = new XboxController(1);

    /* Drive Controls */
    private final int translationAxis = Joystick.AxisType.kY.value;
    private final int strafeAxis = Joystick.AxisType.kX.value;
    private final int rotationAxis = Joystick.AxisType.kZ.value;

    private final int armAxis = XboxController.Axis.kLeftY.value;
    private final int elevatorAxisUp = XboxController.Axis.kLeftTrigger.value;
    private final int elevatorAxisDown = XboxController.Axis.kRightTrigger.value;
    private final int wristAxis = XboxController.Axis.kRightY.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, 11);
    private final JoystickButton Comp = new JoystickButton(driver, 2);
    private final JoystickButton Cancel = new JoystickButton(driver, 10);
    //Reef
    private final JoystickButton L1 = new JoystickButton(driver, 9);
    private final JoystickButton L2 = new JoystickButton(driver, 8);
    private final JoystickButton L3 = new JoystickButton(driver, 6);
    private final JoystickButton L4 = new JoystickButton(driver, 7);
    //Barge
    private final JoystickButton Barge = new JoystickButton(driver, 5);
    //Processor
    private final JoystickButton Processor = new JoystickButton(driver, 14);
    //Feeds
    private final JoystickButton CoralInfeed = new JoystickButton(driver, 3);
    private final JoystickButton AlgaeInfeed = new JoystickButton(driver, 4);
    //Shoot
    private final JoystickButton Shoot = new JoystickButton(driver, 1);

    /* POV Buttons */
    //Right
    private final POVButton top_Right = new POVButton(driver, 45);
    private final POVButton middle_Right = new POVButton(driver, 90);
    private final POVButton bottom_Right = new POVButton(driver, 135);
    private final Trigger alignLeft = new Trigger(top_Right.or(middle_Right).or(bottom_Right));
    //Top
    private final POVButton top = new POVButton(driver, 0);
    //Bottom
    private final POVButton bottom = new POVButton(driver, 180);
    //Left
    private final POVButton top_Left = new POVButton(driver, 315);
    private final POVButton middle_Left = new POVButton(driver, 270);
    private final POVButton bottom_Left = new POVButton(driver, 225);
    private final Trigger alignRight = new Trigger(top_Left.or(middle_Left).or(bottom_Left));
    //AutoAlign
    private final Trigger robotCentric = new Trigger(top_Right.or(middle_Right).or(bottom_Right).or(top_Left).or(middle_Left).or(bottom_Left));

    /* CO Controller */
    private final JoystickButton Co_Cancel = new JoystickButton(xboxController, XboxController.Button.kB.value);
    private final JoystickButton Co_ResetElevatorCount = new JoystickButton(xboxController, XboxController.Button.kA.value);
    //Co Buttons
    private final JoystickButton Co_X = new JoystickButton(xboxController, XboxController.Button.kX.value);
    private final JoystickButton Co_Start = new JoystickButton(xboxController, XboxController.Button.kStart.value);
    private final JoystickButton Co_LeftStick = new JoystickButton(xboxController, XboxController.Button.kLeftStick.value);
    //Co Bumpers
    private final JoystickButton SuckIn = new JoystickButton(xboxController, XboxController.Button.kLeftBumper.value);
    private final JoystickButton SuckOut = new JoystickButton(xboxController, XboxController.Button.kRightBumper.value);
    

    /* Subsystems */
    public static final SwerveSS s_Swerve = new SwerveSS();
    private final SensorSS s_Sensor = new SensorSS();
    private final ElevatorSS s_Elevator = new ElevatorSS();
    private final ArmSS s_Arm = new ArmSS();
    private final InfeedSS s_Infeed = new InfeedSS();
    private final WristSS s_Wrist = new WristSS();
    private final ClimberSS s_Climber = new ClimberSS();
    private final LEDSS s_LED = new LEDSS();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                s_Arm,
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean(),
                () -> alignLeft.getAsBoolean(),
                () -> alignRight.getAsBoolean(),
                () -> top.getAsBoolean(),
                () -> bottom.getAsBoolean(),
                s_Elevator
            )
        );

        s_LED.setDefaultCommand(
            new LEDDefault(
                s_LED, 
                s_Sensor, 
                s_Arm, 
                s_Swerve,
                () -> alignLeft.getAsBoolean(),
                () -> alignRight.getAsBoolean()
                )
        );

        /* Auto Commands */
        //L4 Commands
        NamedCommands.registerCommand("FrontL4", new L4FrontCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed));
        NamedCommands.registerCommand("Back L4", new L4BackCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed));
        //Shoot
        NamedCommands.registerCommand("Shoot", new ShootCoCommand(s_Arm, s_Infeed, s_Wrist, s_Elevator, s_Sensor));
        //comp
        NamedCommands.registerCommand("Comp", new CompCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed, s_Sensor));
        //Coral Infeeds
        NamedCommands.registerCommand("Infeed", new AutoCoralInfeedSensorCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed, s_Sensor)
            .until(() -> AutoCoralInfeedSensorCommand.endCommand));
        NamedCommands.registerCommand("Source Infeed", new CoralSourceInfeedSensorCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed, s_Sensor)
            .until(() -> CoralSourceInfeedSensorCoCommand.endCommand));
        //SuckBack
        NamedCommands.registerCommand("Suckback", new SuckBack(s_Wrist, s_Arm, s_Elevator, s_Infeed, s_Sensor));
        //AutoAlign Strafe
        NamedCommands.registerCommand("Strafe Left", new LaserCanAutoScore(s_Arm, s_Infeed, s_Wrist, s_Elevator, s_Swerve, s_Sensor, true));
        NamedCommands.registerCommand("Strafe Right", new LaserCanAutoScore(s_Arm, s_Infeed, s_Wrist, s_Elevator, s_Swerve, s_Sensor, false));
        //Algae Infeeds
        NamedCommands.registerCommand("Algae L1 Front", new AlgaeInfeedL1FrontSensorCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed, s_Sensor));  
        NamedCommands.registerCommand("Algae L2 Front", new AlgaeInfeedL2SensorFrontCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed, s_Sensor)); 
        //Barge  
        NamedCommands.registerCommand("Barge", new AutoBargeCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed));  
        NamedCommands.registerCommand("Barge Shot", new InfeedCommand(s_Infeed, -InfeedConstants.BARGE_SHOT, -InfeedConstants.BARGE_SHOT));
        NamedCommands.registerCommand("Barge Shot 3", new InfeedCommand(s_Infeed, -InfeedConstants.BARGE_SHOT_3, -InfeedConstants.BARGE_SHOT_3));
        //Coast
        NamedCommands.registerCommand("Coast", new InstantCommand(() -> s_Swerve.setNeutralMode(NeutralModeValue.Coast)));

        /* Autos */
        AutoChooser = new SendableChooser<Command>();
        //None
        AutoChooser.setDefaultOption("None", new PrintCommand("carson is a brick"));
        //Barge
        AutoChooser.addOption("Blue Barge", new PathPlannerAuto("Blue Barge"));
        AutoChooser.addOption("Red Barge", new PathPlannerAuto("Red Barge"));
        AutoChooser.addOption("Barge Test", new PathPlannerAuto("Barge Test"));
        //Auto Align Ground
        AutoChooser.addOption("AA Red Left Ground", new PathPlannerAuto("AA Red Left Ground"));
        AutoChooser.addOption("AA Red Right Ground", new PathPlannerAuto("AA Red Right Ground"));
        AutoChooser.addOption("AA Blue Left Ground", new PathPlannerAuto("AA Blue Left Ground"));
        AutoChooser.addOption("AA Blue Right Ground", new PathPlannerAuto("AA Blue Right Ground"));
        //Red Ground
        AutoChooser.addOption("Red Left Ground", new PathPlannerAuto("Red Left Ground"));
        AutoChooser.addOption("Red Right Ground", new PathPlannerAuto("Red Right Ground"));
        //Blue Ground
        AutoChooser.addOption("Blue Left Ground", new PathPlannerAuto("Blue Left Ground"));
        AutoChooser.addOption("Blue Right Ground", new PathPlannerAuto("Blue Right Ground"));

        SmartDashboard.putData(AutoChooser); 
        
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        /* Infeeds */
        //Coral
        CoralInfeed.onTrue(new ToggleCoralInfeedCoCommand(s_Arm, s_Infeed, s_Wrist, s_Elevator, s_Sensor));
        CoralInfeed.onTrue(new ToggleCoralInfeedStateCoCommand(s_Sensor)
            .finallyDo(() -> new InstantCommand(() -> s_Sensor.setInfeedState(true))));
        //Ground Algae
        AlgaeInfeed.onTrue(new ToggleAlgaeLollyInfeedCoCommand(s_Arm, s_Infeed, s_Wrist, s_Elevator, s_Sensor)
            .finallyDo(() -> new InstantCommand(() -> s_Sensor.setInfeedState(true))));  
        AlgaeInfeed.onTrue(new ToggleCoralInfeedStateCoCommand(s_Sensor)
            .finallyDo(() -> new InstantCommand(() -> s_Sensor.setInfeedState(true))));
        //Front Algae
        top.onTrue(new ToggleAlgaeFrontInfeedCoCommand(s_Arm, s_Infeed, s_Wrist, s_Elevator, s_Sensor)
        .finallyDo(() -> new InstantCommand(() -> s_Sensor.setInfeedState(true))));
        top.onTrue(new ToggleCoralInfeedStateCoCommand(s_Sensor)
            .finallyDo(() -> new InstantCommand(() -> s_Sensor.setInfeedState(true))));
        //Back Algae
        bottom.onTrue(new ToggleAlgaeBackInfeedCoCommand(s_Arm, s_Infeed, s_Wrist, s_Elevator, s_Sensor)
            .finallyDo(() -> new InstantCommand(() -> s_Sensor.setInfeedState(true))));
        bottom.onTrue(new ToggleCoralInfeedStateCoCommand(s_Sensor)
            .finallyDo(() -> new InstantCommand(() -> s_Sensor.setInfeedState(true))));

        /* Scoring */
        //Coral
        L1.onTrue(new L1FrontCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed));
        L2.onTrue(new L2ToggleCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed));
        L3.onTrue(new L3ToggleCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed));
        L4.onTrue(new L4ToggleCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed));
        //Barge
        Barge.onTrue(new ToggleAlgaeBargeCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed, s_Sensor, Barge));
        //Processor
        Processor.onTrue(new ProcessorCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed));
        //Shoot
        Shoot.onTrue(new ShootCoCommand(s_Arm, s_Infeed, s_Wrist, s_Elevator, s_Sensor));
        //Comp
        Comp.onTrue(new CompCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed, s_Sensor));
        //Cancel
        Cancel.onTrue(new CancelCoCommand(s_Wrist, s_Arm, s_Climber, s_Elevator, s_Infeed, s_Sensor, 
            () -> xboxController.getRawAxis(wristAxis),
            () -> -xboxController.getRawAxis(armAxis),
            () -> xboxController.getRawAxis(elevatorAxisUp),
            () -> xboxController.getRawAxis(elevatorAxisDown)
            )
        );
        /* Co Buttons */
        //Cancel
        Co_Cancel.onTrue(new CancelCoCommand(s_Wrist, s_Arm, s_Climber, s_Elevator, s_Infeed, s_Sensor,
            () -> xboxController.getRawAxis(wristAxis),
            () -> -xboxController.getRawAxis(armAxis),
            () -> xboxController.getRawAxis(elevatorAxisUp),
            () -> xboxController.getRawAxis(elevatorAxisDown)
            )
        );
        //Elevator Reset  
        Co_ResetElevatorCount.onTrue(new InstantCommand(() -> s_Elevator.resetEncoder()));
        //Climber Buttons
        Co_Start.onTrue(new InstantCommand(() -> s_Climber.setSpeed(0)));
        Co_LeftStick.onTrue(new ClimbCoCommand(s_Wrist, s_Arm, s_Elevator, s_Climber));
        Co_X.onTrue(new LaserCanAutoScore(s_Arm, s_Infeed, s_Wrist, s_Elevator, s_Swerve, s_Sensor, true));
        //Co Bumpers
        SuckIn.onTrue(new SuckIn(s_Wrist, s_Arm, s_Elevator, s_Infeed, s_Sensor));
        SuckOut.onTrue(new SuckOut(s_Wrist, s_Arm, s_Elevator, s_Infeed, s_Sensor));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return AutoChooser.getSelected();
    }
}