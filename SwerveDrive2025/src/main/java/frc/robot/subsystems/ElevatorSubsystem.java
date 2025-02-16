    package frc.robot.subsystems;
    import edu.wpi.first.wpilibj.Timer;
    import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
    import edu.wpi.first.wpilibj2.command.Command;
    import edu.wpi.first.wpilibj2.command.Commands;
    import edu.wpi.first.wpilibj2.command.SubsystemBase;
    import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

    import java.util.function.DoubleSupplier;

    import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
    import com.ctre.phoenix6.configs.Slot0Configs;
    import com.ctre.phoenix6.configs.TalonFXConfiguration;
    import com.ctre.phoenix6.controls.*;
    import com.ctre.phoenix6.hardware.*;
    import com.ctre.phoenix6.signals.GravityTypeValue;
    import com.ctre.phoenix6.signals.InvertedValue;
    //import com.ctre.phoenix6.motorcontrol.ControlMode;
    import com.ctre.phoenix6.signals.NeutralModeValue;

    public class ElevatorSubsystem extends SubsystemBase{
        // Declares the two motors controling the elevator
        private final TalonFX backElevator = new TalonFX(14, "canivore");
        private final TalonFX frontElevator = new TalonFX(15, "canivore");
        // Defines motionMagic stuff(allows the elevator to move smoothly)
        private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0).withSlot(0);
        // Defines a static voltage for the elevator
        private final VoltageOut voltageOut = new VoltageOut(0);

        // Initializes the motors and controller
        public ElevatorSubsystem(CommandXboxController controller) {
            // Back Elevator follows front elevator
            backElevator.setControl(new Follower(frontElevator.getDeviceID(), false));
            // Make sure elevator is at base postion each time rio restarts.
            frontElevator.setPosition(0, 100);

            // Applies necessary configuration for each motor
            configureMotor(frontElevator);
            configureMotor(backElevator);
        }

        // Configures settings for motor at start
        private void configureMotor(TalonFX motor) {
            TalonFXConfiguration config = new TalonFXConfiguration();
            // PID
            config.Slot0.kP = 1;        
            config.Slot0.kI = 0;        
            config.Slot0.kD = 0;   
            // Feedforward
            config.Slot0.kG = 0.2;        
            config.Slot0.kS = 0.2;
            config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
            // Motion Magic
            config.MotionMagic.MotionMagicCruiseVelocity = 100;
            config.MotionMagic.MotionMagicAcceleration = 120;
            // Current limits
            config.CurrentLimits.SupplyCurrentLimit = 40;
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            // Limits for the height of the elevator
            config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.5;
            config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 40.5;
            config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            // Brakes when there is no output
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            // Applies configuration in 1 second
            motor.getConfigurator().apply(config, 1);
        }

        // Sets the voltage of the elevator
        public Command setOpenLoop(DoubleSupplier voltage) {
            return Commands.run(() -> frontElevator.setControl(voltageOut.withOutput(voltage.getAsDouble())) , this);
        }
        // Holds the position of the elevator at a certain rotation
        // The rio will assume position 0 is when you click start, so always reset the elevator to the bottom
        public Command setCloseLoop(DoubleSupplier position) {
            return Commands.run(() -> frontElevator.setControl(motionMagic.withPosition(position.getAsDouble())), this);
        }

        @Override
        public void periodic() {
            // Displays telemetry about the elevators position and power
            SmartDashboard.putNumber("elevator position", frontElevator.getPosition().getValueAsDouble());
            SmartDashboard.putNumber("elevator applied", frontElevator.getMotorVoltage().getValueAsDouble());
        }

        /** Sets the position of both motors.
         * @param rotations is the number of rotations it will hold at.
         */
        public void setPositions(double rotations){
            frontElevator.setPosition(rotations);
        }

        public void reset() {
            frontElevator.setPosition(0);
        }

        // Returns the position of the elevator
        public double getPosition(){
            return frontElevator.getPosition().getValueAsDouble();
        }
    }
