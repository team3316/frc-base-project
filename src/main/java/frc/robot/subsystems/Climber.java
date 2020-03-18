package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team3316.kit.DBugSubsystem;
import com.team3316.kit.motors.DBugSparkMax;
import com.team3316.kit.motors.DBugTalon;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.Utils;

public class Climber extends DBugSubsystem {

    private DBugSparkMax _climberSpark;
    private DBugTalon _elevatorArm;

    public Climber() {
        this._elevatorArm = (DBugTalon) Utils.getBean("climb.elevatorTalon");
        this._climberSpark = (DBugSparkMax) Utils.getBean("climb.climberSpark");

        this._elevatorArm.setInverted(Constants.Climber.MotorControllers.ClimberTalon.inverted);
        this._elevatorArm.configVoltageCompSaturation(Constants.Climber.MotorControllers.ClimberTalon.voltageCompensation);
        this._elevatorArm.enableVoltageCompensation(true);
        this._elevatorArm.configNeutralDeadband(Constants.Climber.MotorControllers.ClimberTalon.deadband);
        this._elevatorArm.setNeutralMode(NeutralMode.Brake);

        this._climberSpark.setInverted(Constants.Climber.MotorControllers.ClimberSpark.inverted);
    }

    public enum ClimberState {
        PULL(Constants.Climber.ClimberState.PULL.percent), NONE(Constants.Climber.ClimberState.NONE.percent);

        private double _percent;

        private ClimberState(double percent) {
            this._percent = percent;
        }

        public double getPercent() {
            return this._percent;
        }

        public void setPercent(double percent) {
            this._percent = percent;
        }
    }

    public void setElevatorPercentage(double percentage) {
        this._elevatorArm.set(ControlMode.PercentOutput, percentage);
    }

    /**
     * this function sets the percent output between minus 1 and 1 for the
     * climberTalon and the climberVictor
     * 
     * @pre 0 < percent < 1
     * @param percent - value to set the climber motor controllers to between 0 to 1
     * 
     */
    public void setClimberPercent(double percent) {
        this._climberSpark.set(percent);
    }

    /**
     * This function sets the climber's state
     * 
     * @param state - <code>ElevatorState</code> to set the climber to
     */
    public void setClimberState(ClimberState state) {
        this.setClimberPercent(state.getPercent());
    }
    
    public ClimberState getClimberState() {
        double percent = this._climberSpark.get();
        if (Utils.isInNeighborhood(percent, ClimberState.PULL.getPercent(), Constants.Climber.ClimberState.tolerance))
            return ClimberState.PULL;
        return ClimberState.NONE;
    }


    @Override
    public void displayTestData() {
        // SmartDashboard.putBoolean("Climber elevator motor inverted", this._elevatorTalon.getInverted());
        // SmartDashboard.putBoolean("Climber talon inverted", this._climberTalon.getInverted());
        // SmartDashboard.putBoolean("Climber victor inverted", this._climberVictor.getInverted());
        // SmartDashboard.putNumber("Climber elevator current", this.getElevatorCurrent());
        // SmartDashboard.putNumber("Climber talon current", this.getClimberCurrent());
    }

    @Override
    public void displayMatchData() {
        // TODO Auto-generated method stub
    }

    @Override
    public void displayCommands() {
        SmartDashboard.putData("climb", new InstantCommand(new Runnable(){
        
            @Override
            public void run() {
                Robot.climber.setClimberPercent(0.1);
            }
        }));

        SmartDashboard.putData("stop climbing", new InstantCommand(new Runnable(){
        
            @Override
            public void run() {
                Robot.climber.setClimberPercent(0.0);
            }
        }));
    }

    @Override
    public void initDefaultCommand() {
        // TODO Auto-generated method stub
    }
}