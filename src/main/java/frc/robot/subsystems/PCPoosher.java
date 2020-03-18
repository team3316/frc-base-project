package frc.robot.subsystems;

import com.team3316.kit.DBugSubsystem;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.utils.Utils;

/**
 * Ballplow
 */
public class PCPoosher extends DBugSubsystem {

    private DoubleSolenoid _shifter;

    public PCPoosher() {
        this._shifter = (DoubleSolenoid) Utils.getBean("PCPoosher.shifter");
    }

    public enum PCPoosherState {
        IN(Value.kReverse), OUT(Value.kForward), INTERMIDIET(Value.kOff);

        private Value _value;

        private PCPoosherState(Value value) {
            this._value = value;
        }

        public Value getValue() {
            return this._value;
        }
    }

    public void setState(PCPoosherState wantedState) {
        this._shifter.set(wantedState.getValue());
    }

    public PCPoosherState getState() {
        if (this._shifter.get()  == PCPoosherState.IN.getValue()) return PCPoosherState.IN;
        if (this._shifter.get()  == PCPoosherState.OUT.getValue()) return PCPoosherState.OUT;
        return PCPoosherState.OUT;
    }

    @Override
    public void initDefaultCommand() {
        // TODO Auto-generated method stub

    }

    @Override
    public void displayTestData() {
        // TODO Auto-generated method stub

    }

    @Override
    public void displayMatchData() {
        // TODO Auto-generated method stub

    }

    @Override
    public void displayCommands() {
        // TODO Auto-generated method stub

    }
}