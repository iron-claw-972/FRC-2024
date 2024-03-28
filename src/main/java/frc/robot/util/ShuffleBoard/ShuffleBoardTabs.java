package frc.robot.util.ShuffleBoard;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public abstract class  ShuffleBoardTabs {
    protected ShuffleboardTab tab;

    public abstract void createEntries();

    public abstract void update();

    public double truncate(double value){
        return Math.floor(value*1000)/1000;
    }
}
