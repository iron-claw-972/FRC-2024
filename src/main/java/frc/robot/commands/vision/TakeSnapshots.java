package frc.robot.commands.vision;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TakeSnapshots extends CommandBase {
  private int m_snapshots;
  private int m_snapshotsToTake;
  private Timer m_timer;
  private double m_delay;
  private NetworkTableEntry m_snapshotEntry;
  private int m_value = 0;

  public TakeSnapshots(NetworkTableEntry snapshotEntry){
    this(snapshotEntry, 0.5);
  }
  public TakeSnapshots(NetworkTableEntry snapshotEntry, double delay){
    this(snapshotEntry, delay, Integer.MAX_VALUE);
  }
  public TakeSnapshots(NetworkTableEntry snapshotEntry, double delay, int snapshotsToTake){
    m_snapshotEntry = snapshotEntry;
    m_delay = delay;
    m_snapshotsToTake = snapshotsToTake;
    m_timer = new Timer();
  }

  @Override
  public void initialize(){
    m_snapshots = 0;
    m_timer.restart();
    m_value = 0;
  }
  @Override
  public void execute(){
    if(m_timer.advanceIfElapsed(m_delay)){
      m_value = 1 - m_value;
      m_snapshotEntry.setNumber(m_value);
      if(m_value==1){
        m_snapshots++;
        System.out.println(m_snapshots+" snapshots taken");
      }
    }
  }
  @Override
  public void end(boolean interrupted){
    if(interrupted && m_snapshotsToTake < Integer.MAX_VALUE){
      System.out.println(m_snapshots*100/m_snapshotsToTake+"% of snapshots taken");
    }
  }
  @Override
  public boolean isFinished(){
    return m_snapshots >= m_snapshotsToTake;
  }
}
