package frc.robot.util;


import frc.robot.ChoreoPoses;

public enum AlgaePositions {
  AB(ChoreoPoses.AB, CoralPositions.A, CoralPositions.B),
  CD(ChoreoPoses.CD, CoralPositions.C, CoralPositions.D),
  EF(ChoreoPoses.EF, CoralPositions.E, CoralPositions.F),
  GH(ChoreoPoses.GH, CoralPositions.G, CoralPositions.H),
  IJ(ChoreoPoses.IJ, CoralPositions.I, CoralPositions.J),
  KL(ChoreoPoses.KL, CoralPositions.K, CoralPositions.L);


  public MaybeFlippedPose2d m_pose;
  public CoralPositions m_coralLeft;
  public CoralPositions m_coralRight;


  AlgaePositions(MaybeFlippedPose2d pose, CoralPositions coralLeft, CoralPositions coralRight) {
    m_pose = pose;
    m_coralLeft = coralLeft;
    m_coralRight = coralRight;
  }
}