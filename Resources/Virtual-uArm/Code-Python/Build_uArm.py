import Structure
def BuildDefaultuArm():
    uArm_Base=Structure.Base("uArm_Base")
    TurnTable=Structure.TurnTable("TurnTable")
    Arm1=Structure.Arm("Arm1",[[8,-16,192],[8,16,192],[-55,16,192],[8,-16,0]],[0,0,142])
    Arm1.LimInterval=[-40,90]
    Arm1.TurnedAngle=-40
    Arm2=Structure.Arm("Arm2",[[0,-10,158.8],[-60,-10,158.8],[-60,10,158.8],[0,-10,0]],[0,0,158.8])
    Arm2.LimInterval=[0,180]
    Arm2.TurnedAngle=180
    ArmLvl=Structure.ArmLvl("ArmLvl",[[44.5,-15,35],[44.5,15,35],[-44.5,15,35],[44.5,-15,-25]],[44.5,0,0])
    TurnTable.ConnectPre(uArm_Base)
    Arm1.ConnectPre(TurnTable)
    Arm2.ConnectPre(Arm1)
    ArmLvl.ConnectPre(Arm2)
    return uArm_Base
