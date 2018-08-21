import Structure
def BuildDefaultuArm():
    uArm_Base=Structure.Base("uArm_Base")
    TurnTable=Structure.TurnTable("TurnTable")
    Arm1=Structure.Arm("Arm1",[[8,-16,192],[8,16,192],[-55,16,192],[8,-16,0]],[0,0,142])
    Arm1.LimInterval=[-40,90]
    Arm1.TurnedAngle=-40
    Arm2=Structure.Arm("Arm2",[[0,-10,158.8],[-60,-10,158.8],[-60,10,158.8],[0,-10,0]],[0,0,158.8])
    Arm2.LimInterval=[-20,90]
    Arm2.TurnedAngle=90
    ArmLvl=Structure.ArmLvl("ArmLvl",[[44.5,-15,35],[44.5,15,35],[-44.5,15,35],[44.5,-15,-25]],[44.5,0,0])
    TurnTable.ConnectPre(uArm_Base)
    Arm1.ConnectPre(TurnTable)
    Arm2.ConnectPre(Arm1)
    ArmLvl.ConnectPre(Arm2)
    return uArm_Base

def BuildArm( TurnAngle = 0, ArmAngle1 = -40, ArmAngle2 = 90  ):
    uArm_Base=Structure.Base("uArm_Base")
    TurnTable=Structure.TurnTable("ArmLvl")
    TurnTable.TurnedAngle = TurnAngle
    Arm1=Structure.Arm("Arm1",[[8,-16,192],[8,16,192],[-55,16,192],[8,-16,0]],[0,0,142])
    Arm1.LimInterval=[-40,90]
    Arm1.TurnedAngle = ArmAngle1
    Arm2=Structure.Arm("Arm2",[[0,-10,158.8],[-60,-10,158.8],[-60,10,158.8],[0,-10,0]],[0,0,158.8])
    Arm2.LimInterval=[0,180]
    Arm2.TurnedAngle = ArmAngle2
    ArmLvl=Structure.ArmLvl("TurnTable",[[44.5,-15,35],[44.5,15,35],[-44.5,15,35],[44.5,-15,-25]],[44.5,0,0])
    TurnTable.ConnectPre(uArm_Base)
    Arm1.ConnectPre(TurnTable)
    Arm2.ConnectPre(Arm1)
    ArmLvl.ConnectPre(Arm2)
    return uArm_Base

def BuildObstacle ( TurnAngle = 0, Center = [200, 100, 0], L = 0, W = 0, H = 0):
    Obstacle = Structure.Base("uArm_Base")
    Obstacle.Length, Obstacle.Width, Obstacle.Height = 0, 0, 0
    Obstacle.Origin = Center
    Obstacle.End = Center
    Obstacle.update()
    
    TurnTable=Structure.TurnTable("ArmLvl")
    TurnTable.Length, TurnTable.Width, TurnTable.Height = L, W, H
    TurnTable.Origin = Center
    TurnTable.End = Center
    TurnTable.ConnectPoint = Center
    TurnTable.TurnedAngle = TurnAngle
    TurnTable.ConnectPre(Obstacle)
    TurnTable.center = Center
    TurnTable.update()
    
    return Obstacle

