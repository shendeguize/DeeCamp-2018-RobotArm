## Here is a virtual platform of uArm-Swift-Pro-UP1300.
### Already Included:
1. Structure of uArm
2. Visualization of uArm model using matplotlib

### To be included:
1. Manipulation of uArm model
2. Path Direction
3. Contact of model and other entities

### Code Structures:
#### Interfaces：
1. uArm model:
```
    import Build_uArm
    uArm_Base=Build_uArm.BuildDefaultuArm()
```
2. visualize the uArm model:
```
    from Visualization import ShowState
    ShowState(uArm_Base,Target=None,ShowAxes=Ture,xLim=[-500,500],yLim=[-500,500],Height=500)
```
#### Structures：
1. uArm model:
```
    Base->TurnTable->Arm1->Arm2->ArmLvl->Hand(Optional)
    func Turn2Angle(Angle)
    func TurnDeltaAnge(Angle) 
```
2. Visualization:
```
    func ShowState(Base,Target=None,ShowAxes=Ture,xLim=[-500,500],yLim=[-500,500],Height=500)
    func ShowOnePart(Part,Target=None,ShowAxes=Ture,xLim=[-500,500],yLim=[-500,500],Height=500)
```
### Log：
1. 2018.07.27-uArm model
2. 2018.07.28-Visualization
