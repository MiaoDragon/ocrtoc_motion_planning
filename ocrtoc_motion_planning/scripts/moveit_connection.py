import moveit_commander

robot = moveit_commander.RobotCommander()
group_arm_name = "robot_arm"
group = moveit_commander.MoveGroupCommander(group_arm_name)
scene = moveit_commander.PlanningSceneInterface()
