from PathFinder.basicPathFinder import PathFinderController
from RobotModel.MoveModel.speedModel import SpeedModel
from RobotModel.SimpleModel import SimpleModel
from UI.position import PositionMap

if __name__ == '__main__':
    controller = PathFinderController(9 / 300, 15 / 300, 3 / 300)
    move_model = SpeedModel(5, 10)
    robot = SimpleModel(move_model, controller)

    PositionMap(robot).update()