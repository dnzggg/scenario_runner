import py_trees
import carla


from basic_scenario import BasicScenario
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import WaypointFollower

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.tools.scenario_helper import get_waypoint_in_distance


class NewScenario(BasicScenario):
    """
    Some documentation on NewScenario
    :param world is the CARLA world
    :param ego_vehicles is a list of ego vehicles for this scenario
    :param config is the scenario configuration (ScenarioConfiguration)
    :param randomize can be used to select parameters randomly (optional, default=False)
    :param debug_mode can be used to provide more comprehensive console output (optional, default=False)
    :param criteria_enable can be used to disable/enable scenario evaluation based on test criteria (optional, default=True)
    :param timeout is the overall scenario timeout (optional, default=60 seconds)
    """

    # some ego vehicle parameters
    # some parameters for the other vehicles
    timeout = 120

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):
        """
        Initialize all parameters required for NewScenario
        """

        self.timeout = timeout
        self._map = CarlaDataProvider.get_map()
        self._first_vehicle_location = 25
        self._first_vehicle_speed = 10
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)

        # Call constructor of BasicScenario
        super(NewScenario, self).__init__(
          "NewScenario",
          ego_vehicles,
          config,
          world,
          debug_mode,
          criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._first_vehicle_location)
        transform = waypoint.transform
        transform.location.z += 0.5
        first_vehicle = CarlaDataProvider.request_new_actor('vehicle.tesla.cybertruck', transform)
        self.other_actors.append(first_vehicle)

    def _create_behavior(self):
        """
        Setup the behavior for NewScenario
        """

        driving_to_next_intersection = py_trees.composites.Parallel(
            "DrivingTowardsIntersection",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        driving_to_next_intersection.add_child(WaypointFollower(self.other_actors[0], self._first_vehicle_speed))
        driving_to_next_intersection.add_child(InTriggerDistanceToJunction())

        sequence = py_trees.composites.Sequence("Sequence Behavior")

        return sequence

    def _create_test_criteria(self):
        """
        Setup the evaluation criteria for NewScenario
        """