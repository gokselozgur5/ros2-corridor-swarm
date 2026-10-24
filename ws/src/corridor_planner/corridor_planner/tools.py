"""Tool schemas the planner offers Claude. The callables behind them live in the ROS node."""

SCHEMAS = [
    {'name': 'list_vehicles', 'description': 'All vehicles with mode, position and corridor status.',
     'input_schema': {'type': 'object', 'properties': {}, 'additionalProperties': False}, 'strict': True},
    {'name': 'get_vehicle', 'description': 'Detail of one vehicle: fix, pose, velocity, battery, mode, active goal.',
     'input_schema': {'type': 'object', 'properties': {'vehicle': {'type': 'string'}}, 'required': ['vehicle'],
                      'additionalProperties': False}, 'strict': True},
    {'name': 'patrol', 'description': 'Start a patrol: laps along the corridor polygon. laps 0 means until stopped.',
     'input_schema': {'type': 'object',
                      'properties': {'vehicle': {'type': 'string'}, 'laps': {'type': 'integer'},
                                     'clockwise': {'type': 'boolean'}},
                      'required': ['vehicle', 'laps', 'clockwise'], 'additionalProperties': False}, 'strict': True},
    {'name': 'goto', 'description': 'Send a vehicle to one WGS84 point inside the corridor.',
     'input_schema': {'type': 'object',
                      'properties': {'vehicle': {'type': 'string'}, 'lat': {'type': 'number'}, 'lon': {'type': 'number'}},
                      'required': ['vehicle', 'lat', 'lon'], 'additionalProperties': False}, 'strict': True},
    {'name': 'stop', 'description': 'Cancel whatever the vehicle is doing and hold.',
     'input_schema': {'type': 'object', 'properties': {'vehicle': {'type': 'string'}}, 'required': ['vehicle'],
                      'additionalProperties': False}, 'strict': True},
    {'name': 'geofence_events', 'description': 'Recent geofence events (exit, enter, warn), newest first.',
     'input_schema': {'type': 'object', 'properties': {'limit': {'type': 'integer'}}, 'required': ['limit'],
                      'additionalProperties': False}, 'strict': True},
]
