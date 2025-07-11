# SkyScout Command Examples

Natural language commands you can use with SkyScout.

## Basic Commands

### Search Missions
- "Find all red vehicles in the parking lot"
- "Search for people wearing safety helmets using zigzag pattern"
- "Look for the blue delivery truck with spiral search"
- "Find any boats in the harbor"
- "Quick search for missing person in the park"

### Inspection Missions
- "Inspect the solar panels for damage"
- "Check the roof for missing tiles" (automatically uses circle pattern)
- "Examine the north wall for cracks with detailed inspection"
- "Look closely at the antenna on building 3"
- "Do a 360-degree inspection of the water tower"

### Counting Missions
- "Count all cars in the parking lot"
- "How many people are on the construction site?"
- "Count the shipping containers in section A"
- "Tell me how many trees are in the field"
- "Count vehicles in the parking lot using grid pattern"

### Mapping Missions
- "Map the perimeter of the warehouse"
- "Create a 3D map of the construction site"
- "Survey the entire parking area"
- "Map out the locations of all red objects"
- "Create detailed terrain map with multiple altitude passes"

### Delivery Missions
- "Deliver the medical supplies to GPS 37.7749, -122.4194"
- "Drop package at the main entrance"
- "Transport emergency supplies to the accident site"
- "Deliver tools to the construction crew on the roof"

### Patrol Missions
- "Patrol the property perimeter for security"
- "Do continuous security sweeps of the parking lot"
- "Monitor the fence line every 30 minutes"
- "Watch for intruders along the north boundary"

### Survey Missions
- "Survey the agricultural field for crop health"
- "Check the vineyard for disease using zigzag pattern"
- "Assess storm damage across the entire property"
- "Analyze field moisture levels with systematic coverage"

### Emergency Response
- "Emergency response to accident at main gate"
- "Urgent: Search for injured hiker at GPS coordinates"
- "Rapid assessment of fire damage in sector 3"
- "Emergency medical supply delivery to remote location"

## Advanced Commands

### Multi-criteria Search
- "Find all white vans near the loading dock"
- "Search for people not wearing hard hats in the construction zone"
- "Locate damaged solar panels on the south-facing roofs"
- "Find red cars or trucks in visitor parking"

### Weather-Aware Commands
- "Search for missing person if weather permits"
- "Inspect roof but abort if wind exceeds safety limits"
- "Survey field with extended range in calm conditions"

### Pattern-Specific Commands
- "Use zigzag pattern to search the cornfield"
- "Circle the building to inspect all sides"
- "Follow the polygon shape of the property boundary"
- "Spiral out from the last known position"

### Template-Based Commands
- "Do a quick search for the missing dog"
- "Standard building inspection of the office complex"
- "Agricultural survey of the north field"
- "Security patrol around the warehouse"

## Flight Pattern Examples

### Grid Search
"Search for debris in a grid pattern over the field"
- Systematic back-and-forth coverage
- Best for thorough area search
- Adjustable spacing between passes

### Spiral Search
"Do a spiral search for the lost drone starting from here"
- Expanding outward from center
- Good for searching from last known position
- Efficient for uncertain target location

### Perimeter Patrol
"Fly around the perimeter looking for fence damage"
- Follows boundary of defined area
- Ideal for security and inspection
- Can be repeated for continuous monitoring

### Zigzag Pattern
"Search the field using zigzag pattern for maximum coverage"
- Tighter spacing than grid
- More efficient for large areas
- Better for linear features

### Circle/Orbit
"Circle the communications tower for full inspection"
- 360-degree view of target
- Multiple altitudes for complete coverage
- Smooth, continuous movement

### Polygon
"Follow the property boundaries for survey"
- Custom shape definition
- Matches irregular areas
- Precise coverage control

### Waypoint Mission
"Check these three locations for the delivery truck: loading dock, parking lot B, and the service road"
- Direct path between points
- Efficient for known locations
- Minimal flight time

## Example Mission Outputs

### Search Mission with Weather Check
**Command**: "Find all people wearing orange safety vests if weather is safe"

**Generated Mission**:
```json
{
  "mission_type": "search",
  "target_description": "people wearing orange safety vests",
  "flight_pattern": "zigzag",
  "parameters": {
    "altitude": 30,
    "speed": 6,
    "coverage_area": {
      "center": {"lat": 37.7749, "lon": -122.4194},
      "radius": 150
    }
  },
  "safety": {
    "geofence": {
      "max_altitude": 100,
      "max_distance": 200
    },
    "rtl_battery_threshold": 25,
    "weather_check": true,
    "max_wind_speed": 10.0
  }
}
```

### Emergency Delivery Mission
**Command**: "Emergency medical supply delivery to accident site"

**Generated Mission**:
```json
{
  "mission_type": "emergency",
  "target_description": "accident site for medical delivery",
  "flight_pattern": "waypoints",
  "parameters": {
    "altitude": 80,
    "speed": 10,
    "coverage_area": {
      "center": {"lat": 37.7749, "lon": -122.4194},
      "radius": 50
    }
  },
  "safety": {
    "geofence": {
      "max_altitude": 120,
      "max_distance": 2000
    },
    "rtl_battery_threshold": 20,
    "weather_check": false,
    "max_wind_speed": 15.0
  }
}
```

## Mission Templates

SkyScout includes pre-configured templates for common operations:

1. **quick_search** - Rapid spiral search for missing persons/objects
2. **building_inspection** - 360° structural examination
3. **perimeter_patrol** - Security boundary monitoring
4. **delivery_direct** - Point-to-point package transport
5. **emergency_response** - Rapid deployment with relaxed safety limits
6. **agricultural_survey** - Systematic field analysis
7. **parking_count** - Vehicle enumeration in parking areas
8. **3d_mapping** - Multi-altitude terrain modeling

Use them like: "Do a building inspection on the warehouse"

## Tips for Better Results

1. **Be specific**: "red sedan" works better than "car"
2. **Include patterns**: "search using zigzag" for better coverage
3. **Mention weather**: "if weather permits" for safety
4. **Use landmarks**: "between building A and the parking lot"
5. **Specify urgency**: "emergency" relaxes some constraints
6. **Add constraints**: "stay below 50 meters altitude"

## Weather Integration

Commands automatically check weather conditions unless it's an emergency:
- Wind speed limits (default 10 m/s)
- Visibility requirements (minimum 3 km)
- Precipitation checks
- Temperature constraints

Override with: "emergency" or "ignore weather"

## Coming Soon

- Voice commands
- Multi-drone coordination commands
- Follow-me modes
- Real-time mission modification
- Geofence drawing on map
