# SkyScout Command Examples

Natural language commands you can use with SkyScout.

## Basic Commands

### Search Missions
- "Find all red vehicles in the parking lot"
- "Search for people wearing safety helmets"
- "Look for the blue delivery truck"
- "Find any boats in the harbor"

### Inspection Missions
- "Inspect the solar panels for damage"
- "Check the roof for missing tiles"
- "Examine the north wall for cracks"
- "Look closely at the antenna on building 3"

### Counting Missions
- "Count all cars in the parking lot"
- "How many people are on the construction site?"
- "Count the shipping containers in section A"
- "Tell me how many trees are in the field"

### Mapping Missions
- "Map the perimeter of the warehouse"
- "Create an overview of the construction site"
- "Survey the entire parking area"
- "Map out the locations of all red objects"

## Advanced Commands

### Multi-criteria Search
- "Find all white vans near the loading dock"
- "Search for people not wearing hard hats in the construction zone"
- "Locate damaged solar panels on the south-facing roofs"

### Conditional Actions
- "If you find more than 5 cars, expand the search area"
- "Search until you find the missing hiker or cover 100 meters"
- "Count vehicles but ignore motorcycles"

### Area-specific Commands
- "Search the northeast quadrant for equipment"
- "Inspect only the panels installed last month"
- "Count cars in visitor parking only"

## Flight Pattern Examples

### Grid Search
"Search for debris in a grid pattern over the field"

### Spiral Search
"Do a spiral search for the lost drone starting from here"

### Perimeter Patrol
"Fly around the perimeter looking for fence damage"

### Waypoint Mission
"Check these three locations for the delivery truck: loading dock, parking lot B, and the service road"

## Example Mission Output

**Command**: "Find all people wearing orange safety vests"

**Generated Mission**:
```json
{
  "mission_type": "search",
  "target_description": "people wearing orange safety vests",
  "flight_pattern": "grid",
  "parameters": {
    "altitude": 30,
    "speed": 5,
    "coverage_area": {
      "center": {"lat": 37.4419, "lon": -122.1430},
      "radius": 100
    }
  }
}
```

## Tips for Better Results

1. **Be specific**: "red sedan" works better than "car"
2. **Include context**: "near the entrance" helps narrow search
3. **Use landmarks**: "between building A and the parking lot"
4. **Specify urgency**: "quickly scan" vs "detailed inspection"

## Coming Soon

- Voice commands
- Multi-drone coordination commands
- Follow-me modes
- Custom area definitions
