# Spatial-Exploration-Simul

## Inefficiency in Movement and Ray Casting: 
  The current implementation moves one step at a time, performing ray casting at each step, leading to prolonged exploration times. Omitting consecutive ray casts may result in missing critical information.​
  
Complexity of Directional Ray Casting Code:
  The code for ray casting varies significantly with direction, resulting in increased complexity. Attempts to simplify this have been challenging due to varying ray starting offsets depending on direction.​

Ray Dispersion Upon Obstacle Collision: 
  To achieve comprehensive visual information with the minimal necessary rays, the system disperses rays upon detecting obstacles. However, this approach further complicates the ray casting process.​

Discrepancy with Real Sensor Behavior: 
  During development, it became evident that actual sensors do not perform such exhaustive ray casting. Therefore, while the current implementation serves as a valuable simulation for game-like vision systems, it may not be practical for real-world applications.
