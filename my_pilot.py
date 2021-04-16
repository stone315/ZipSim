import struct
import sys
import argparse
import math
import threading
  
TELEMETRY_STRUCT = struct.Struct(">Hhffb31B")
COMMAND_STRUCT = struct.Struct(">fB3s")
showSites = False
showTrees = False


#Abstact Class
class Agent:

  def __init__(self):
    # Data from parent process
    self.wind = [0.0, 0.0]
    self.timestamp = 0
    self.recovery = [0.0, 0.0]
    self.samples = [0.0 for i in range(31)]


  # Update the value from ReadDate
  def update(self):
  
    telemetry = self.readData()
    if telemetry == False:
      return False
      
    timestamp, recovery_x_error, wind_vector_x, wind_vector_y, recovery_y_error = telemetry[:5]
    lidar_samples = telemetry[5:]

    self.timestamp = timestamp
    self.wind = [wind_vector_x, wind_vector_y]
    self.recovery = [recovery_x_error, recovery_y_error]
    self.samples = lidar_samples
    return True

  def getAction(self):
    pass

  # Read the data from parent process and unpack the data
  def readData(self):
    telemetry = sys.stdin.buffer.read(TELEMETRY_STRUCT.size)
    if len(telemetry) != TELEMETRY_STRUCT.size:
      return False
    telemetry = TELEMETRY_STRUCT.unpack(telemetry)


    return telemetry

  # Return the control input to parent process
  def writeData(self, action):
    lateral_airspeed, drop_package = action
    data = COMMAND_STRUCT.pack(lateral_airspeed, int(drop_package), b'3s')
    sys.stdout.buffer.write(data)
    sys.stdout.flush()


# Native Agent: The first attempt solution
# Most of the decision is native, not optimized.
class NativeAgent(Agent):

  def __init__(self):

    # CONSTANTS given by the zip sim chanllege pdf
    # Screen width
    self.WIDTH = 25.0
    
    # LIDAR constants are used to distinguish Tree object and Delivery Site
    self.DELIVERY_SITE_LIDAR_RADIUS = 0.5
    self.TREE_LIDAR_RADIUS = 3.0
    self.SITE_RADIUS = 5.0

    # Vehicle speed: Assume this constant is know according to the document.
    # This can be removed for advanced agent
    self.VEHICLE_AIRSPEED = 30.0
    
    # Maximum and Minimum wind speed
    self.MAXIMUM_WINDSPEED = 20.0
    self.MINIMUM_WINDSPEED = -20.0



    # Data from parent process
    self.wind = [0.0, 0.0]
    self.timestamp = 0
    self.recovery = [0.0, 0.0]
    self.samples = [0.0 for i in range(31)]

    # List the Trees and Delivery Sites object
    self.delivery_sites = []
    self.trees = []
    self.unknown = []

    # Last drop. The intial distance can be anything > 2000
    self.lastDrop = 3000

  # Decide the lateral airspeed and the need of dropping a package for next time step
  def getAction(self):
    
    # Decide the lateral airspeed and the need of dropping a package for next time step
    lateral_airspeed, drop_package = self.recovery[1] - self.wind[1], 0
    targetPos = 0

    # Update the objects
    # remove Passed objects and search for new objects
    self.removePastSite()
    self.removePastTree()
    self.searchObject()

    # If there is an unknown object, sort the objects and move closer to the first object
    if len(self.unknown):
      self.unknown.sort(key = lambda p: p[0])
      targetPos = min(12, max(-12.0, self.unknown[0][1]))
    
    # If there is a delivery site, sort the site by vertical distance
    if len(self.delivery_sites):
      self.delivery_sites.sort(key = lambda p: p[0])
      site = self.delivery_sites[0]
      
      # Compare the distance of the unknown object and the distance of the site.
      # Move toward the closer one
      if len(self.unknown) == 0 or self.unknown[0][0] < site[0]:
        targetPos = site[1]

    # Check the Trees position. If it is too closed to vehicle, find the dangerous zone and avoid the zone.
    if len(self.trees):
    
      dangerousZones = []
      for tree in self.trees:
        if tree[0] > self.recovery[0] - self.VEHICLE_AIRSPEED:
          dangerousZones.append(tree[1])
      
      
      # Possible zone is the vehicle can travel in next 1s
      possibleZone = [ max(-self.WIDTH, (-self.VEHICLE_AIRSPEED - self.wind[1] + self.recovery[1])), min(self.WIDTH, (self.VEHICLE_AIRSPEED - self.wind[1] + self.recovery[1]))]
      
      # Case 1: if is safe and front sensor does not detect obstacle
      if self.isSafe(self.recovery[1], dangerousZones) and self.recovery[0] - self.disToXY(self.recovery, [15, self.samples[15]])[0] > 15:
        if targetPos > self.recovery[1]:
          direction = 1
        else:
          direction = -1
      
        targetPos = self.findClosestSafe(self.recovery[1], targetPos, dangerousZones, direction)
      
      # Case 2: if is near the dangerous zone but the front sensor does not detect obstacle. Therefore, it is still safe to move forward.
      elif self.recovery[0] - self.disToXY(self.recovery, [15, self.samples[15]])[0] > 15 or self.samples[15] == 0 :
        targetPos = self.recovery[1]
      
      # Case 3: if it is in dangerous zone, then vehicle should find a safe zone and move toward the safe zone
      else:
        safeZone = [ self.findSafe(self.recovery[1] - 1, dangerousZones, possibleZone, -1), self.findSafe(self.recovery[1] + 1, dangerousZones, possibleZone, 1)]
      
      
        if safeZone[0] == None and safeZone[1] == None:      # If no safe zone is detected, the vehicle turns to the maximum angle
          if abs(targetPos - possibleZone[0]) < abs(targetPos - possibleZone[1]):  # Choose the position that is closest to target
            targetPos = -self.VEHICLE_AIRSPEED
          else:
            targetPos = self.VEHICLE_AIRSPEED
            
        elif safeZone[0] != None and safeZone[1] == None:   # If only left is safe, choose left
          targetPos = -self.VEHICLE_AIRSPEED
          
        elif safeZone[0] == None and safeZone[1] != None:   # If only right is safe, choose right
          targetPos = self.VEHICLE_AIRSPEED
          
        else:
        
          if abs(safeZone[0] - self.recovery[1]) < abs(safeZone[1] - self.recovery[1]):  # If both are safe, choose safe zone that is closest to current position
            targetPos = -self.VEHICLE_AIRSPEED
          else:
            targetPos = self.VEHICLE_AIRSPEED
    

    lateral_airspeed = min(self.VEHICLE_AIRSPEED, max(-self.VEHICLE_AIRSPEED, - targetPos + self.recovery[1] - self.wind[1]))
    drop_package = self.boolDeploy(lateral_airspeed)

    return lateral_airspeed, drop_package

  # Group the points into object
  def searchObject(self):

    # object list is collection of all object including Trees and Sites and single object is the collection of all points for one object
    objects = []
    single_object = []
    
    pervious = 0
    
    # Loop over 2nd to 30th samples
    for i in range(2,len(self.samples)-2):
      dis = self.samples[i]
      
      # If an object is detected, compare the sample to pervious sample and group the samples if they are closed.
      if dis > 0:
        if pervious == 0 or abs(dis - pervious) <= 2:
          single_object.append([i,dis])
        else:
          objects.append(single_object)
          single_object = [[i, dis]]
      else:
        if len(single_object) > 0:
          objects.append(single_object)
          single_object = []
      
      pervious = dis
      self.identifyObject(objects) # Call identifyObject function to identify the type of object ( Tree/Delivery Site)
      self.cleanTrees() # Remove duplicated Tree
    
  # Identify the type of object (Tree/Delivery Site/ Unknown)
  def identifyObject(self, objects):
  
    accuracy = 2 * math.sin(math.radians(0.5)) # accuracy = 0.01745 is the measurement accuracy which is proportional to the distance
    self.unknown = []
    
    # Loop over the objects
    for single_object in objects:

      # find maximum distance among the points in a single object
      maxDis = max(single_object, key = lambda p: p[1])
      
              
      # If number of measurement is greater than the possible number of measuring a delivery site, then it is a tree.
      if len(single_object) > math.ceil((2 * self.DELIVERY_SITE_LIDAR_RADIUS)/ (maxDis[1] * accuracy)) + 1:
        
        point = self.findCenter(self.recovery, single_object[0], single_object[-1]) # Find the center of tree
        
        # If the center can be computed, check the duplication in the list and then add the tree to the list
        if point != False:
        
          if len(self.trees) == 0:

            self.trees.append(point)
            self.removeSite(point)
            
            if showTrees:
              print("Tree is", self.trees, self.recovery[0], file = sys.stderr)
          
          elif point[0] < self.recovery[0] - 30: # If the object is too closed to vehicle, it can affect the accuracy of computed circle's center
          
            if not self.checkRepeatedTree(point) :
              self.trees.append(point)
              self.removeSite(point)
              
              if showTrees:
                print("Tree is", self.trees, self.recovery[0], file = sys.stderr)
      
      
      # If number of measurement is less than the possible number of measuring a  tree, then it is a delivery site.
      elif len(single_object) < math.floor( (2 * self.TREE_LIDAR_RADIUS)/ (accuracy * maxDis[1])) -1:

        point = self.disToXY(self.recovery, single_object[0])

        # Make sure the object does not overlap other objects
        if len(self.delivery_sites) == 0 and (not self.isRepeated(point, self.trees, 2 * self.SITE_RADIUS)):
          self.delivery_sites.append(point)
          
          if showSites:
            print("Site is", self.delivery_sites, file = sys.stderr)
          
        else:

          # Make sure the object does not overlap other object
          if (not self.isRepeated(point, self.delivery_sites, 2 * self.SITE_RADIUS)) and (not self.isRepeated(point, self.trees, 2 * self.SITE_RADIUS)):
            self.delivery_sites.append(point)
            
            if showSites:
              print("Site is", self.delivery_sites, file = sys.stderr)
      
      # Cannot identify the type of object. Let it be unknown.
      else:
      
        point = self.disToXY(self.recovery, single_object[0])
        
        # Make sure the object does not overlap site and the distance is at least 30m
        if (not self.isRepeated(point, self.delivery_sites, 2 * self.SITE_RADIUS)) and len(single_object[0]) > 1 and point[0]< self.recovery[0] - 30:
        
            point = self.findCenter(self.recovery, single_object[0], single_object[-1])
            
            if point != False:
              if not self.checkRepeatedTree(point):
                self.unknown.append(self.disToXY(self.recovery, single_object[0]))


  # Convert distance into X-Y coordinate
  def disToXY(self, position, distance):
    x = position[0] - distance[1]* math.sin(math.radians(distance[0] + 75))
    y = position[1] + distance[1]* math.cos(math.radians(distance[0] + 75)) % 50
    y = (y - 50 if y > self.WIDTH else y)

    return [x,y]

  # Check wheather hte point is near the object
  def isRepeated(self, point, objects, radius):
    for object in objects:
      if (point[0] - object[0])**2 + (point[1] - object[1])**2 < radius**2:
        return True
    return False
  
  # Check wheather the point is near the tree in the trees list. Update the center of trees using the new point
  def checkRepeatedTree(self, point):
    for i in range(len(self.trees)):
      tree = self.trees[i]
      if (point[0] - tree[0])**2 + (point[1] - tree[1])**2 < 8**2:
        self.trees[i] = point
        return True
    return False
  
  # Check wheather two same trees register in the list. Update the list if it has repeated trees.
  def cleanTrees(self):
    newTrees = []
    for tree in self.trees:
      repeated = False
      for point in newTrees:
        if (point[0] - tree[0])**2 + (point[1] - tree[1])**2 < 7**2:
          repeated = True
      
      if repeated == False:
        newTrees.append(tree)
        
    self.trees = newTrees
  
  def findCenter(self, position, point1, point2):
  
    # Convert the distance to (x,y) coordinate
    p1 = self.disToXY(position, point1)
    p2 = self.disToXY(position, point2)
    
    # Adject the coordinate if the tree is across the boundary of y-axis
    if abs( p1[1] - p2[1]) > self.WIDTH:
      if p1[1] < 0:
        p1[1] += 50
      else:
        p2[1] += 50
    
    # Find the mid point between p1 and p2
    p3 = [ (p1[0] + p2[0])/2 , (p1[1] + p2[1])/2]
    
    if p1[0] == p2[0]: # If X_p1 = X_p2, the slope goes to infinite. To prevent this, motify the computation.
      y = p3[1]
      
      if self.TREE_LIDAR_RADIUS**2 - (p2[1] - y)**2 < 0: # No center point is found. It could due to the computational error.
        return False
        
      x = p2[0] - math.sqrt(self.TREE_LIDAR_RADIUS**2 - (p2[1] - y)**2) # Solve the circle equation (x1 -x2)^2 + (y1 - y2)^2 = r^2. Choose negative delta.
      y = y % 50
      y = (y - 50 if y > self.WIDTH else y)
      
      return [x,y]
      
    slope = (p1[1] - p2[1])/(p1[0] - p2[0])
    
    D = p2[1] - p3[1] - p3[0]/slope
    
    a = 1 + 1/(slope**2)
    b = 2 * ( D/slope -  p2[0])
    c = D**2 + p2[0]**2 - 9
    delta = b*b - 4*a*c
    
    if delta < 0: # No center point is found. It could due to the computational error.
      return False
      
    x = (-b + math.sqrt(delta))/ (2*a)  # Solve the circle equation (x1 -x2)^2 + (y1 - y2)^2 = r^2 using quadratic formula. Choose negative delta.
    y = ((x - p3[0])/(-slope) + p3[1]) % 50
    y = (y - 50 if y > self.WIDTH else y)
    
    return [round(x,3),round(y,3)]

  # remove repeated site
  def removeSite(self, point):
    newSites = []
    for site in self.delivery_sites:
      if (point[0] - site[0])**2 + (point[1] - site[1])**2 < (2 * self.SITE_RADIUS) **2:   # the distance between sites is at least 10m that is 2 * radius of site
        newSites.append(site)
    
    self.delivery_sites = newSites

  # remove the passed site from the delivery_sites list
  def removePastSite(self):
    newSites = []
    for site in self.delivery_sites:
      if site[0] < self.recovery[0] + self.SITE_RADIUS:  # make sure vehicle pass its radius
        newSites.append(site)
    
    self.delivery_sites = newSites

  # remove the passed tree from the trees list
  def removePastTree(self):
    newTrees = []
    for tree in self.trees:
      if tree[0] < self.recovery[0] + 10: # make sure vehicle pass 10m since there is computational error
       newTrees.append(tree)
       
    self.trees = newTrees

  # Determine should the package be dropped
  def boolDeploy(self, lateral_airspeed):

    # If there is no delivery site or the last package was dropped within 10m, no package should be dropped
    if self.delivery_sites == [] or self.recovery[0] > self.lastDrop - 2 * self.SITE_RADIUS:
      return 0
      
    # find the closest site
    site = self.delivery_sites[0]
    
    # find the distance between vehicle and site, and the vehicle's current velocity
    dis = math.sqrt( (self.recovery[0] - site[0])**2 + (self.recovery[1] - site[1])**2)
    speed = math.sqrt((self.wind[0] + self.VEHICLE_AIRSPEED)**2 + (self.wind[1] + lateral_airspeed)**2)

    # Check the timing of dropping package. If (d - r1)/v < ts < (d - r2)/v holds, a package is dropped
    if 0.4 * speed  <= dis and 0.4 * speed + self.SITE_RADIUS >= dis:
      self.lastDrop = self.recovery[0]
      self.delivery_sites = self.delivery_sites[1:]
      return 1
    return 0
    
  # Find the safe zone
  def findSafe(self, targetPos, dangerousZones, possibleZone, direction):
    
    # If target position is outside of the possible zone, it means no safe zone can be found
    if targetPos < possibleZone[0] or targetPos > possibleZone[1]:
      return None
    
    # Return target position if it is safe. Otherwise, increase or decrease by 1 according to the direction
    flag = self.isSafe(targetPos, dangerousZones)
        
    if flag:
      return targetPos
    else:
      return self.findSafe(targetPos + 1 * direction, dangerousZones, possibleZone, direction)
      
      
  # Check the safety of position
  def isSafe(self, Pos, dangerousZones):
    for zone in dangerousZones:
      if Pos >= max(-self.WIDTH, zone - 5) and Pos <= min(self.WIDTH, zone + 5): # Choose 5m instead of 3m, the radius of trees. It is due to the computational error.
        return False
    return True
    
  # Find the safe zone that is closest to target position
  def findClosestSafe(self, curPos, targetPos, dangerousZones, direction):
    
    while self.isSafe(curPos, dangerousZones) and (curPos - targetPos) * direction <= 0:
      curPos += (1 * direction)
    
    return curPos - (1 * direction)

# ReflexAgent: This agent is similiar to Native agent but it uses current sensors data to find the safe zone
class ReflexAgent(NativeAgent):

  # Decide the lateral airspeed and the need of dropping a package for next time step
  def getAction(self):
  
    # Defualt action and target position: maintain 0m in y-axis
    lateral_airspeed, drop_package = self.recovery[1] - self.wind[1], 0
    targetPos = [self.recovery[0] - (self.MAXIMUM_WINDSPEED + self.VEHICLE_AIRSPEED), 0]

    # Update the objects
    # remove Passed objects and search for new objects
    self.removePastSite()
    self.removePastTree()
    self.searchObject()

    # If there is an unknown object, sort the objects and move closer to the first object
    if len(self.unknown):
      self.unknown.sort(key = lambda p: p[0])
      targetPos = [self.recovery[0] - 50, min(12, max(-12.0, self.unknown[0][1]))]
    
    # If there is a delivery site, sort the site by vertical distance
    if len(self.delivery_sites):
      self.delivery_sites.sort(key = lambda p: p[0])
      site = self.delivery_sites[0]

      # Compare the distance of the unknown object and the distance of the site.
      # Move toward the closer one
      if len(self.unknown) == 0 or self.unknown[0][0] < site[0]:
        targetPos = site

    
    forwardSpeed = self.VEHICLE_AIRSPEED + self.wind[0]
    
    # the maximum angle and the minimum angle are angles that sensors can measure (between -15 degree and 15 degree)
    maxAngle = 15
    minAngle = -15

    # Convert the target position into angle. If the angle is greater than maximum angle (15 degree) or minimum angle (-15 degree),
    # set the target angle to be the closest possible angle.
    targetAngle = math.degrees(math.atan( (targetPos[1] - self.recovery[1])/forwardSpeed))
    targetAngle = round(max(minAngle, min(maxAngle, targetAngle)))
      
    # Find the safe angle on the left side and on the side side
    leftAngle = self.safeAngle(targetAngle, minAngle, maxAngle, -1)
    rightAngle = self.safeAngle(targetAngle, minAngle, maxAngle, 1)
    
    
    
    if leftAngle == None and rightAngle == None:   # If no safe angle is detected, the vehicle turns to the maximum angle
      targetAngle = 15
    
    elif leftAngle == None and rightAngle != None: # If only right angle is safe, the vehicle turns to right angle
      targetAngle = rightAngle
    
    elif leftAngle != None and rightAngle == None: # If only left angle is safe, the vehicle turns to left angle
      targetAngle = leftAngle
      
    else:
      
      if abs(leftAngle) < abs(rightAngle):  # If both sides are safe, choose the minimum turning angle
        targetAngle = leftAngle
      else:
        targetAngle = rightAngle
    

    # Convert from target angle to lateral airspeed
    lateral_airspeed = -1 * math.tan(math.radians(targetAngle))*forwardSpeed - self.wind[1]
    
    # Check should the vehicle deploy a package
    drop_package = self.boolDeploy(lateral_airspeed)

    return lateral_airspeed, drop_package
    
  # Compute the safe angle that can guarantee the safety of vehicle if it follows the direction
  def safeAngle(self, targetAngle, minAngle, maxAngle, direction):
  
    # If the measurement distance is greater than 30 m or infinite ( reading is 0), the vehicle is safe
    if self.samples[15 - targetAngle] >= 30  or self.samples[15 - targetAngle] == 0:
      return targetAngle

    # If the angle is not safe, increase or decrease its angle by 1 according to the direction
    if direction == 1 and targetAngle + 1 <= maxAngle:
      return self.safeAngle(targetAngle + 1, minAngle, maxAngle, direction)
    if direction == -1 and targetAngle - 1 >= minAngle:
      return self.safeAngle(targetAngle - 1, minAngle, maxAngle, direction)
      
    # No safe angle is found
    return None

# TestAgent: This is used to do some functional test
class TestAgent(Agent):
  def __init__(self):

    # LIDAR constants are used to distinguish Tree object and Delivery Site
    self.DELIVERY_SITE_LIDAR_RADIUS = 0.5
    self.TREE_LIDAR_RADIUS = 3.0

    # Vehicle speed: Assume this constant is known according to the document.
    # This can be removed for advanced agent
    self.VEHICLE_AIRSPEED = 30.0

    # Data from parent process
    self.wind = [0.0, 0.0]
    self.timestamp = 0
    self.recovery = [0.0, 0.0]
    self.samples = [0.0 for i in range(31)]

    # List the Trees and Delivery Sites object
    self.delivery_sites = []
    self.Trees = []

    # Current direction (toward left or right)
    self.direction = -1

    # Create boundary
    self.lowerBound = -7.0
    self.upperBound = 7.0

  # Decide the lateral airspeed and the need of dropping a package for next time step
  def getAction(self):
    
    # When the vehicle hits the boundary, it changes its direction
    if self.direction == -1 and self.recovery[1] < self.lowerBound:
      self.direction = 1
    
    if self.direction == 1 and self.recovery[1] > self.upperBound:
      self.direction = -1


    # Vehicle moves at a constant speed 10 m/s
    lateral_airspeed = (10.0 - self.wind[1] if self.direction == -1 else -10.0 - self.wind[1])
    lateral_airspeed = min(self.VEHICLE_AIRSPEED, max(-self.VEHICLE_AIRSPEED, lateral_airspeed))
    drop_package = 0

    return lateral_airspeed, drop_package

# Create a specific type of agent
def createAgent(type = 'Native'):
  if type == 'Native':
    return NativeAgent()
  if type == 'Reflex':
    return ReflexAgent()
  if type == 'Test':
    return TestAgent()

  print("No agent is found. Use default agent: Native.", file = sys.stderr)
  return NativeAgent() #if type is not vilad 



'''
 TO DO:
 1. Path Planning (prevent the obstacle)
 2. Path Planning (search for the delivery site)  #This maybe less concern if the sensors are powerful enough
 3. Distinguish Tree and delivery site
 4. Determine the time to deploy package
'''

if __name__ == "__main__":

  parser = argparse.ArgumentParser(description='Option')
  parser.add_argument('-a', '--agent', nargs=argparse.REMAINDER, help='choose agent Type. Default: Native')
  parser.add_argument('--sites', action="store_true", help ='show all sites')
  parser.add_argument('--trees', action="store_true", help ='show all trees')
  
  if parser.parse_args().agent:
    agent_type = parser.parse_args().agent[0]
  else:
    agent_type = '' # If no selection, use Native agent
    
  #Create agent with agent_type
  agent = createAgent(agent_type)
  
  showSites = parser.parse_args().sites
  showTrees = parser.parse_args().trees
  
  while True:
  

    if not agent.update():
      break
      
    agent.writeData(agent.getAction())

