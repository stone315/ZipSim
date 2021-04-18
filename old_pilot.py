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

    # LIDAR constants are used to distinguish Tree object and Delivery Site
    self.DELIVERY_SITE_LIDAR_RADIUS = 0.5
    self.TREE_LIDAR_RADIUS = 3.0

    # Vehicle speed: Assume this constant is know according to the document.
    # This can be removed for advanced agent
    self.VEHICLE_AIRSPEED = 30.0

    # Data from parent process
    self.wind = [0.0, 0.0]
    self.timestamp = 0
    self.recovery = [0.0, 0.0]
    self.samples = [0.0 for i in range(31)]

    # List the Trees and Delivery Sites object
    self.delivery_sites = []
    self.trees = []
    self.unknown = []

    # Last drop
    self.lastDrop = 3000

  def getAction(self):
    
    lateral_airspeed, drop_package = self.recovery[1] - self.wind[1], 0
    targetPos = 0

    self.removePastSite()
    self.removePastTree()
    self.searchObject()

    if len(self.unknown):
      self.unknown.sort(key = lambda p: p[0])
      targetPos = min(12, max(-12.0, self.unknown[0][1]))
    
    if len(self.delivery_sites):
      self.delivery_sites.sort(key = lambda p: p[0])
      site = self.delivery_sites[0]

      if len(self.unknown) == 0 or self.unknown[0][0] < site[0]:
        targetPos = site[1]

    message = 'normal'
    if len(self.trees):
      dangerousZones = []
      for tree in self.trees:
        if tree[0] > self.recovery[0] - 30:
          dangerousZones.append(tree[1])
      
      possibleZone = [ max(-25, (-30.0 - self.wind[1] + self.recovery[1])), min(25, (30.0 - self.wind[1] + self.recovery[1]))]
      if self.isSafe(self.recovery[1], dangerousZones) and self.recovery[0] - self.disToXY(self.recovery, [15, self.samples[15]])[0] > 15:
        if targetPos > self.recovery[1]:
          direction = 1
        else:
          direction = -1
      
        targetPos = self.findClosestSafe(self.recovery[1], targetPos, dangerousZones, direction)
        message = 'normal'
        
      elif self.recovery[0] - self.disToXY(self.recovery, [15, self.samples[15]])[0] > 15 or self.samples[15] == 0 :
        targetPos = self.recovery[1]
        message = 'lock'
      
      else:
        safeZone = [ self.findSafe(self.recovery[1] - 1, dangerousZones, possibleZone, -1), self.findSafe(self.recovery[1] + 1, dangerousZones, possibleZone, 1)]
      
      
        if safeZone[0] == None and safeZone[1] == None:
          if abs(targetPos - possibleZone[0]) < abs(targetPos - possibleZone[1]):
            targetPos = -30
          else:
            targetPos = 30
        elif safeZone[0] != None and safeZone[1] == None:
          targetPos = -30
        elif safeZone[0] == None and safeZone[1] != None:
          targetPos = 30
        else:
          if abs(safeZone[0] - self.recovery[1]) < abs(safeZone[1] - self.recovery[1]):
            targetPos = -30
          else:
            targetPos = 30
        message = 'danger'
    
    #print( self.recovery, targetPos, self.trees, self.wind[1], message, file = sys.stderr)
    lateral_airspeed = min(30.0, max(-30.0, - targetPos + self.recovery[1] - self.wind[1]))
    drop_package = self.boolDeploy(lateral_airspeed)

    return lateral_airspeed, drop_package

  # Group the points into object
  def searchObject(self):

    #collection of all object including Trees and 
    objects = []
    single_object = []
    pervious = 0
    for i in range(2,len(self.samples)-2):
      dis = self.samples[i]
      
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
      self.identifyObject(objects)
      self.cleanTrees()
    

  def identifyObject(self, objects):
    maxPoint = 2 * math.sin(math.radians(0.5))
    self.unknown = []
    # Loop over the objects
    for single_object in objects:

      # if there is two or more points
      maxDis = max(single_object, key = lambda p: p[1])
      
              
      
      if len(single_object) > math.ceil(1.0 / (maxDis[1] * maxPoint)) + 1:
        
        point = self.findCenter(self.recovery, single_object[0], single_object[-1])
        
        if point != False:
        
          if len(self.trees) == 0:

            self.trees.append(point)
            self.removeSite(point)
            
            if showTrees:
              print("Tree is", self.trees, self.recovery[0], file = sys.stderr)
          
          elif point[0] < self.recovery[0] - 30:
          
            if not self.checkRepeatedTree(point) :
              self.trees.append(point)
              self.removeSite(point)
              
              if showTrees:
                print("Tree is", self.trees, self.recovery[0], file = sys.stderr)
      
      
      elif len(single_object) < math.floor( 6.0/ (maxPoint * maxDis[1])) -1:

        point = self.disToXY(self.recovery, single_object[0])

        if len(self.delivery_sites) == 0 and (not self.isRepeated(point, self.trees, 10)):
          self.delivery_sites.append(point)
          
          if showSites:
            print("Site is", self.delivery_sites, file = sys.stderr)
          
        else:

          if (not self.isRepeated(point, self.delivery_sites, 10)) and (not self.isRepeated(point, self.trees, 10)):
            self.delivery_sites.append(point)
            
            if showSites:
              print("Site is", self.delivery_sites, file = sys.stderr)
          
      else:
      
        point = self.disToXY(self.recovery, single_object[0])
        if (not self.isRepeated(point, self.delivery_sites, 10)):
          
          if len(single_object[0]) > 1 and point[0]< self.recovery[0] - 30:
            point = self.findCenter(self.recovery, single_object[0], single_object[-1])
            if point != False:
              if not self.checkRepeatedTree(point):
                self.unknown.append(self.disToXY(self.recovery, single_object[0]))


  def disToXY(self, position, distance):
    x = position[0] - distance[1]* math.sin(math.radians(distance[0] + 75))
    y = position[1] + distance[1]* math.cos(math.radians(distance[0] + 75)) % 50
    y = (y - 50 if y > 25 else y)

    return [x,y]

  def isRepeated(self, point, objects, radius):
    for object in objects:
      if (point[0] - object[0])**2 + (point[1] - object[1])**2 < radius**2:
        return True
    return False
  
  
  def checkRepeatedTree(self, point):
    for i in range(len(self.trees)):
      tree = self.trees[i]
      if (point[0] - tree[0])**2 + (point[1] - tree[1])**2 < 8**2:
        self.trees[i] = point
        return True
    return False
  
  def cleanTrees(self):
    temp = []
    for tree in self.trees:
      repeated = False
      for point in temp:
        if (point[0] - tree[0])**2 + (point[1] - tree[1])**2 < 7**2:
          repeated = True
      
      if repeated == False:
        temp.append(tree)
        
    self.trees = temp
  
  def findCenter(self, position, point1, point2):
    p1 = self.disToXY(position, point1)
    p2 = self.disToXY(position, point2)
    
    if abs( p1[1] - p2[1]) > 25:
      if p1[1] < 0:
        p1[1] += 50
      else:
        p2[1] += 50
    
    
    p3 = [ (p1[0] + p2[0])/2 , (p1[1] + p2[1])/2]
    
    if p1[0] == p2[0]:
      y = p3[1]
      if 3**2 - (p2[1] - y)**2 < 0:
        return False
      x = p2[0] - math.sqrt(3**2 - (p2[1] - y)**2)
      y = y % 50
      y = (y - 50 if y > 25 else y)
      
      return [x,y]
      
    slope = (p1[1] - p2[1])/(p1[0] - p2[0])
    
    D = p2[1] - p3[1] - p3[0]/slope
    
    a = 1 + 1/(slope**2)
    b = 2 * ( D/slope -  p2[0])
    c = D**2 + p2[0]**2 - 9
    
    delta = b*b - 4*a*c
    if delta < 0:
      return False
      
    x = (-b + math.sqrt(delta))/ (2*a)
    y = ((x - p3[0])/(-slope) + p3[1]) % 50
    y = (y - 50 if y > 25 else y)
    
    return [round(x,3),round(y,3)]

  def removeSite(self, point):
    temp = []
    for site in self.delivery_sites:
      if (point[0] - site[0])**2 + (point[1] - site[1])**2 < 10**2:
        temp.append(site)
    
    self.delivery_sites = temp

  def removePastSite(self):
    newSites = []
    for site in self.delivery_sites:
      if site[0] < self.recovery[0] + 5:
        newSites.append(site)
    
    self.delivery_sites = newSites

  def removePastTree(self):
    newTrees = []
    for tree in self.trees:
      if tree[0] < self.recovery[0] + 10:
       newTrees.append(tree)
       
    self.trees = newTrees


  def boolDeploy(self, lateral_airspeed):

    if self.delivery_sites == [] or self.recovery[0] > self.lastDrop - 10:
      return 0
    site = self.delivery_sites[0]
    dis = math.sqrt( (self.recovery[0] - site[0])**2 + (self.recovery[1] - site[1])**2)
    speed = math.sqrt((self.wind[0] + self.VEHICLE_AIRSPEED)**2 + (self.wind[1] + lateral_airspeed)**2)

    if 0.4 * speed  <= dis and 0.4 * speed + 5 >= dis:
      self.lastDrop = self.recovery[0]
      self.delivery_sites = self.delivery_sites[1:]
      return 1
    return 0
    
  def findSafe(self, targetPos, dangerousZones, possibleZone, direction):
    
    if targetPos < possibleZone[0] or targetPos > possibleZone[1]:
      return None
    
    flag = self.isSafe(targetPos, dangerousZones)
        
    if flag:
      return targetPos
    else:
      return self.findSafe(targetPos + 1 * direction, dangerousZones, possibleZone, direction)
      
  def isSafe(self, Pos, dangerousZones):
    for zone in dangerousZones:
      if Pos >= max(-25, zone - 5) and Pos <= min(25, zone + 5):
        return False
    return True
    
  def findClosestSafe(self, curPos, targetPos, dangerousZones, direction):
    
    while self.isSafe(curPos, dangerousZones) and (curPos - targetPos) * direction <= 0:
      curPos += (1 * direction)
    
    return curPos - (1 * direction)

# ReflexAgent: This agent is similiar to Native agent but it uses current sensors data to plan the path
class ReflexAgent(NativeAgent):
  def getAction(self):
    lateral_airspeed, drop_package = self.recovery[1] - self.wind[1], 0
    targetPos = [self.recovery[0] - 50, 0]

    self.removePastSite()
    self.removePastTree()
    self.searchObject()

    if len(self.unknown):
      self.unknown.sort(key = lambda p: p[0])
      targetPos = [self.recovery[0] - 50, min(12, max(-12.0, self.unknown[0][1]))]
    
    if len(self.delivery_sites):
      self.delivery_sites.sort(key = lambda p: p[0])
      site = self.delivery_sites[0]

      if len(self.unknown) == 0 or self.unknown[0][0] < site[0]:
        targetPos = site

    forwardSpeed = 30 + self.wind[0]
    maxAngle = min(15.0 , math.degrees(math.atan( (30 + self.wind[1])/forwardSpeed)))
    minAngle = max(-15.0 , math.degrees(math.atan( (-30 + self.wind[1])/forwardSpeed)))
    
    maxAngle = 15
    minAngle = -15

    targetAngle = round(max(minAngle, min(maxAngle, math.degrees(math.atan( (targetPos[1] - self.recovery[1])/forwardSpeed)))))
      
    leftAngle = self.safeAngle(targetAngle, minAngle, maxAngle, -1)
    rightAngle = self.safeAngle(targetAngle, minAngle, maxAngle, 1)
    
    if leftAngle == None and rightAngle == None:
      targetAngle = 15
      
    elif leftAngle == None and rightAngle != None:
      targetAngle = rightAngle
    
    elif leftAngle != None and rightAngle == None:
      targetAngle = leftAngle
    else:
      if abs(leftAngle) < abs(rightAngle):
        targetAngle = leftAngle
      else:
        targetAngle = rightAngle
    

    
    lateral_airspeed = -1 * math.tan(math.radians(targetAngle))*forwardSpeed - self.wind[1]
    #print( self.recovery, self.samples, lateral_airspeed + self.wind[1] - self.recovery[1], leftAngle, rightAngle, file = sys.stderr)
    drop_package = self.boolDeploy(lateral_airspeed)

    return lateral_airspeed, drop_package
    
  def safeAngle(self, targetAngle, minAngle, maxAngle, direction):
  
    if self.samples[15 - targetAngle] >= 30  or self.samples[15 - targetAngle] == 0:
      return targetAngle

    if direction == 1 and targetAngle + 1 <= maxAngle:
      return self.safeAngle(targetAngle + 1, minAngle, maxAngle, direction)
    if direction == -1 and targetAngle - 1 >= minAngle:
      return self.safeAngle(targetAngle - 1, minAngle, maxAngle, direction)
      
    return None

# TestAgent: This is used to do some functional test
class TestAgent(Agent):
  def __init__(self):

    # LIDAR constants are used to distinguish Tree object and Delivery Site
    self.DELIVERY_SITE_LIDAR_RADIUS = 0.5
    self.TREE_LIDAR_RADIUS = 3.0

    # Vehicle speed: Assume this constant is know according to the document.
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
    self.direction = 'left'

    # create boundary
    self.recovery_site_min = -7.0
    self.recovery_site_max = 7.0

  def getAction(self):
    
    if self.direction == 'left' and self.recovery[1] < self.recovery_site_min:
      self.direction = 'right'
    
    if self.direction == 'right' and self.recovery[1] > self.recovery_site_max:
      self.direction = 'left'

    lateral_airspeed = (10.0 - self.wind[1] if self.direction == 'left' else -10.0 - self.wind[1])
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
  agent_type = parser.parse_args().agent[0]
  agent = createAgent(agent_type)
  
  showSites = parser.parse_args().sites
  showTrees = parser.parse_args().trees
  
  while True:
  

    if not agent.update():
      break
    agent.writeData(agent.getAction())

