#!/usr/bin/python
import math
import heapq
import bisect
import sys
import collections
import functools

# Gravity in m/s
G_m = 9.80665
MUN = 1.624 / G_m * 1.5
MINMUS = .491 / G_m * 1.5
DUNA = 2.94 / G_m * 1.3
IKE = 1.10 / G_m * 1.3

LARGE_POD = 4
SMALL_POD = .8
CHAIR = .05 + .09 # With kerbal - +.09t
LARGE_CAN = 2.5
SMALL_CAN = .6
OKTO = .1
OKTO2 = .04
XENON = .25
XENON_TANK = .12
PANEL = .0175
TINY_PANEL = .005

CHUTE = .15
CHUTES = .45
TINY_STRUTS = .045
SMALL_STRUTS = .15
LARGE_STRUTS = .3
LADDER = .005
LIGHTS = .03
BATT = .005
LARGE_RCS = 3.4 + .2
SMALL_RCS = .55 + .2
# SMALL_RCS = 3.4 + .2
SMALL_DOCKING_PORT = .05
SEPARATOR = .05
STAGE_MASS = .015
#MISC = .3
# 
PAYLOAD = LARGE_CAN + CHUTES + SMALL_STRUTS + LADDER + LIGHTS + BATT
# PAYLOAD = SMALL_CAN + CHUTE + LARGE_STRUTS + LADDER + LIGHTS + BATT + SMALL_DOCKING_PORT + SMALL_RCS
# PAYLOAD = 40. # Large fuel tank
# PAYLOAD = OKTO + XENON + XENON_TANK + PANEL * 8 + BATT # Probe
# PAYLOAD = LARGE_POD + CHUTES + LIGHTS + BATT + SMALL_RCS + SMALL_DOCKING_PORT # Gemini
# PAYLOAD = CHAIR + OKTO2 + TINY_STRUTS + CHUTE # Scott Manley's tiny lander
# PAYLOAD = SMALL_POD + CHUTE + SMALL_STRUTS + LADDER + LIGHTS + BATT
# PAYLOAD = SMALL_POD + CHUTE
# PAYLOAD = 0.

TANKS = []
# Tanks are listed as tuples with wet_mass, dry_mass
# Total fuel is 8xdry mass

class Tank(collections.namedtuple('Tank', 'fuel dry')):
	@property
	def mass(self):
		return self.fuel + self.dry

for tank in range(1, 40, 1):
	TANKS.append(Tank(4.*tank, tank/2.))

for tank in (1, 2, 4):
	TANKS.append(Tank(tank, tank/8.))
	TANKS.append(Tank(2*tank, tank/4.))

TANKS.append(Tank(.078675-.015, .015))
TANKS.append(Tank(.136-.025, .025))
TANKS.sort()

SYMMETRIES = [1, 2, 3, 4, 6, 8]
SYMMETRIES.sort()

class Engine(collections.namedtuple('Engine', 'isp_atm isp_vac mass thrust name min')):
	def isp(self, atm):
		return self.isp_atm * atm + self.isp_vac * (1 - atm)
	

# Table of engines
# Impulse, mass, thrust, string
ENGINES = [
	Engine(320, 370, 1.5, 200, 'LV-45', 1),
	Engine(300, 390, .5, 50, 'LV-909', 1),
	Engine(270, 390, 2.5, 220, 'Poodle', 1),
	Engine(280, 330, 6., 1500 * .9, 'Mainsail', 1), # Run mainsails at 90% for heat
	Engine(300, 350, 4., 650, 'Skipper', 1),
	# Engine(220, 800, 2.25, 60, 'LV-N', 1), # Skipped -- Too hard to land around
	Engine(290, 320, 1.8, 240, 'Radial', 2),
	Engine(220, 290, .03, 1.5, 'LV-1', 1),
	Engine(250, 300, .18, 40, '24-77', 2),
]

ENGINES_BY_THRUST = sorted(ENGINES, key=lambda e: e.thrust)
ENGINES_BY_MASS = sorted(ENGINES, key=lambda e: e.mass)
ENGINES_BY_ATM = sorted(ENGINES, key=lambda e: e.isp_atm)
ENGINES_BY_VAC = sorted(ENGINES, key=lambda e: e.isp_vac)
MIN_ISP = min(ENGINES_BY_ATM[0].isp_atm, ENGINES_BY_VAC[0].isp_vac)
MAX_ISP = max(ENGINES_BY_ATM[-1].isp_atm, ENGINES_BY_VAC[-1].isp_vac)

Plan = collections.namedtuple('Plan', 'dv twr atm')

class StarryCollection():
	def __init__(self, plan, max_stages):
		self.best = None
		self._plan = plan
		self._heap = []
		self._best_stages = []
		self._goal = plan[-1].dv
		k1 = lambda stage: stage.mass
		k2 = lambda stage: stage.dv
		for i in xrange(max_stages + 1):
			self._best_stages.append(FancyCollection(k1, k2))

	def _accept(self, stage):
		if self.best is not None and self.best.mass < stage.mass:
			return False
		if stage.twr < stage_twr(self._plan, stage):
			return False
		if (stage.last is not None and 
			stage.last not in self._best_stages[stage.count - 2]):
			return False
		return True

	def heapentry(self, stage):
		return (ideal(stage.mass, self._goal - stage.dv), stage)

	def push(self, stage):
		if not self._accept(stage):
			return False
		if (stage.dv >= self._goal and 
			(self.best is None or self.best.mass < stage.mass)):
			self.best = stage
			for best in self._best_stages:
				best.cull(stage)
		if self._best_stages[stage.count - 1].insert(stage):
			heapq.heappush(self._heap, self.heapentry(stage))
			return True
		return False

	def pop(self):
		return heapq.heappop(self._heap)

	def __len__(self):
		return len(self._heap)

class FancyCollection():
	def __init__(self, key, key2):
		self._key = key
		self._key2 = key2
		self._keys = []
		self._items = []
		self._values = []

	def insert(self, item):
		key = self._key(item)
		key2 = self._key2(item)
		pos = bisect.bisect_left(self._keys, key)
		if pos > 0 and key2 <= self._values[pos - 1]:
			return False
		while pos < len(self._keys) and self._values[pos] <= key2:
			#print self._keys[pos], self._values[pos], "replaced by", key, key2
			del self._keys[pos]
			del self._items[pos]
			del self._values[pos]
		self._keys.insert(pos, key)
		self._values.insert(pos, key2)
		self._items.insert(pos, item)
		return True

	def key2(self, item):
		key = self._key(item)
		pos = bisect.bisect_left(self._keys, key)
		while pos < len(self._items) and key <= self._keys[pos]:
			if self._items[pos] == item:
				return self._values[pos]
		return self._key2(item)

	def cull(self, item):
		'''
		Removes any instance where key is larger than key on the given item.
		'''
		key = self._key(item)
		pos = bisect.bisect(self._keys, key)
		while pos < len(self._keys):
			del self._keys[pos]
			del self._items[pos]
			del self._values[pos]

	def __contains__(self, item):
		key = self._key(item)
		pos = bisect.bisect_left(self._keys, key)
		while pos < len(self._items) and key <= self._keys[pos]:
			if self._items[pos] == item:
				return True
			pos += 1
		return False

	def __len__(self):
		return len(self._keys)

class Stage():
	def __init__(self, tank, engine, last, nengines, plan):
		self._mass = 0.
		self._dry_mass = 0.
		self.tank = tank
		self.engine = engine
		self.last = last
		self.nengines = nengines
		self.count = 0
		stage = self
		while stage != None:
			self.count += 1
			stage = stage.last
		self.plan = plan
		self._dv = 0.

	def pretty_print(self):
		print self.mass, self.dv,
		if self.tank is None:
			print 'No tank',
		else:
			print self.tank.dry,
		if self.engine is None:
			print 'No engine'
		else:
			print str(self.nengines) + 'x' + self.engine.name,
			print self.twr
		if self.last is not None:
			self.last.pretty_print()

	@property
	def mass(self):
		if self._mass != 0.:
			return self._mass
		self._mass = self.dry_mass
		if self.tank is not None:
			self._mass += self.tank.fuel
		return self._mass

	@property
	def dry_mass(self):
		if self._dry_mass != 0.:
			return self._dry_mass
		if self.tank is not None:
			self._dry_mass += self.tank.dry
		if self.engine is not None:
			self._dry_mass += self.engine.mass * self.nengines
		if self.last is not None:
			self._dry_mass += self.last.mass
		self._dry_mass += STAGE_MASS * self.nengines
		return self._dry_mass

	@property
	def thrust(self):
		if self.engine == None:
			return 0.
		return self.engine.thrust * self.nengines

	@property
	def twr(self):
		return self.thrust / (self.mass * G_m)

	@property
	def dv(self):
		if self._dv != 0:
			return self._dv
		# print 'dv not cached'
		dv_last = 0. if self.last is None else self.last.dv
		if self.tank is None or self.engine is None:
			return dv_last
		# print 'dv_last, fuel, payload'
		# print dv_last,
		fuel = self.tank.fuel
		# print fuel,
		payload = self.dry_mass
		# print payload
		dv = 0.
		pos = bisect.bisect(self.plan, (dv_last, 0))
		excess = self.plan[pos].dv - dv_last
		fuel_used = fuel_mass(excess, self.engine.isp(self.plan[pos].atm), payload)
		# print 'excess, fuel_used'
		# print excess, fuel_used
		# print 'stage:', pos
		if fuel_used <= fuel:
			# print 'less fuel used than we had, using remainder'
			dv += delta_v(payload, fuel_used, self.engine.isp(self.plan[pos].atm))
			payload += fuel_used
			fuel -= fuel_used
			# print 'dv, payload, fuel'
			# print dv, payload, fuel
		else:
			# print 'more fuel used than we had, using all'
			dv += delta_v(payload, fuel, self.engine.isp(self.plan[pos].atm))
			fuel = 0.
			payload += fuel
			# print 'dv, payload, fuel'
			# print dv, payload, fuel

		pos += 1
		while fuel > 0 and pos < len(self.plan):
			# print 'stage:', pos
			fuel_used = fuel_mass(self.plan[pos].dv - self.plan[pos-1].dv, self.engine.isp(self.plan[pos].atm), payload)
			# print 'fuel used to satisfy stage'
			# print 'dv required, last dv, isp, payload, fuel used, fuel rem'
			# print self.plan[pos].dv - self.plan[pos - 1].dv, self.engine.isp(self.plan[pos].atm), payload, fuel_used, fuel
			if fuel_used >= fuel:
				# print 'more fuel used than we had, using all'
				fuel_used = fuel
			# print 'dv, payload, fuel'
			dv += delta_v(payload, fuel_used, self.engine.isp(self.plan[pos].atm))
			payload += fuel_used
			fuel -= fuel_used
			# print dv, payload, fuel
			pos += 1
		if fuel > 0.:
			# print 'using excess'
			dv += delta_v(payload, fuel, self.engine.isp(self.plan[-1].atm))
			# print 'dv', dv
		# print 'new dv:',
		self._dv = dv + dv_last
		# print self._dv
		return self._dv

class BoosterStage(Stage): 
	@property 
	def thrust(self): 
		if self.last == None:
			return 0.
		return self.engine.thrust * self.nengines + self.last.thrust 

	def pretty_print(self):
		print self.mass,
		if self.tank is None:
			print 'No tank',
		else:
			print self.tank.dry,
		if self.engine is None:
			print 'No engine'
		else:
			print '+' + str(self.nengines) + 'x' + self.engine.name,
			print self.twr
		if self.last is not None:
			self.last.pretty_print

class FuelStage(Stage):
	@property
	def thrust(self):
		if self.last == None:
			return 0.
		return self.last.thrust

	@property
	def engine(self):
		return self.last.engine

	def pretty_print(self):
		print self.mass, self.dv
		if self.tank is None:
			print 'No tank',
		else:
			print 'fuel'
		if self.last is not None:
			return self.last.pretty_print()

class Payload():
	def __init__(self, mass):
		self.mass = mass
		self.dv = 0.
		self.engine = None
		self.last = None
		self.tank = None
		self.thrust = 0.
		self.twr = 0.
		self.count = 1
	
	def pretty_print(self):
		print str(self.mass) + 't Payload'

def delta_v(dry_mass_k, fuel_mass_k, impulse_s):
	'''
	Returns the delta-v of the spacecraft as returned by the rocket
	equation.
	'''
	return impulse_s * G_m * math.log((dry_mass_k + fuel_mass_k) / dry_mass_k)

def fuel_mass(delta_v, impulse_s, payload_k):
	'''
	Return the total fuel mass to achieve a given delta-v with a given
	impulse. The value returned does NOT include the mass of the payload
	'''
	return (payload_k * math.exp(delta_v/(impulse_s * G_m))) - payload_k

def ideal(payload_k, delta_v):
	'''
	Return the lowest possible mass to accelerate the payload to a given delta-v. 
	Includes the weight of the fuel, payload, "perfect" engine, and STAGE_MASS
	'''
	min_weight = ENGINES_BY_MASS[0].mass
	adj_payload = payload_k + STAGE_MASS + min_weight;
	ideal_mass = fuel_mass(delta_v, MAX_ISP, payload_k + STAGE_MASS + min_weight)
	return adj_payload + ideal_mass

def stage_twr(plans, stage):
	''' Return the maximum TWR listed in delta_vs between the min and max delta-vs '''
	if (stage.dv == 0.):
		return 0.
	left = bisect.bisect(plans, (stage.last.dv, 0))
	if left == len(plans):
		return plan[-1].twr
	right = bisect.bisect(plans, (stage.dv, 0)) + 1
	if len(plans[left:right]) == 0:
		print left, stage.last.dv, right, stage.dv, plans
	return max(plan.twr for plan in plans[left:right])

def find(payload_k, given_plans, max_stages=10):
	'''
	Finds a way to get the payload up by delta-v.
	Anything that doesn't match the requrested TWR ratios will be ignored.
	Remember: These assume kerbin weights; Use Body/Kerbin for other bodies!!
	These are in terms of stages; The first TWR should be the lifter, etc. etc.
	Optimize for fuel mass
	@param delta_vs: A sequence of delta-vs + TWRs that must be achieved
	The first stage of the mission must be the last delta-v parameter
	'''
	def next_states(last, dv_goal, plan, heap):
		''' Returns a list of the next possible states from this stage, These will be of a form (score (mass, dv, tank, engine, next)) '''
		next_stages = []
		for engine in ENGINES:
			fuel_max = fuel_mass(dv_goal - last.dv, 
				min(engine.isp_atm, engine.isp_vac), 
				last.mass + (engine.mass + STAGE_MASS) * SYMMETRIES[-1] + TANKS[-1].dry)
			tank_idx = bisect.bisect_right(TANKS, (fuel_max, 0))
			for tank in TANKS[0:tank_idx + 1]:
				for num_engines in SYMMETRIES:
					if num_engines < engine.min:
						continue
					heap.push(Stage(tank, engine, last, num_engines, plan))

	dv_sum = 0.
	plans = []
	for plan in given_plans:
		dv_sum += plan.dv
		plans.append(Plan(dv_sum, plan.twr, plan.atm))
		
	dv_goal = plans[-1].dv
	print dv_goal
	start = Payload(payload_k)
	heap = StarryCollection(plans, max_stages)
	heap.push(start)
	best = None # Best solution
	solved = False
	print heap
	while solved == False and len(heap) > 0:
		head = heap.pop()
		print head[1].mass, head[1].dv
		if head[1].dv > dv_goal:
			# We have a solution
			if best is not None:
				if best[1].mass > head[1].mass:
					# We have a better solution
					print "best:", best[1].mass, '->', head[1].mass
					best = head
				elif best[1].mass < head[0]:
					# best is THE solution
					solved = True
			else:
				best = head
				print "best:", best[1].mass
		elif head[1].count < max_stages:
			next_states(head[1], dv_goal, plans, heap)
				
	if solved or best is not None:
		return best[1].mass, best[1]
	else:
		print "Not solved!"
		return None, None

print PAYLOAD

MINMUS_PLAN = [
	Plan(960, 0., 0.), 
	Plan(2*80+300*2 + 100, MINMUS, 0.), 
	Plan(960, 0., 0.)
]
GEMINI_PLAN = [(240, MINMUS)]
JOOL_PLAN = [(2630, 0), (2630, 0), (950+965, 0)]
SYNC = [(1000, 0)]
# MUNAR_LANDER_PLAN = [(860., MUN), (860., MUN)] # Too little fuel to land, just enough to head back.
MUN_PLAN = [
	Plan(800., 0, 0.), 
	Plan(2200., MUN, 0.), 
	Plan(800., 0, 0.),
]
DUNA_PLAN = [(1673, 0), (1380*2,  DUNA), (915*2, IKE),  (1702, 0)]
DUNAR_LANDER_PLAN = [(1380*2, DUNA), (915*2, IKE)]
DUNA_AND_BACK = [(1673, 0), (1702, 0)]
#LKO = [(2000, 1.3), (2500, 1.5)]
LKO = [
	Plan(2000., 1.1, .3),
	Plan(1500., 1.4, .7),
	Plan(1000., 1.5, .9),
]
	
JETO = [(1000, 1.5), (3500, 1.5)]
SSTO = [(4500, 1.5)]
# PLAN = MUN_PLAN + LKO
# PLAN = MINMUS_PLAN + LKO
# PLAN = DUNAR_LANDER_PLAN
PLAN = MUN_PLAN + LKO
#, (2000, 1.5), (2500, 1.5)]
#DUNA_PLAN = desired_stages = [(1380*2 + 1673 + 915*2, DUNA), (1702, 0), (2000, 1.5), (2500, 1.5)]
mass, stages = find(PAYLOAD, PLAN, 5)
print "Lander:", mass
stages.pretty_print()
sys.exit(0)
stage_print(stages, PLAN)
# print PLAN
bisect.insort(ENGINES, 
	(800, 2.25, 60, 'LV-N', 1), # Skipped -- Too hard to work around
)
bisect.insort(ENGINES_BY_MASS,
	(2.25, (800, 2.25, 60, 'LV-N', 1)) # Skipped -- Too hard to work around
)
bisect.insort(ENGINES_BY_THRUST,
	(60, (800, 2.25, 60, 'LV-N', 1)) # Skipped -- Too hard to work around
)
MIN_ISP = ENGINES[0][0]
MAX_ISP = ENGINES[-1][0]
mass, stages = find(mass, DUNA_AND_BACK + LKO, 7)
print "Lifter:"
stage_print(stages, LKO)
