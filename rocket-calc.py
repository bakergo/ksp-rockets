#!/usr/bin/python
import math
import heapq
import bisect
import sys

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
SEPARATOR = .015
STAGE_MASS = .2
#MISC = .3
# 
# PAYLOAD = LARGE_CAN + CHUTES + SMALL_STRUTS + LADDER + LIGHTS + BATT
# PAYLOAD = SMALL_CAN + CHUTE + LARGE_STRUTS + LADDER + LIGHTS + BATT + SMALL_DOCKING_PORT + SMALL_RCS
PAYLOAD = 40. # Large fuel tank
# PAYLOAD = OKTO + XENON + XENON_TANK + PANEL * 8 + BATT # Probe
# PAYLOAD = LARGE_POD + CHUTES + LIGHTS + BATT + SMALL_RCS + SMALL_DOCKING_PORT # Gemini
# PAYLOAD = CHAIR + OKTO2 + TINY_STRUTS + BATT + TINY_PANEL # Scott Manley's tiny lander
# PAYLOAD = SMALL_POD + CHUTE + SMALL_STRUTS + LADDER + LIGHTS + BATT
# PAYLOAD = SMALL_POD + CHUTE
# PAYLOAD = 0.

TANKS = []
# Tanks are listed as tuples with wet_mass, dry_mass
# Total fuel is 8xdry mass

for tank in range(1, 200, 1):
	TANKS.append((4.*tank, tank/2.))

for tank in (1, 2, 4):
	TANKS.append((tank, tank/8.))
	TANKS.append((2*tank, tank/4.))

TANKS.append((.078675-.015, .015))
TANKS.append((.136-.025, .025))
TANKS.append((0, 0))
TANKS.sort()

# Table of engines
# Impulse, mass, thrust, string
ENGINES = [
	# (370, 1.25, 215, 'LV-30'), # I really like gimballing
	# (0, 0, 0, 'Nothing'), # Only rarely useful
	(370, 1.5, 200, 'LV-45'),
	(370, 3.1, 400, 'LV-45 pair'),
	# (370, 4.8, 600, 'LV-45 triplet'), # Not entirely sure why not
	(390, .5, 50, 'LV-909'),
	(390, 1.1, 100, 'Double LV-909'),
	(390, 1.8, 150, 'Triple LV-909'),
	(390, 2.5, 220, 'Poodle'),
	# (280, 7.5, 1830, 'Mainsail + 2xSRB'), # This is an estimate, probably fairly inaccurate
	(280, 6., 1500 * .9, 'Mainsail'), # Run mainsails at 90% for heat
	(280, 12., 3000 * .9, 'Double Mainsail'), # Run mainsails at 90% for heat
	(280, 18., 4500 * .9, 'Triple Mainsail'), # Run mainsails at 90% for heat
	(280, 24., 6000 * .9, 'Quadruple Mainsail'), # Run mainsails at 90% for heat
	(280, 24., 7500 * .9, 'Quintuple Mainsail'), # Run mainsails at 90% for heat
	(280, 24., 9000 * .9, '6x Mainsail'), # Run mainsails at 90% for heat
	(280, 24., 10500 * .9, 'Septuple Mainsail'), # Run mainsails at 90% for heat
	(330, 4., 650, 'Skipper'),
	# (330, 8., 1300, 'Double Skipper'),
	# (800, 2.25, 60, 'LV-N'), # Skipped -- Too hard to land around
	(320, 1.8, 240, 'Radial pair'),
	(320, 2.4, 260, 'Radial triplet'),
	(290, .03, 1.5, 'LV-1'),
	(300, .18, 40, '24-77 pair'),
	(300, .27, 60, '24-77 triplet'),
	(300, .36, 80, '24-77 quadruplet'),
	(300, .45, 100, '24-77 quintet'),
]
ENGINES_BY_THRUST = [(e[2], e) for e in ENGINES]
ENGINES_BY_MASS = [(e[1], e) for e in ENGINES]
ENGINES.sort()
ENGINES_BY_THRUST.sort()
ENGINES_BY_MASS.sort()

MIN_ISP = ENGINES[0][0]
MAX_ISP = ENGINES[-1][0]

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
	min_weight = ENGINES_BY_MASS[0][1][1]
	adj_payload = payload_k + STAGE_MASS + min_weight;
	ideal_mass = fuel_mass(delta_v, MAX_ISP, payload_k + STAGE_MASS + min_weight)
	return adj_payload + ideal_mass

# stage = (mass, dv, dv_sum, tank, engine, next)
# old stage = (dv, mass, 
def stage_print(stage, delta_vs):
	while stage is not None:
		print stage[1], stage[0], 
		if stage[3] is None:
			print 'No tank',
		else:
			print stage[3][1],
		if stage[4] is None:
			print 'No engine'
		else:
			print stage[4][3],
			print stage[4][2]/(stage[0] * G_m)
			#print stage_twr(delta_vs, stage[2] - stage[1], stage[1])
		stage = stage[5]

def stage_count(stage):
	count = 0
	while stage is not None:
		count += 1
		stage = stage[5]
	return count

def stage_twr(delta_vs, dv_min, dv_max):
	''' Return the maximum TWR listed in delta_vs between the min and max delta-vs '''
	twr = 0.
	for idx in xrange(len(delta_vs)):
		if dv_min >= 0:
			dv_min -= delta_vs[idx][0]
			excess = delta_vs[idx][0] + dv_min
		if dv_min <= 0 and dv_max >= 0:
			twr = max(delta_vs[idx][1], twr)
			dv_max -= (delta_vs[idx][0] - excess)
			excess = 0
			dv_min = 0
		if dv_max <= 0:
			if idx < len(delta_vs):
				return max(delta_vs[idx][1], twr)
			return twr
	return twr

def fuel_stage(stage, best, dv_goal, stages, delta_vs, next_stages, tank):
	if stage[4] is None:
		return
	engine = (stage[4][0], 0, stage[4][2], 'fuel')
	return next_stage(stage, best, dv_goal, stages, delta_vs, next_stages, tank, engine)

def next_stage(stage, best, dv_goal, stages, delta_vs, next_stages, tank, engine):
	dry_mass = tank[1] + engine[1] + stage[0] + STAGE_MASS
	stage_mass = dry_mass + tank[0]
	dv_this = delta_v(dry_mass, tank[0], engine[0])
	req_twr = stage_twr(delta_vs, stage[2], dv_this)
	if (req_twr <= engine[2]/(stage_mass * G_m)):
		this_stage = (stage_mass, dv_this, stage[2] + dv_this, tank, engine, stage)
		pos = bisect.bisect(stages, stage)
		if len(stages) <= 0 or stages[pos - 1][2] < this_stage[2]:
			next_stages.append(this_stage)

def find(payload_k, delta_vs, max_stages=10):
	'''
	Finds a way to get the payload up by delta-v.
	Anything that doesn't match the requrested TWR ratios will be ignored.
	Remember: These assume kerbin weights; Use Body/Kerbin for other bodies!!
	These are in terms of stages; The first TWR should be the lifter, etc. etc.
	Optimize for fuel mass
	@param delta_vs: A sequence of delta-vs + TWRs that must be achieved
	The first stage of the mission must be the last delta-v parameter
	'''
	def next_states(stage, best, dv_goal, stages, delta_vs):
		''' Returns a list of the next possible states from this stage, These will be of a form (score (mass, dv, tank, engine, next)) '''
		if best is None:
			best = float('inf')
		#dv_sum = stage_delta_v(stage) # This is already calculated above, no need to 
					      # do it twice.
		# Skip tanks and engines that are too heavy
		next_stages = []
		# Find the largest tank we'll need by using the worst engine we can (and thrusting allll the way)
		theory_tank = bisect.bisect(TANKS, (fuel_mass(dv_goal - stage[2], MIN_ISP, payload_k + ENGINES_BY_MASS[-1][1][1] + TANKS[-1][1]), 0))
		# Find the largest tank we'll need by getting the largest tank we can use 
		meet_tank = bisect.bisect(TANKS, (best - stage[0], 0))
		
		for tank in TANKS[0:min(theory_tank, meet_tank)]:
			fuel_stage(stage, best, dv_goal, stages, delta_vs, next_stages, tank)
			for engine in ENGINES:
				next_stage(stage, best, dv_goal, stages, delta_vs, next_stages, tank, engine)
		return next_stages

	def push_stage(heap, best_list, stage, dv_goal, best_by_stage):
		''' 
		Pushes the stage onto the heap iff the stage is higher than the bar. 
		Stages must be lighter or have more dv than similar stages.
		'''
		if len(best_list) == 0:
			heapq.heappush(heap, (ideal(stage[0], dv_goal - stage[2]), stage))
			best_list.append(stage)
			return
		nstages = stage_count(stage[5])
		if ((len(best_by_stage[nstages]) > 0) and 
			(stage[5] not in best_by_stage[nstages-1])):
			return
		pos = bisect.bisect(best_list, stage)
		# Ignore stage completely if the smaller item has a higher dv
		if pos > 1 and best_list[pos - 1][2] >= stage[2]:
			return
		while pos < len(best_list) and best_list[pos][2] <= stage[2]:
			best_list.pop(pos)
		best_list.insert(pos, stage)
		heapq.heappush(heap, (ideal(stage[0], dv_goal - stage[2]), stage))
		
	dv_goal = sum((dv[0] for dv in delta_vs))
	print dv_goal
	start = (payload_k, 0., 0., None, None, None)
	print start[0]
	best = None # Best solution
	heap = []
	stages = [[] for x in xrange(max_stages + 1)]
	solved = False
	push_stage(heap, stages[0], start, dv_goal, stages)
	
	while solved == False and len(heap) > 0:
		head = heapq.heappop(heap)
		nstages = stage_count(head[1])
		# print head[1][0]
		#dv_sum = head[1][2]
		if head[1][2] > dv_goal:
			# We have a solution
			if best is not None:
				if best[1][0] > head[1][0]:
					# We have a better solution
					print "best:", best[1][0], '->', head[1][0]
					best = head
					# stage_print(best[1], delta_vs)
				elif best[0] > head[0]:
					# best is THE solution
					solved = True
			else:
				best = head
				# stage_print(best[1], delta_vs)
		elif nstages <= max_stages:
			best_mass = float('inf')
			if best is not None:
				best_mass = best[0]
			for item in next_states(head[1], best_mass, dv_goal, stages[nstages], delta_vs):
				push_stage(heap, stages[nstages], item, dv_goal, stages)
				
	if solved or best is not None:
		print stages
		return best[1][0], best[1]
	else:
		print "Not solved!"
		print stages
		return None, None

print PAYLOAD
MINMUS_PLAN = [(960, 0), (2*80+300*2 + 100, MINMUS), (960, 0)]
GEMINI_PLAN = [(240, MINMUS)]
JOOL_PLAN = [(2630, 0), (2630, 0), (950+965, 0)]
SYNC = [(1000, 0)]
# MUNAR_LANDER_PLAN = [(860., MUN), (860., MUN)] # Too little fuel to land, just enough to head back.
MUN_PLAN = [(800., 0), (2200., MUN), (800., 0)]
DUNA_PLAN = [(1673, 0), (1380*2,  DUNA), (915*2, IKE),  (1702, 0)]
DUNAR_LANDER_PLAN = [(1380*2, DUNA), (915*2, IKE)]
DUNA_AND_BACK = [(1673, 0), (1702, 0)]
LKO = [(2000, 1.3), (2500, 1.5)]
JETO = [(1000, 1.5), (3500, 1.5)]
SSTO = [(4500, 1.5)]
# PLAN = MUN_PLAN + LKO
# PLAN = MINMUS_PLAN + LKO
# PLAN = DUNAR_LANDER_PLAN
# PLAN = DUNAR_LANDER_PLAN
PLAN = LKO
#, (2000, 1.5), (2500, 1.5)]
#DUNA_PLAN = desired_stages = [(1380*2 + 1673 + 915*2, DUNA), (1702, 0), (2000, 1.5), (2500, 1.5)]
mass, stages = find(PAYLOAD, PLAN, 5)
print "Lander:", mass
stage_print(stages, PLAN)
# print PLAN
sys.exit(0)
bisect.insort(ENGINES, 
	(800, 2.25, 60, 'LV-N'), # Skipped -- Too hard to work around
)
bisect.insort(ENGINES_BY_MASS,
	(2.25, (800, 2.25, 60, 'LV-N')) # Skipped -- Too hard to work around
)
bisect.insort(ENGINES_BY_THRUST,
	(60, (800, 2.25, 60, 'LV-N')) # Skipped -- Too hard to work around
)
MIN_ISP = ENGINES[0][0]
MAX_ISP = ENGINES[-1][0]
mass, stages = find(mass, DUNA_AND_BACK + LKO, 7)
print "Lifter:"
stage_print(stages, LKO)

