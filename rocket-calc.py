#!/usr/bin/python
import math
import heapq
import bisect

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
# PAYLOAD = 40. # Large fuel tank
PAYLOAD = OKTO + XENON + XENON_TANK + PANEL * 8 + BATT # Probe
# PAYLOAD = LARGE_POD + CHUTES + LIGHTS + BATT + SMALL_RCS + SMALL_DOCKING_PORT # Gemini
# PAYLOAD = CHAIR + OKTO2 + TINY_STRUTS + BATT + TINY_PANEL # Scott Manley's tiny lander
# PAYLOAD = SMALL_POD + CHUTE + SMALL_STRUTS + LADDER + LIGHTS + BATT
# PAYLOAD = SMALL_POD + CHUTE
# PAYLOAD = 0.

TANKS = []
# Tanks are listed as tuples with wet_mass, dry_mass
# Total fuel is 8xdry mass

for tank in range(1, 100, 1):
	TANKS.append((4.*tank, tank/2.))

for tank in (1, 2, 4):
	TANKS.append((tank, tank/8.))
	TANKS.append((2*tank, tank/4.))

TANKS.append((.078675-.015, .015))
TANKS.append((.136-.025, .025))
TANKS.append((0, 0))
TANKS_BY_MASS = [(sum(t), t) for t in TANKS]
TANKS_BY_MASS.sort()
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
	(280, 6., 1500 * .9, 'Mainsail'),
	(280, 12., 3000 * .9, 'Double Mainsail'),
	(280, 18., 4500 * .9, 'Triple Mainsail'),
	(280, 24., 6000 * .9, 'Quadruple Mainsail'),
	(280, 24., 7500 * .9, 'Quintuple Mainsail'),
	(280, 24., 9000 * .9, '6x Mainsail'),
	(280, 24., 10500 * .9, 'Septuple Mainsail'),
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
	min_weight = min((engine[1] for engine in ENGINES))
	adj_payload = payload_k + STAGE_MASS + min_weight;
	ideal_mass = fuel_mass(delta_v, MAX_ISP, payload_k + STAGE_MASS + min_weight)
	return adj_payload + ideal_mass

# stage = (mass, dv, tank, engine, next)
# old stage = (dv, mass, 
def stage_print(stage):
	while stage is not None:
		print stage[1], stage[0], 
		if stage[2] is None:
			print 'No tank',
		else:
			print stage[2][1],
		if stage[3] is None:
			print 'No engine'
		else:
			print stage[3][3]
		stage = stage[4]

def stage_delta_v(stage):
	dv = 0.
	while stage is not None:
		dv += stage[1]
		stage = stage[4]
	return dv

def stage_twr(delta_vs, dv_min, dv_max):
	''' Return the maximum TWR listed in delta_vs between the min and max delta-vs '''
	index = 0
	rem = 0
	twr = 0.
	for idx in xrange(len(delta_vs)):
		index = idx
		dv_min -= delta_vs[idx][0]
		if dv_min < delta_vs[idx][0]:
			rem = delta_vs[idx][0] - dv_min
			break
		else:
			dv_min -= delta_vs[idx][0]
	if (index < len(delta_vs)):
		twr = delta_vs[index][1]
		dv_max -= delta_vs[index][0]
	for idx in xrange(len(delta_vs) - index - 1):
		twr = max(delta_vs[idx][1], twr)
		dv_max -= delta_vs[idx][0]
		if dv_max < 0:
			return twr
	return twr

def find(payload_k, delta_vs):
	'''
	Finds a way to get the payload up by delta-v.
	Anything that doesn't match the requrested TWR ratios will be ignored.
	Remember: These assume kerbin weights; Use Body/Kerbin for other bodies!!
	These are in terms of stages; The first TWR should be the lifter, etc. etc.
	Optimize for fuel mass
	@param delta_vs: A sequence of delta-vs + TWRs that must be achieved
	The first stage of the mission must be the last delta-v parameter
	'''
	def next_states(stage, best, dv_sum):
		''' Returns a list of the next possible states from this stage, These will be of a form (score (mass, dv, tank, engine, next)) '''
		if best is None:
			best = float('inf')
		dv_sum = stage_delta_v(stage) # This is already calculated above, no need to 
					      # do it twice.
		# Skip tanks and engines that are too heavy
		next_stages = []
		#print bisect.bisect(TANKS_BY_MASS, (best - stage[0]))
		for tank in TANKS[0:bisect.bisect(TANKS_BY_MASS, (best - stage[0], 0))]:
			for engine in ENGINES[0:bisect.bisect(ENGINES_BY_MASS, (best - (stage[0] + sum(tank)), 0))]:
				dry_mass = tank[1] + engine[1] + stage[0] + STAGE_MASS
				dv_this = delta_v(dry_mass, tank[0], engine[0])
				req_twr = stage_twr(delta_vs, dv_sum, dv_this)
				if (req_twr <= engine[2]/(dry_mass + tank[0])):
					next_stages.append((dry_mass + tank[0], dv_this, tank, engine, stage))
		#print next_stages
		return next_stages
	dv_goal = sum((dv[0] for dv in delta_vs))
	print dv_goal
	start = (ideal(payload_k, dv_goal), (payload_k, 0., None, None, None))
	print start[0]
	best = None # Best solution
	heap = []
	solved = False
	heapq.heappush(heap, (start))
	
	while solved == False and len(heap) > 0:
		head = heapq.heappop(heap)
		#print head[0]
		dv_sum = stage_delta_v(head[1])
		if dv_sum > dv_goal:
			# We have a solution
			if best is not None:
				if best[1][0] > head[1][0]:
					# We have a better solution
					print "best:", best[1][0], '->', head[1][0]
					stage_print(best[1])
					best = head
				elif best[0] > head[0]:
					# best is THE solution
					solved = True
			else:
				best = head
		else:
			best_mass = float('inf')
			if best is not None:
				best_mass = best[0]
			for item in next_states(head[1], best_mass, dv_sum):
				heapq.heappush(heap, (ideal(item[0], dv_goal - item[1]), item))
	if solved or best is not None:
		return best[1][0], best[1]
	else:
		print "Not solved!"
		return None, None

print PAYLOAD
MINMUS_PLAN = [(2*80+300*2+960*2 + 100, MINMUS)]
GEMINI_PLAN = [(240, MINMUS)]
JOOL_PLAN = [(2630, 0), (2630, 0), (950+965, 0)]
SYNC = [(1000, 0)]
MUN_PLAN = [(800*2., 0)]
# MUNAR_LANDER_PLAN = [(860., MUN), (860., MUN)] # Too little fuel to land, just enough to head back.
MUNAR_LANDER_PLAN = [(1000., MUN), (1200., MUN)]
DUNA_PLAN = [(1673, 0), (1380*2,  DUNA), (915*2, IKE),  (1702, 0)]
DUNAR_LANDER_PLAN = [(1380*2, DUNA), (915*2, IKE)]
DUNA_AND_BACK = [(1673, 0), (1702, 0)]
LKO = [(2000, 1.5), (2500, 1.5)]
JETO = [(1000, 1.5), (3500, 1.5)]
SSTO = [(4500, 1.5)]
#, (2000, 1.5), (2500, 1.5)]
#DUNA_PLAN = desired_stages = [(1380*2 + 1673 + 915*2, DUNA), (1702, 0), (2000, 1.5), (2500, 1.5)]
mass, stages = find(PAYLOAD, LKO)
print "Lander:", mass
stage_print(stages)
#ENGINES.append( 
	#(800, 2.25, 60, 'LV-N'), # Skipped -- Too hard to work around
#)
#mass, stages = find(mass, MUN_PLAN + LKO)
#print "Lifter:"
# mass, stages = find(6.76675, DUNA_AND_BACK + LKO)
#print_stages(stages)

