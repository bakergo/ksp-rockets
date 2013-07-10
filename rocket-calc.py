#!/usr/bin/python
import math

# Gravity in m/s
G_m = 9.80665
MUN = 1.624 / G_m * 1.5
MINMUS = .491 / G_m * 1.5
DUNA = 2.94 / G_m * 1.3
IKE = 1.10 / G_m * 1.3

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

def print_stages(stages):
	for stage in stages:
		print stage[0], stage[1], stage[2][1], stage[3][3]

def trim_stages(desired_stages, stages):
	desired_stages = [x for x in desired_stages]
	new_stages = []
	carry = 0.
	while len(stages) > 0:
		stage = stages.pop()
		desired_stage = desired_stages.pop()
		new_dv = desired_stage[0] + carry
		carry = desired_stage[0] - stage[0]
		print desired_stage[0], stage[0], new_dv, carry
		if new_dv < 0:
			carry = new_dv
		else:
			new_stages.insert(0, (new_dv, desired_stage[1]))
	return new_stages

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
PAYLOAD = LARGE_CAN + CHUTES + SMALL_STRUTS + LADDER + LIGHTS + BATT
# PAYLOAD = SMALL_CAN + CHUTE + LARGE_STRUTS + LADDER + LIGHTS + BATT + SMALL_DOCKING_PORT + SMALL_RCS
# PAYLOAD = 40. # Large fuel tank
# PROBE = OKTO + XENON + XENON_TANK + PANEL * 8 + BATT # Probe
# PAYLOAD = LARGE_POD + CHUTES + LIGHTS + BATT + SMALL_RCS + SMALL_DOCKING_PORT # Gemini
# PAYLOAD = CHAIR + OKTO2 + TINY_STRUTS + BATT + TINY_PANEL # Scott Manley's tiny lander
# PAYLOAD = SMALL_POD + CHUTE + SMALL_STRUTS + LADDER + LIGHTS + BATT
# PAYLOAD = SMALL_POD + CHUTE
# PAYLOAD = 0.

tanks = []
# Tanks are listed as tuples with wet_mass, dry_mass
# Total fuel is 8xdry mass

for tank in range(1, 100, 1):
	tanks.append((4.*tank, tank/2.))

for tank in (1, 2, 4):
	tanks.append((tank, tank/8.))
	tanks.append((2*tank, tank/4.))

tanks.append((.078675-.015, .015))
tanks.append((.136-.025, .025))
tanks.append((0, 0))

# Table of engines
# Impulse, mass, thrust, string
ENGINES = [
	#(370, 1.25, 215, 'LV-30'),
	(0, 0, 0, 'Nothing'),
	(370, 1.5, 200, 'LV-45'),
	(370, 3.1, 400, 'LV-45 pair'),
	# (370, 4.8, 600, 'LV-45 triplet'),
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
	# (800, 2.25, 60, 'LV-N'), # Skipped -- Too hard to work around
	(320, 1.8, 240, 'Radial pair'),
	(320, 2.4, 260, 'Radial triplet'),
	(290, .03, 1.5, 'LV-1'),
	(300, .18, 40, '24-77 pair'),
	(300, .27, 60, '24-77 triplet'),
	(300, .36, 80, '24-77 quadruplet'),
	(300, .45, 100, '24-77 quintet'),
]

def find(payload, delta_vs):
	'''
	Finds a way to get the payload up by delta-v
	Anything that doesn't match the requrested TWR ratios will be ignored.
	Remember: These assume kerbin weights; Use Body/Kerbin for other bodies!!
	These are in terms of stages; The first TWR should be the lifter, etc. etc.
	Optimize for fuel mass
	@param delta_vs: A sequence of delta-vs + TWRs that must be achieved
	The first stage of the mission must be the last delta-v parameter
	'''
	if (len(delta_vs) <= 0):
		return
	best = None
	req_dv, twr = delta_vs[-1]
	if len(delta_vs) <= 1:
		next_mass = payload
		next_print = []
		engines = ENGINES
	else:
		next_mass, next_print = find(payload, delta_vs[0:-1])
		if next_print is None:
			return 0., None
		last_engine = next_print[-1][3]
		fuel_only = (last_engine[0], 0, last_engine[2], 'fuel')
		engines = ENGINES + [fuel_only]

	for tank in tanks:
		for engine in engines:
			dry_mass = next_mass + tank[1] + engine[1] + STAGE_MASS
			delt = delta_v(dry_mass, tank[0], engine[0])
			min_thrust = (dry_mass + tank[0]) * G_m * twr
			if delt < req_dv:
				continue
			if engine[2] < min_thrust:
				continue
			if best is not None:
				if best[1] > dry_mass + tank[0]:
					best = (delt, dry_mass + tank[0], tank, engine)
			else:
				best = (delt, dry_mass + tank[0], tank, engine)

	# Use this solution as the bound for the previous one.
	if best is not None:
		improved = True
		while improved:
			excess = best[0] - req_dv
			if len(delta_vs) <= 1:
				# No next stage to calculate
				break
			req_dv_old, req_dv_twr = delta_vs[-2]
			#print req_dv_old, req_dv_twr
			if req_dv_twr > 0 and twr < req_dv_twr and (engine[2] / best[1]) < req_dv_twr:
				# Don't fuck with required TWR
				break
			#print "Improving"
			req_dv_new = req_dv_old - excess
			req_dv_new = 0 if req_dv_new < 0 else req_dv_new
			new_delts = delta_vs[0:-2]
			new_delts.append((req_dv_new, req_dv_twr))
			child_mass, child_print = find(payload, new_delts)
			if child_mass >= next_print[-1][1]:
				break
			new_delts.append((best[0], twr))
			newmass, new_print = find(payload, new_delts)
			if newmass < best[1]:
				#print "Improved from ", best[1], 'to', newmass
				best = new_print[-1]
				next_print = new_print[:-1]
				improved = True
			else:
				improved = False
	if best is not None:
		next_print.append(best)
		return best[1], next_print
	else:
		return None, None

print PAYLOAD
MINMUS_PLAN = [(2*80+300*2+960*2 + 100, MINMUS) ]
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
mass, stages = find(PAYLOAD, MINMUS_PLAN + LKO)
print "Lander:", mass
print_stages(stages)
#ENGINES.append( 
	#(800, 2.25, 60, 'LV-N'), # Skipped -- Too hard to work around
#)
#mass, stages = find(mass, MUN_PLAN + LKO)
#print "Lifter:"
# mass, stages = find(6.76675, DUNA_AND_BACK + LKO)
#print_stages(stages)

