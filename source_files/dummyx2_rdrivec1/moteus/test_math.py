import math

def wrap_balanced_cpr(value, cpr):
    val = value % cpr
    if val >= cpr / 2.0:
        val -= cpr
    elif val < -cpr / 2.0:
        val += cpr
    return val

def simulate(rotor_pos, output_pos, ratio):
    output_ambiguity_scale = ratio
    
    # rotor_pos in full revolutions. ratio in revolutions.
    output_value = wrap_balanced_cpr(rotor_pos * ratio, ratio)
    
    reference_value = wrap_balanced_cpr(output_pos, 1.0)
    
    integral_offsets_lower = math.floor(reference_value / ratio)
    integral_offsets_upper = integral_offsets_lower + 1
    
    maybe_lower_value = integral_offsets_lower * ratio + output_value
    maybe_upper_value = integral_offsets_upper * ratio + output_value
    
    if abs(maybe_lower_value - reference_value) < abs(maybe_upper_value - reference_value):
        first_disambiguation = maybe_lower_value
    else:
        first_disambiguation = maybe_upper_value
        
    return first_disambiguation * 360.0

ratio = 1.0 / 30.0

print("Testing hysteresis effect")
# Suppose true position is 0
# Backlash introduces a difference between output_pos and rotor_pos * ratio
def test(test_pos_deg, backlash_deg):
    test_pos = test_pos_deg / 360.0
    backlash = backlash_deg / 360.0
    rotor_p = (test_pos + backlash) / ratio
    out_p = test_pos
    res = simulate(rotor_p, out_p, ratio)
    print("True: {:.2f}, Backlash: {:.2f}, Resolved: {:.2f}, Diff: {:.2f}".format(
        test_pos_deg, backlash_deg, res, res - test_pos_deg))

for b in [-8, -6, -4, 0, 4, 6, 8]:
    test(0, b)

