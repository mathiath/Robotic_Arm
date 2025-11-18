def enc_mult(ppr, quad_factor, gear_ratio):
    return (ppr * quad_factor * gear_ratio) / 360.0


quad_factor = 4  # Teensy Encoder.h teller alle flanker

joints = {
    "Joint 1": {"ppr": 1000, "gear": 10},
    "Joint 2": {"ppr": 1000, "gear": 50},
    "Joint 3": {"ppr": 1000, "gear": 20},
    "Joint 4": {"ppr": 300,  "gear": 20},
    "Joint 5": {"ppr": 300,  "gear": 20},
    "Joint 6": {"ppr": 1000, "gear": 20},
}

for name, cfg in joints.items():
    value = enc_mult(cfg["ppr"], quad_factor, cfg["gear"])
    print(f"{name}: encMult = {value:.4f} counts/degree")
