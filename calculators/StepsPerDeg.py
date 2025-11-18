def spd(spr, gr):
    return (spr * gr) / 360

steps_per_rev = 1600 # Satt som felles verdi på alle driverne våre.
gear = [10, 50, 20, 20, 20, 20]

for i, gr in enumerate(gear, 1):
    print(f"J{i}: {spd(steps_per_rev, gr):.6f}")
