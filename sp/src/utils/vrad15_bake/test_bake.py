#!/usr/bin/env python3
"""Checks vrad15_bake against scenes with known answers. Usage: test_bake.py <path to vrad15_bake>"""
import math, os, random, struct, subprocess, sys, tempfile
EXE = sys.argv[1]
TMP = tempfile.mkdtemp()
WORLD, SKY, PROP = 0, 1, 2
BUMPED, INVALID = 1, 2
DENOISE = 1

def quad(a, b, c, d, kind=WORLD):
    return [(a + b + c, kind), (a + c + d, kind)]

def box_faces(lo, hi):
    """(name, origin, u axis, v axis, inward normal, extent u, extent v) per wall."""
    x0, y0, z0 = lo; x1, y1, z1 = hi
    w, d, h = x1 - x0, y1 - y0, z1 - z0
    return {
        'floor': ((x0, y0, z0), (1, 0, 0), (0, 1, 0), (0, 0, 1), w, d),
        'top':   ((x0, y0, z1), (1, 0, 0), (0, 1, 0), (0, 0, -1), w, d),
        'west':  ((x0, y0, z0), (0, 1, 0), (0, 0, 1), (1, 0, 0), d, h),
        'east':  ((x1, y0, z0), (0, 1, 0), (0, 0, 1), (-1, 0, 0), d, h),
        'south': ((x0, y0, z0), (1, 0, 0), (0, 0, 1), (0, 1, 0), w, h),
        'north': ((x0, y1, z0), (1, 0, 0), (0, 0, 1), (0, -1, 0), w, h),
    }

def wall_tris(f, kind=WORLD):
    o, u, v, _, eu, ev = f
    p = lambda a, b: tuple(o[i] + u[i] * a + v[i] * b for i in range(3))
    return quad(p(0, 0), p(eu, 0), p(eu, ev), p(0, ev), kind)

def bump_basis(n, u, v):
    a, b = math.sqrt(2 / 3), 1 / math.sqrt(3)
    dirs = [(a, 0, b), (-1 / math.sqrt(6), 1 / math.sqrt(2), b), (-1 / math.sqrt(6), -1 / math.sqrt(2), b)]
    return [tuple(d[0] * u[i] + d[1] * v[i] + d[2] * n[i] for i in range(3)) for d in dirs]

def wall_samples(f, face, step, direct, albedo, flags=0, offset=0.1):
    o, u, v, n, eu, ev = f
    out = []
    bump = bump_basis(n, u, v) if flags & BUMPED else [(0, 0, 0)] * 3
    for t in range(int(ev // step)):
        for s in range(int(eu // step)):
            a, b = (s + .5) * step, (t + .5) * step
            pos = tuple(o[i] + u[i] * a + v[i] * b + n[i] * offset for i in range(3))
            d = direct(pos) if callable(direct) else direct
            out.append(dict(pos=pos, n=n, bump=bump, direct=d, albedo=albedo, radius=step, face=face, s=s, t=t, flags=flags))
    return out

def run(tris, samples, rays=256, bounces=8, options=0, threads=2, seed=1):
    scene, res = os.path.join(TMP, 's.bin'), os.path.join(TMP, 'r.bin')
    with open(scene, 'wb') as fp:
        fp.write(struct.pack('<IIIIIIIiI', 0x4E435342, 1, len(tris), len(samples), rays, bounces, options, threads, seed))
        for v, k in tris:
            fp.write(struct.pack('<9fI', *v, k))
        for s in samples:
            flat = list(s['pos']) + list(s['n']) + [c for b in s['bump'] for c in b] + list(s['direct']) + list(s['albedo'])
            fp.write(struct.pack('<%df' % (len(flat) + 1), *flat, s['radius']))
            fp.write(struct.pack('<IiiI', s['face'], s['s'], s['t'], s['flags']))
    subprocess.run([EXE, scene, res], check=True, stdout=subprocess.DEVNULL)
    with open(res, 'rb') as fp:
        magic, ver, n = struct.unpack('<III', fp.read(12))
        assert magic == 0x53455242 and ver == 1 and n == len(samples)
        return [struct.unpack('<12f', fp.read(48)) for _ in range(n)]

fails = 0
def check(name, got, want, tol):
    global fails
    ok = abs(got - want) <= tol
    fails += not ok
    print(f"{'PASS' if ok else 'FAIL'}  {name}: got {got:.3f}, expected {want:.3f} +/- {tol}")

def mean(xs):
    xs = list(xs); return sum(xs) / len(xs)

LO, HI = (0, 0, 0), (256, 256, 256)
W = box_faces(LO, HI)
def room(direct=(100, 100, 100), albedo=(.5, .5, .5), step=32, flags=0, kinds=None, albedos=None):
    tris, samples = [], []
    for i, (name, f) in enumerate(W.items()):
        kind = (kinds or {}).get(name, WORLD)
        tris += wall_tris(f, kind)
        if kind == WORLD:
            samples += wall_samples(f, i, step, direct, (albedos or {}).get(name, albedo), flags)
    return tris, samples

# 1. Closed box with uniform light: the indirect total is the geometric series.
tris, samples = room()
r = run(tris, samples, rays=400, bounces=6)
check('uniform box, 6 bounces', mean(x[0] for x in r), 100 * sum(.5 ** k for k in range(1, 7)), 0.8)
r1 = run(tris, samples, rays=100, bounces=1)
check('uniform box, 1 bounce', mean(x[0] for x in r1), 50.0, 0.01)

# 2. Bumped channels follow stock's dot(dir,basis)/dot(dir,normal), cosine-sampled.
tris, samples = room(flags=BUMPED)
r = run(tris, samples, rays=2000, bounces=1)
random.seed(3)
vals = []
b = bump_basis((0, 0, 1), (1, 0, 0), (0, 1, 0))[0]
for _ in range(400000):
    a, ph = random.random(), random.random() * 2 * math.pi
    rr = math.sqrt(a); d = (rr * math.cos(ph), rr * math.sin(ph), math.sqrt(1 - a))
    db = sum(d[i] * b[i] for i in range(3))
    vals.append(min(db / max(d[2], 1e-3), 8) if db > 0 else 0)
want = 50 * mean(vals)
for c in (1, 2, 3):
    check(f'bump channel {c}', mean(x[c * 3] for x in r), want, 0.35)

# 3. Open sky: floor-to-top form factor of a cube is 0.19982, and sky adds nothing.
tris, samples = room(kinds={'top': SKY})
r = run(tris, samples, rays=4000, bounces=1)
idx = [i for i, s in enumerate(samples) if s['face'] == 0]
check('sky opening removes its share', mean(r[i][0] for i in idx), 50 * (1 - 0.19982), 0.5)

# 4. Props absorb, matching stock VRAD: floor sees only prop walls.
tris = []
for name, f in W.items():
    tris += wall_tris(f, WORLD if name == 'floor' else PROP)
samples = wall_samples(W['floor'], 0, 32, (100, 100, 100), (.5, .5, .5))
r = run(tris, samples, rays=64)
check('prop-enclosed floor is unlit', mean(x[0] for x in r), 0, 1e-6)

# 5. No leak: a thin wall splits a lit room from a dark room.
A, B = box_faces((0, 0, 0), (256, 256, 256)), box_faces((256, 0, 0), (512, 256, 256))
tris, samples = [], []
for i, f in enumerate(A.values()):
    tris += wall_tris(f); samples += wall_samples(f, i, 32, (100, 100, 100), (.5, .5, .5))
for i, f in enumerate(B.values()):
    tris += wall_tris(f); samples += wall_samples(f, 10 + i, 32, (0, 0, 0), (.5, .5, .5))
r = run(tris, samples, rays=64, bounces=4)
check('dark room behind thin wall', mean(x[0] for x, s in zip(r, samples) if s['face'] >= 10), 0, 1e-6)
check('lit room next to it', mean(x[0] for x, s in zip(r, samples) if s['face'] < 10), 100 * sum(.5 ** k for k in range(1, 5)), 2.5)

# 6. Colour bleeding: a red west wall reddens the floor nearest it.
tris, samples = room(albedos={'west': (.8, .05, .05)}, step=16)
r = run(tris, samples, rays=400, bounces=3)
near = [x for x, s in zip(r, samples) if s['face'] == 0 and s['s'] == 0]
far = [x for x, s in zip(r, samples) if s['face'] == 0 and s['s'] == 15]
ratio = lambda L: mean(x[0] for x in L) / mean(x[1] for x in L)
print(f"      red/green near the wall {ratio(near):.2f}, far from it {ratio(far):.2f}")
check('bleed is stronger near the wall', float(ratio(near) > ratio(far) * 1.15), 1, 0)

# 7. Samples inside solid get filled from neighbours rather than staying black.
tris, samples = room()
for i, s in enumerate(samples):
    if s['face'] == 0 and s['s'] == 3 and s['t'] == 3:
        s['flags'] |= INVALID; bad = i
r = run(tris, samples, rays=400, bounces=2)
check('filled sample', r[bad][0], 75.0, 4)

# 8. Denoiser and seams: two coplanar faces share a noisy floor.
def seam_scene():
    tris, samples = room(kinds={'top': SKY}, step=8)
    samples = [s for s in samples if s['face'] != 0]
    tris_floor = W['floor']
    left = wall_samples((tris_floor[0], tris_floor[1], tris_floor[2], tris_floor[3], 128, 256), 20, 8, (100, 100, 100), (.5, .5, .5))
    right_o = (128, 0, 0)
    right = wall_samples((right_o, tris_floor[1], tris_floor[2], tris_floor[3], 128, 256), 21, 8, (100, 100, 100), (.5, .5, .5))
    return tris, samples + left + right
tris, samples = seam_scene()
ref = run(tris, samples, rays=1024, bounces=2, seed=5)
noisy = run(tris, samples, rays=16, bounces=2, seed=9)
try:
    clean = run(tris, samples, rays=16, bounces=2, seed=9, options=DENOISE)
except subprocess.CalledProcessError:
    clean = None
rmse = lambda a: math.sqrt(mean((x[0] - y[0]) ** 2 for x, y in zip(a, ref)))
print(f"      RMSE against reference: raw {rmse(noisy):.2f}" + (f", denoised {rmse(clean):.2f}" if clean else ", denoiser not built"))
if clean:
    check('denoiser reduces error by at least 2x', float(rmse(clean) * 2 < rmse(noisy)), 1, 0)
    mean_of = lambda a: mean(x[0] for x in a)
    check('denoiser keeps total energy', mean_of(clean), mean_of(ref), mean_of(ref) * 0.02)
    def step(a):
        by = {}
        for x, s in zip(a, samples):
            if s['face'] in (20, 21):
                by.setdefault((s['face'], s['s'], s['t']), x[0])
        return mean(abs(by[(20, 15, t)] - by[(21, 0, t)]) for t in range(32))
    print(f"      step across the seam: reference {step(ref):.2f}, denoised {step(clean):.2f}")
    check('no visible seam after denoising', float(step(clean) <= max(step(ref) * 2, 1.0)), 1, 0)

print('ALL PASSED' if not fails else f'{fails} FAILED')
sys.exit(1 if fails else 0)
