#!/usr/bin/env python3
"""sec 63 #7g-3 fix phase -- build-level red / green / mutant evidence.

For each target: copy the 7g-2 cold gate's staged work dir (its .vc VERBATIM and
its staged sources, tree e5a5c574 == dc27a12's), substitute a waiver and zero or
more source files (by basename), and run `verilator -f <vc>` exactly as the gate's
build step does.  NO -Wno-fatal: every core is warnings-fatal (Verilator's
default), and whether the build dies is the thing being measured.
Nothing is committed red: a red build is MEASURED here, never carried by a
commit (sec 22.75, every commit bisects).

usage: elab_check.py <out_dir> <waiver.vlt> <targets,comma> [basename=path ...]
"""
import os, sys, glob, shutil, subprocess, re, collections
GB = '/home/kourosh/gate_7g2_cold/build'
out, waiver, targets = sys.argv[1], sys.argv[2], sys.argv[3].split(',')
over = dict(a.split('=', 1) for a in sys.argv[4:])
env = dict(os.environ, PATH='/homes/kourosh/miniconda3/envs/pcie/bin:' + os.environ['PATH'])
for t in targets:
    src = glob.glob(f'{GB}/*/{t}')
    assert len(src) == 1, (t, src)
    d = os.path.join(out, t)
    shutil.rmtree(d, ignore_errors=True)
    os.makedirs(d)
    shutil.copytree(os.path.join(src[0], 'src'), os.path.join(d, 'src'))
    vc = glob.glob(os.path.join(src[0], '*.vc'))[0]
    shutil.copy(vc, d)
    shutil.copy(waiver, os.path.join(d, 'waiver.vlt'))
    n_over = collections.Counter()
    for base, path in over.items():
        for f in glob.glob(os.path.join(d, 'src', '**', base), recursive=True):
            shutil.copy(path, f); n_over[base] += 1
    r = subprocess.run(['verilator', '-f', os.path.basename(vc)], cwd=d, env=env,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=1800)
    log = r.stdout.decode(errors='replace')
    open(os.path.join(d, 'elab.log'), 'w').write(log)
    warns = [re.sub(r'^%Warning-([A-Z]+): \S*/([^/ ]+:\d+):\d+:.*$', r'\1 \2', l)
             for l in log.splitlines() if l.startswith('%Warning-')]
    errs = [l for l in log.splitlines() if l.startswith('%Error') and 'Exiting due to' not in l]
    shutil.rmtree(os.path.join(d, 'src'))
    for f in glob.glob(os.path.join(d, 'V*')):
        os.remove(f)
    print(f"{t}\trc={r.returncode}\twarnings={len(warns)}\terrors={len(errs)}\t"
          f"overlaid={dict(n_over)}\t{sorted(collections.Counter(warns).items())}", flush=True)
