"""Experiment suite E1-E6, statistics and figures.

  E1 main comparison      scenarios x policies x speeds x link rates
  E2 bandwidth frontier   route policies over look-ahead K vs Cognitive-NDN (coverage vs kB/km)
  E3 VLA robustness       injected misses, false requests and wrong zones
  E4 VLA latency          fast / llava-phi3 / slow reasoning profiles (model-swap sensitivity)
  E5 multi-vehicle        1-20 vehicles sharing one road-side unit
  E6 fairness control     straight road: Cognitive-NDN must not beat route prefetching

Usage:  python3 -m cogndn_eval.experiments --out results [--seeds 30] [--only E1,E5] [--replay recs.json]
"""
import argparse
import csv
import itertools
import json
import math
import os
import statistics
from concurrent.futures import ProcessPoolExecutor

from cogndn_core.scenario import Scenario
from cogndn_core.world import World

from .harness import POLICIES, RunParams, VLAParams, run_once
from .netsim import NetParams

HERE = os.path.dirname(os.path.abspath(__file__))
COLORS = {  # fixed categorical order (validated palette); colour follows the policy, never its rank
    'cognitive': '#2a78d6', 'route_typed': '#eb6834', 'route_all': '#1baf7a', 'rules': '#eda100',
    'cognitive_hybrid': '#e87ba4', 'oracle': '#008300', 'ndn_ondemand': '#6250d6', 'ip_ondemand': '#e34948'}
MARKERS = {'cognitive': 'o', 'route_typed': 's', 'route_all': '^', 'rules': 'D', 'cognitive_hybrid': 'v',
           'oracle': '*', 'ndn_ondemand': 'P', 'ip_ondemand': 'X'}
LABEL = {'cognitive': 'Cognitive-NDN', 'cognitive_hybrid': 'Cognitive-NDN (hybrid)', 'route_typed': 'Route-typed',
         'route_all': 'Route-all', 'rules': 'Detector rules', 'oracle': 'Oracle', 'ndn_ondemand': 'NDN on-demand',
         'ip_ondemand': 'IP on-demand'}
METRICS = ('cov_semantic', 'cov_all', 'delay_mean_ms', 'delay_p95_ms', 'delay_sem_mean_ms', 'kb_per_km',
           'wasted_kb_per_km', 'precision', 'interests', 'origin_kb_per_vehicle_km', 'rsu_hit_ratio')


def default_paths():
    cfg = os.environ.get('COGNDN_SIM_CONFIG') or os.path.join(HERE, '..', '..', 'cogndn_sim', 'config')
    if not os.path.isdir(cfg):
        from ament_index_python.packages import get_package_share_directory
        cfg = os.path.join(get_package_share_directory('cogndn_sim'), 'config')
    return os.path.join(cfg, 'town.yaml'), os.path.join(cfg, 'scenarios')


_W = None
_SC = {}


def _scenario(sid):
    global _W
    town, sdir = default_paths()
    if _W is None:
        _W = World.from_yaml(town)
    if sid not in _SC:
        _SC[sid] = Scenario.from_yaml(os.path.join(sdir, sid + '.yaml'), _W)
    return _SC[sid]


def _job(spec):
    sc = _scenario(spec['scenario'])
    run = RunParams(speed=spec['speed'], k=spec.get('k', 3))
    net = NetParams(rate_kbps=spec['rate_kbps'], **spec.get('net', {}))
    vla = VLAParams(**spec.get('vla', {}))
    m = run_once(sc, spec['policy'], run, net, vla, seed=spec['seed'], n_vehicles=spec.get('n', 1),
                 headway_s=spec.get('headway', 5.0), records=spec.get('records'))
    row = {k: v for k, v in spec.items() if k not in ('vla', 'records', 'net')}
    row.update({('vla_' + k): v for k, v in spec.get('vla', {}).items()})
    row.update({k: m.get(k) for k in METRICS})
    return row


SCEN = ['s0_straight_control', 's1_livestock', 's2_workzone', 's3_closure_detour', 's4_school_crossing',
        's5_decoy_sign']


def plan(name, seeds):
    S = range(seeds)
    if name == 'E1':
        for sc, pol, v, r, s in itertools.product(SCEN, POLICIES, (3.0, 8.0, 14.0), (2000, 5000, 20000), S):
            yield {'exp': 'E1', 'scenario': sc, 'policy': pol, 'speed': v, 'rate_kbps': r, 'seed': s}
    elif name == 'E2':
        for sc, s in itertools.product(SCEN[1:5], S):
            for pol, k in itertools.chain(itertools.product(('route_typed', 'route_all'), (1, 2, 3, 5, 8)),
                                          itertools.product(('cognitive',), (1, 2, 3))):
                yield {'exp': 'E2', 'scenario': sc, 'policy': pol, 'k': k, 'speed': 8.0, 'rate_kbps': 5000,
                       'seed': s}
    elif name == 'E3':
        grid = [{'p_miss': x} for x in (0.0, 0.1, 0.3, 0.5, 0.8)] + \
               [{'p_false': x} for x in (0.2, 0.5, 0.9)] + [{'p_wrong_zone': x} for x in (0.2, 0.5)]
        for sc, pol, g, s in itertools.product(SCEN[1:5], ('cognitive', 'cognitive_hybrid'), grid, S):
            yield {'exp': 'E3', 'scenario': sc, 'policy': pol, 'speed': 8.0, 'rate_kbps': 5000, 'seed': s, 'vla': g}
        for sc, pol, s in itertools.product(SCEN[1:5], ('route_typed', 'rules'), S):
            yield {'exp': 'E3', 'scenario': sc, 'policy': pol, 'speed': 8.0, 'rate_kbps': 5000, 'seed': s}
    elif name == 'E4':
        prof = {'fast_1-2s': (1.0, 2.0), 'llava-phi3_3.4-5.9s': (3.4, 5.9), 'slow_8-12s': (8.0, 12.0)}
        for sc, (pn, (lo, hi)), v, s in itertools.product(SCEN[1:5], prof.items(), (3.0, 8.0, 14.0), S):
            yield {'exp': 'E4', 'scenario': sc, 'policy': 'cognitive', 'profile': pn, 'speed': v,
                   'rate_kbps': 5000, 'seed': s, 'vla': {'lat_lo': lo, 'lat_hi': hi}}
    elif name == 'E5':
        for sc, pol, n, s in itertools.product(('s1_livestock', 's3_closure_detour'),
                                               ('ndn_ondemand', 'route_typed', 'route_all', 'cognitive'),
                                               (1, 5, 10, 20), range(min(seeds, 10))):
            yield {'exp': 'E5', 'scenario': sc, 'policy': pol, 'n': n, 'headway': 5.0, 'speed': 8.0,
                   'rate_kbps': 5000, 'seed': s}
    elif name == 'E6':
        for pol, v, r, s in itertools.product(('cognitive', 'route_typed', 'rules', 'oracle'), (3.0, 8.0, 14.0),
                                              (2000, 5000, 20000), S):
            yield {'exp': 'E6', 'scenario': 's0_straight_control', 'policy': pol, 'speed': v, 'rate_kbps': r,
                   'seed': s}


def write_csv(rows, path):
    keys = sorted({k for r in rows for k in r})
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, keys)
        w.writeheader()
        w.writerows(rows)


# ---------------------------------------------------------------------------- statistics
def ci95(xs):
    xs = [x for x in xs if x is not None and not (isinstance(x, float) and math.isnan(x))]
    if not xs:
        return float('nan'), float('nan'), 0
    m = statistics.fmean(xs)
    if len(xs) < 2:
        return m, 0.0, len(xs)
    from scipy import stats
    return m, stats.t.ppf(0.975, len(xs) - 1) * statistics.stdev(xs) / math.sqrt(len(xs)), len(xs)


def paired(rows, a, b, metric, keys):
    """Wilcoxon signed-rank on per-seed paired values of policies a vs b, pooled over `keys` conditions."""
    from scipy import stats
    idx = {}
    for r in rows:
        idx[(r['policy'],) + tuple(r[k] for k in keys) + (r['seed'],)] = r[metric]
    da = []
    for k, v in idx.items():
        if k[0] == a and (b,) + k[1:] in idx:
            x, y = v, idx[(b,) + k[1:]]
            if x is not None and y is not None and not (math.isnan(x) or math.isnan(y)):
                da.append(x - y)
    if not da or all(d == 0 for d in da):
        return {'n': len(da), 'mean_diff': 0.0 if da else float('nan'), 'p': 1.0, 'dz': 0.0}
    p = stats.wilcoxon(da, zero_method='zsplit').pvalue
    sd = statistics.stdev(da) if len(da) > 1 else 0.0
    return {'n': len(da), 'mean_diff': statistics.fmean(da), 'p': p, 'dz': statistics.fmean(da) / sd if sd else float('inf')}


def summarize(rows, group, metrics=METRICS):
    out = {}
    for r in rows:
        out.setdefault(tuple(r[g] for g in group), []).append(r)
    table = []
    for k, rs in sorted(out.items(), key=lambda kv: tuple(str(x) for x in kv[0])):
        row = dict(zip(group, k))
        for m in metrics:
            mean, h, n = ci95([r.get(m) for r in rs])
            row[m], row[m + '_ci'] = mean, h
        row['runs'] = len(rs)
        table.append(row)
    return table


# ---------------------------------------------------------------------------- figures
def _plt():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family': 'serif', 'font.size': 9, 'axes.spines.top': False,
                         'axes.spines.right': False, 'axes.grid': True, 'grid.color': '#e6e6e6',
                         'grid.linewidth': 0.6, 'axes.edgecolor': '#888888', 'legend.frameon': False})
    return plt


def fig_e1(rows, out):
    plt = _plt()
    pols = ['cognitive', 'route_typed', 'route_all', 'rules', 'ndn_ondemand']
    scs = SCEN[1:5]
    sub = [r for r in rows if r['speed'] == 8.0 and r['rate_kbps'] == 5000]
    fig, axes = plt.subplots(1, 2, figsize=(7.2, 3.0))
    w = 0.16
    for j, pol in enumerate(pols):
        for ax, metric, scale in ((axes[0], 'cov_semantic', 100.0), (axes[1], 'kb_per_km', 1 / 1000.0)):
            ms, hs = [], []
            for sc in scs:
                m, h, _ = ci95([r[metric] for r in sub if r['scenario'] == sc and r['policy'] == pol])
                ms.append(m * scale)
                hs.append(h * scale)
            xs = [i + (j - 2) * w for i in range(len(scs))]
            ax.bar(xs, ms, w * 0.9, yerr=hs, color=COLORS[pol], label=LABEL[pol], error_kw={'lw': 0.8},
                   edgecolor='white', linewidth=0.5)
    axes[0].set_ylabel('Required semantic data on time (%)')
    axes[1].set_ylabel('Data fetched (MB per km)')
    for ax in axes:
        ax.set_xticks(range(len(scs)))
        ax.set_xticklabels(['Live-\nstock', 'Work\nzone', 'Detour', 'School'], fontsize=8)
    axes[0].set_ylim(0, 105)
    fig.legend(*axes[0].get_legend_handles_labels(), loc='upper center', bbox_to_anchor=(0.5, 1.06), ncol=5,
               fontsize=7.5)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'fig_e1_main.png'), dpi=300, bbox_inches='tight')
    plt.close(fig)


def fig_e2(rows, out):
    plt = _plt()
    fig, ax = plt.subplots(figsize=(3.5, 2.8))
    for pol in ('route_typed', 'route_all', 'cognitive'):
        ks = sorted({r['k'] for r in rows if r['policy'] == pol})
        xs, ys = [], []
        for k in ks:
            rs = [r for r in rows if r['policy'] == pol and r['k'] == k]
            xs.append(statistics.fmean(r['kb_per_km'] for r in rs) / 1000.0)
            ys.append(100 * statistics.fmean(r['cov_semantic'] for r in rs))
        ax.plot(xs, ys, marker=MARKERS[pol], color=COLORS[pol], lw=2, ms=6, label=LABEL[pol])
        ax.annotate('K=%d' % ks[-1], (xs[-1], ys[-1]), textcoords='offset points', xytext=(4, -10), fontsize=7,
                    color='#444444')
    ax.set_xlabel('Data fetched (MB per km)')
    ax.set_ylabel('Semantic data on time (%)')
    ax.legend(fontsize=7.5, loc='lower right')
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'fig_e2_frontier.png'), dpi=300, bbox_inches='tight')
    plt.close(fig)


def fig_e3(rows, out):
    plt = _plt()
    fig, ax = plt.subplots(figsize=(3.5, 2.8))
    base = {p: 100 * statistics.fmean(r['cov_semantic'] for r in rows if r['policy'] == p)
            for p in ('route_typed', 'rules')}
    for pol in ('cognitive', 'cognitive_hybrid'):
        xs = sorted({r.get('vla_p_miss') for r in rows if r['policy'] == pol and r.get('vla_p_miss') is not None})
        ys = [100 * statistics.fmean(r['cov_semantic'] for r in rows
                                     if r['policy'] == pol and r.get('vla_p_miss') == x) for x in xs]
        ax.plot(xs, ys, marker=MARKERS[pol], color=COLORS[pol], lw=2, ms=6, label=LABEL[pol])
    for p, v in base.items():
        ax.axhline(v, color=COLORS[p], lw=1.2, ls='--', label=LABEL[p])
    ax.set_xlabel('VLA miss rate (share of needs never requested)')
    ax.set_ylabel('Semantic data on time (%)')
    ax.legend(fontsize=7.5, loc='lower left')
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'fig_e3_robustness.png'), dpi=300, bbox_inches='tight')
    plt.close(fig)


def fig_e5(rows, out):
    plt = _plt()
    fig, ax = plt.subplots(figsize=(3.5, 2.8))
    for pol in ('ndn_ondemand', 'route_typed', 'route_all', 'cognitive'):
        ns = sorted({r['n'] for r in rows})
        ys = [statistics.fmean(r['origin_kb_per_vehicle_km'] for r in rows if r['policy'] == pol and r['n'] == n)
              / 1000.0 for n in ns]
        ax.plot(ns, ys, marker=MARKERS[pol], color=COLORS[pol], lw=2, ms=6, label=LABEL[pol])
    ax.set_xlabel('Vehicles sharing the road-side unit')
    ax.set_ylabel('Origin traffic (MB per vehicle-km)')
    ax.legend(fontsize=7.5)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'fig_e5_fleet.png'), dpi=300, bbox_inches='tight')
    plt.close(fig)


# ---------------------------------------------------------------------------- main
def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='results')
    ap.add_argument('--seeds', type=int, default=30)
    ap.add_argument('--only', default='E1,E2,E3,E4,E5,E6')
    ap.add_argument('--workers', type=int, default=os.cpu_count())
    ap.add_argument('--replay', default=None, help='JSON list of recorded VLA states (see make_replay)')
    ap.add_argument('--calibration', default=None,
                    help='calibration.json from `fleet --mode calibrate`: measured one-way delay and origin latency')
    a = ap.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)
    report = {}
    for name in a.only.split(','):
        specs = list(plan(name, a.seeds))
        if a.calibration:
            cal = json.load(open(a.calibration)).get('netparams', {})
            for s_ in specs:
                s_['net'] = {k: float(cal[k]) for k in ('one_way_s', 'origin_median_s') if k in cal}
        if a.replay and name in ('E1', 'E6'):
            recs = json.load(open(a.replay))
            for s in specs:
                if s['policy'] in ('cognitive', 'cognitive_hybrid') and s['scenario'] in recs:
                    s['records'] = recs[s['scenario']]
        with ProcessPoolExecutor(a.workers) as ex:
            rows = list(ex.map(_job, specs, chunksize=32))
        write_csv(rows, os.path.join(a.out, '%s_runs.csv' % name))
        grp = {'E1': ['scenario', 'policy', 'speed', 'rate_kbps'], 'E2': ['scenario', 'policy', 'k'],
               'E3': ['scenario', 'policy', 'vla_p_miss', 'vla_p_false', 'vla_p_wrong_zone'],
               'E4': ['scenario', 'profile', 'speed'], 'E5': ['scenario', 'policy', 'n'],
               'E6': ['policy', 'speed', 'rate_kbps']}[name]
        for r in rows:
            for g in grp:
                r.setdefault(g, None)
        write_csv(summarize(rows, grp), os.path.join(a.out, '%s_summary.csv' % name))
        if name == 'E1':
            tests = {}
            for b in ('route_typed', 'route_all', 'rules', 'ndn_ondemand'):
                for sc in SCEN[1:5]:
                    rs = [r for r in rows if r['scenario'] == sc]
                    tests['%s|cognitive-%s|cov_semantic' % (sc, b)] = paired(rs, 'cognitive', b, 'cov_semantic',
                                                                             ['speed', 'rate_kbps'])
                    tests['%s|cognitive-%s|kb_per_km' % (sc, b)] = paired(rs, 'cognitive', b, 'kb_per_km',
                                                                          ['speed', 'rate_kbps'])
            report['E1_tests'] = tests
            fig_e1(rows, a.out)
        if name == 'E2':
            fig_e2(rows, a.out)
        if name == 'E3':
            fig_e3(rows, a.out)
        if name == 'E5':
            fig_e5(rows, a.out)
        if name == 'E6':
            report['E6_tests'] = {'cognitive-%s|%s' % (b, m): paired(rows, 'cognitive', b, m, ['speed', 'rate_kbps'])
                                  for b in ('route_typed', 'rules') for m in ('cov_all', 'kb_per_km')}
        print('%s: %d runs' % (name, len(rows)))
    with open(os.path.join(a.out, 'tests.json'), 'w') as f:
        json.dump(report, f, indent=1, default=str)


if __name__ == '__main__':
    main()
