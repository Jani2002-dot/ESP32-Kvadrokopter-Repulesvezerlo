#!/usr/bin/env python3
"""Capture ESP32 serial log, or analyse saved log. Python 3.10+."""
import argparse
import csv
import io
import json
from pathlib import Path
import threading
import time


def capture(args):
    import serial
    path = Path(args.output)
    if path.exists():
        raise SystemExit(f'File already exists: {path}; choose another filename.')
    stop = threading.Event()
    with serial.Serial(args.port, args.baud, timeout=0.2, write_timeout=1) as port:
        # Opening an ESP32 serial connection may reset the board. Keep boot messages in log.
        def commands():
            while not stop.is_set():
                try:
                    command = input().strip()
                except EOFError:
                    return
                if command in ('q', 'quit'):
                    stop.set()
                    return
                if command not in ('1', '2', 's', 'd', 'm', '0', '?'):
                    print('Commands: 1 2 s d m 0 ?; q exits capture')
                    continue
                port.write((command + '\n').encode('ascii'))
        threading.Thread(target=commands, daemon=True).start()
        print('Recording. Type 1/2 to start, s/d selects static/step, m marker, 0 stop, q exit.')
        print('Warm sensor up for 2 minutes before first comparison run.')
        with path.open('x', encoding='utf-8', newline='') as output:
            buffer = bytearray()
            try:
                while not stop.is_set():
                    chunk = port.read(port.in_waiting or 1)
                    buffer.extend(chunk)
                    while b'\n' in buffer:
                        raw, _, rest = buffer.partition(b'\n')
                        buffer = bytearray(rest)
                        line = raw.decode('utf-8', errors='replace').rstrip('\r')
                        output.write(line + '\n')
                        output.flush()
                        if not line or line[0].isdigit():
                            continue
                        print(line)
            except KeyboardInterrupt:
                pass
            finally:
                stop.set()
                try:
                    port.write(b'0\n')
                except serial.SerialException:
                    pass
                if buffer:
                    output.write(buffer.decode('utf-8', errors='replace'))
    print(f'Saved: {path}')


def load_log(path):
    rows, marks, completed, cancelled = {}, {}, set(), set()
    for line in Path(path).read_text(encoding='utf-8', errors='replace').splitlines():
        if line.startswith('#MARK,'):
            _, run, timestamp, index = line.split(',')
            marks.setdefault(int(run), []).append(float(timestamp)/1e6)
        elif line.startswith('#END,'):
            completed.add(int(line.split(',')[1]))
        elif line.startswith('#CANCEL,') or line.startswith('#ABORT'):
            if line.startswith('#CANCEL,'):
                cancelled.add(int(line.split(',')[1]))
        elif line and line[0].isdigit():
            parts = next(csv.reader(io.StringIO(line)))
            if len(parts) != 10:
                continue
            try:
                row = dict(run=int(parts[0]), profile=int(parts[1]), kind=parts[2],
                           t=float(parts[3])/1e6, dt=float(parts[4])/1e6,
                           conv=float(parts[5])/1e6, p=float(parts[6]),
                           temp=float(parts[7]), h=float(parts[8]), status=int(parts[9]))
            except ValueError:
                continue
            rows.setdefault(row['run'], []).append(row)
    return rows, marks, completed, cancelled


def metrics(samples):
    import numpy as np
    t = np.array([r['t'] for r in samples])
    h = np.array([r['h'] for r in samples])
    dt = np.diff(t)
    if len(h)<3 or not np.all(np.isfinite(h)) or np.any(dt<=0):
        raise ValueError('Too few samples, non-finite values or non-increasing timestamps.')
    slope, offset = np.polyfit(t, h, 1)
    residual = h-(slope*t+offset)
    raw_sd = float(np.std(h, ddof=1))
    sd = float(np.std(residual, ddof=1))
    rho = float(np.corrcoef(residual[:-1], residual[1:])[0,1]) if sd>1e-12 else None
    # A drift-sensitive observable over longer lags, not assumed sensor white noise.
    differences = {}
    for lag_s in (0.1, 0.5, 1.0, 5.0):
        lag = max(1, round(lag_s/np.median(dt)))
        if len(h)>lag+2:
            differences[str(lag_s)] = float(np.std(h[lag:]-h[:-lag], ddof=1)/np.sqrt(2))
    return dict(n=len(h), duration_s=float(t[-1]-t[0]),
                achieved_hz=float((len(h)-1)/(t[-1]-t[0])),
                interval_median_ms=float(np.median(dt)*1000),
                interval_p95_ms=float(np.percentile(dt,95)*1000),
                conversion_median_ms=float(np.median([r['conv'] for r in samples])*1000),
                mean_cm=float(np.mean(h)), sd_raw_cm=raw_sd,
                sd_detrended_cm=sd, R_raw_cm2=raw_sd**2, R_detrended_cm2=sd**2,
                drift_cm_per_min=float(slope*60), lag1_correlation=rho,
                temperature_range_C=float(np.ptp([r['temp'] for r in samples])),
                increment_sd_over_sqrt2_cm=differences)


def step_metrics(samples, markers, height):
    import numpy as np
    t=np.array([r['t'] for r in samples]); h=np.array([r['h'] for r in samples])
    result=[]
    for i,mark in enumerate(markers):
        end=markers[i+1] if i+1<len(markers) else t[-1]
        before=h[(t>=mark-3)&(t<mark)]
        plateau=h[(t>=end-3)&(t<end)]
        if len(before)<3 or len(plateau)<3 or end-mark<5:
            continue
        initial=float(np.median(before)); final=float(np.median(plateau))
        amplitude=final-initial
        entry=dict(marker_s=mark, measured_step_cm=amplitude,
                   absolute_step_error_cm=abs(amplitude)-height,
                   plateau_sd_cm=float(np.std(plateau,ddof=1)),
                   manual_marker_to_75pct_s=None)
        if abs(amplitude)>height*0.25:
            threshold=initial+0.75*amplitude
            indices=np.where((t>=mark)&(t<end))[0]
            crosses=(h[indices]>=threshold) if amplitude>0 else (h[indices]<=threshold)
            for j in range(len(indices)-2):
                if np.all(crosses[j:j+3]):
                    entry['manual_marker_to_75pct_s']=float(t[indices[j]]-mark)
                    break
        result.append(entry)
    return result


def analyse(args):
    import numpy as np
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    rows, marks, completed, cancelled = load_log(args.input)
    if not rows:
        raise SystemExit('No valid samples found.')
    out=Path(args.output); out.mkdir(parents=True, exist_ok=True)
    report=[]
    for run,samples in sorted(rows.items()):
        m=metrics(samples)
        kind=samples[0]['kind']; profile=samples[0]['profile']
        m.update(run=run,profile=profile,kind=kind,
                 complete=run in completed and run not in cancelled)
        if kind=='step':
            m['steps']=step_metrics(samples,marks.get(run,[]),args.height_cm)
            # Whole dynamic-run variance is motion, NOT a measurement-noise R.
            for key in ('R_raw_cm2','R_detrended_cm2'):
                m[key]=None
        report.append(m)
        t=np.array([r['t'] for r in samples]); h=np.array([r['h'] for r in samples])
        fig,ax=plt.subplots(3,1,figsize=(11,9),sharex=True)
        ax[0].plot(t,h,linewidth=.8); ax[0].set_ylabel('Relative altitude (cm)')
        ax[1].plot(t,[r['temp'] for r in samples],linewidth=.8); ax[1].set_ylabel('Temperature (C)')
        ax[2].plot(t[1:],np.diff(t)*1000,linewidth=.8); ax[2].set_ylabel('Sample interval (ms)')
        ax[2].set_xlabel('Time since test start (s)')
        for a in ax:
            a.grid(alpha=.3)
            for mark in marks.get(run,[]): a.axvline(mark,color='red',alpha=.4,linestyle='--')
        fig.suptitle(f'Run {run} / profile {profile} / {kind} / complete={m["complete"]}')
        fig.tight_layout(); fig.savefig(out/f'run_{run}_profile_{profile}.png',dpi=150); plt.close(fig)
        print(f'Run {run}, profile {profile}, {kind}, complete={m["complete"]}: '
              f'{m["achieved_hz"]:.2f} Hz; SD={m["sd_raw_cm"]:.3f} cm; '
              f'detrended SD={m["sd_detrended_cm"]:.3f} cm; '
              f'drift={m["drift_cm_per_min"]:.3f} cm/min; rho1={m["lag1_correlation"]}')
    (out/'summary.json').write_text(json.dumps(report,indent=2,allow_nan=False),encoding='utf-8')
    scalar_keys=[k for k in report[0] if k not in ('steps','increment_sd_over_sqrt2_cm')]
    with (out/'summary.csv').open('w',newline='',encoding='utf-8') as f:
        w=csv.DictWriter(f,fieldnames=scalar_keys,extrasaction='ignore'); w.writeheader(); w.writerows(report)
    print(f'Output: {out.resolve()}')
    print('Use STATIC runs for R; step variance includes real motion. No automatic winner.')
    print('Strong correlation invalidates an independent-noise interpretation of R.')
    print('Step delays are relative to MANUAL markers and include movement/operator delay.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='command',required=True)
    cap=sub.add_parser('capture'); cap.add_argument('--port',required=True)
    cap.add_argument('--baud',type=int,default=460800)
    cap.add_argument('--output',default='bmp280_log.csv'); cap.set_defaults(func=capture)
    ana=sub.add_parser('analyse'); ana.add_argument('input')
    ana.add_argument('--output',default='bmp280_results'); ana.add_argument('--height-cm',type=float,default=50)
    ana.set_defaults(func=analyse)
    args=parser.parse_args(); args.func(args)
if __name__=='__main__': main()
