# Named offline baseline-first hybrid

Rule: retain the original probe whenever its saved `detected` flag (`n > 0`) is
true; otherwise use the saved body-right Holistic probe. This composes existing
unchanged-depth A/B outputs. **No new inference, fusion, thresholds, runtime
selection, or physical control was introduced.** All 2,857 attempts remain.

The candidate provides **1,929 depth-supported surface positions**: all **1,906**
independently replayed baseline positions plus **23 gains**, with zero baseline
position losses. The 23 gained frame IDs are:

```text
8, 9, 150–165, 2444, 2446, 2465, 2466, 2555
```

These are the original-n0 subset of Holistic's 28 gains. The five other gains
are **7, 262, 1957, 2387, 2388**; the original already selected a hand there but
its depth fit failed, so this named rule deliberately keeps that baseline result.

The palm-down block's four gains form **one-, one-, and two-frame runs**, with a
maximum first-to-last sample span of **33.35 ms**. There is no sustained recovery
of that natural pose. Longest whole-recording valid and invalid runs remain
**578** and **419 frames**, respectively, as in the baseline.

Source/reference switching is material. Frame **2554→2555** changes from the
baseline surface reference to the Holistic reference and jumps **156.33 mm** over
**66.696 ms**. The existing translation bound for that interval is **115.04 mm**
(`0.015 + 1.5 * dt`), so the geometric input exceeds it. The other adjacent valid
source switches, **165→166** and **2555→2556**, are below that bound.

Original source track IDs are preserved. Ordinary summary logic consequently
excludes those three identity/source transitions; the explicit cross-source
audit reports them so a discontinuity cannot disappear from the results. This
is not a controller replay: an actual controller would also apply identity and
re-engagement rules. The experiment does not define a persistent identity
policy or justify overwriting IDs to blend sources.

The baseline-first rule also preserves the original wrist-reference error
visible around 2541–2553: a reference on the forearm differs substantially from
the Holistic hand-base reference even though both depth fits pass. See
`live-01-holistic.reference-difference.jpg`. More accepted depth frames are not
proof of correct anatomical wrist localization.

**Recommendation:** keep this candidate offline. It provides neither a sustained
palm-down XYZ recovery nor a safe continuous cross-source reference. On camera
reconnection, the already identified near-range/native-depth/IR diagnostics
remain a stronger next step for the dominant missing-depth problem, while the
body-association evidence remains useful for a separately validated wrist/hand
identity design. No narrower pose range, threshold relaxation, or prediction
as measured position is supported by this result.

Artifacts: `live-01-hybrid.jsonl`, `.summary.json`, `.inputs.json`, and
`.analyzer.py` contain every selected source, unchanged observation/diagnostics,
complete runs, original-input hashes and cross-source guard calculations.
