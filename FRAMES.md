# Wrist-local joint frames from MediaPipe landmarks

`telehand/handpose.py` turns MediaPipe's 21 world landmarks into a coordinate
frame at the palm and at every joint of the chosen fingers. MediaPipe does not
report joint rotations; everything below is reconstructed from point geometry
only, and nothing here pretends otherwise.

Scope of this version: **right hand only**, thumb and index chains, replay from
recordings. No DH116 control is connected.

## 0. Notation

* `p_i` — MediaPipe world landmark `i` (metres, hand-relative), i = 0…20.
  Wrist 0; thumb 1–4 (CMC, MCP, IP, tip); index 5–8 (MCP, PIP, DIP, tip);
  middle 9–12; ring 13–16; pinky 17–20.
* `unit(v) = v / |v|`.
* A frame is a proper rotation `R = [x y z]` (columns are unit axes) with
  `det R = +1`, plus an origin.

## 1. Input adapter: canonical right-hand landmarks

Everything downstream of the adapter sees right-hand geometry. The adapter is
the **only** reflection in the pipeline.

**Why the label can't be used directly.** MediaPipe's world landmarks carry the
chirality of its handedness label: a physical right hand through a mirrored
webcam is labelled "Left" and its landmarks are the mirror image of a right
hand. But the label is unreliable per frame in both directions. On
`fk_session.npz` (730 frames, one physical right hand, mirrored webcam):

| frame | label | curl vote | what actually happened |
|---|---|---|---|
| 0 | Right | 0 (open hand) | pure label flicker — geometry did **not** flip; mirroring by label adds a 146° jump |
| 1–727 | Left | −230 typical | left geometry, as expected |
| 728–729 | Right | +279, +229 | geometry **did** flip with the label; not mirroring gives a 174.6° jump |

So a fixed transform is wrong at 728–729 and a label-driven transform is wrong
at frame 0.

**Chirality from the geometry itself.** A curled fingertip lies on the palmar
side of the palm plane. With `y0 = p5 − p17`, `z0 = mean(p5, p9, p13) − p0` and
`n = unit(y0 × z0)`: under a reflection the cross product flips relative to the
mirrored tip displacement, so

    sign( (p_tip − p_mcp) · n )

reads the geometry's own chirality (+ right, − left), independent of any label.
Each of the four fingers votes with its curl beyond 60° (`chirality_vote`); a
vote of magnitude ≥ 20 is confident. Measured: confident votes are ±230, and
over 533 confident frames the vote disagreed with the label **0** times.

**Decision rule** (`Canonicalizer`):

1. a confident vote decides, immediately — a genuine flip is caught in the
   frame it happens;
2. with no curl (open hand) the previous decision carries over;
3. before any evidence, the default comes from the webcam setup
   (`assume_mirrored` → left geometry), not from the label.

If the decision is *left*, `mirror_x` negates world x. Which axis is mirrored
is immaterial for anything wrist-local — mirror-x, -y and -z give identical
frames to 0.0 — because two reflections differ by a rotation, and the wrist
frame removes rotations.

**Invariants checked on the recording after canonicalization** (all frames):
`det R = +1` (7e-16), thumb on +y 99.7 %, distal +z 100 %, curled index tip on
+x 100 % of 199 curled frames, index MCP at larger y than pinky MCP 100 %, the
six existing angle signals unchanged to 0.0 (a reflection is an isometry).

## 2. Palm frame

Origin: the wrist, `p0`.

1. Least-squares plane through `{p0, p5, p9, p13, p17}`: subtract the centroid,
   SVD, the normal `n` is the last right-singular vector. Its sign is
   arbitrary. Planarity `σ3/σ2` is reported (0.12–0.35 measured; > 0.7 means
   the five points do not define a plane and palm confidence is zeroed).
2. `y0 = p5 − p17` (pinky MCP → index MCP), `z0 = mean(p5, p9, p13) − p0`.
3. **Sign of the normal:** require `n · (y0 × z0) > 0`. For a right hand this
   points out of the palm on the palmar side. Margin
   `m = |n · (y0 × z0)| / |y0 × z0|` is reported (measured min 0.996). Only if
   `m < 0.2` — never observed — is the sign taken from the previous frame's
   normal, so a confident geometric answer is never overridden by history and
   an occlusion cannot lock in a wrong sign.
4. `x = n`, `z = unit(z0 − (z0 · x) x)`, `y = z × x`.

Then `x × y = z`, so `(x, y, z)` is right-handed and `det R_W = +1`. `z0` is
always nearly perpendicular to `n` (measured `|z0 · n|` max 0.009), so the
orthogonalisation in step 4 cannot degenerate.

Semantics: `+x` out of the palm, `+y` toward the thumb/index side, `+z`
distal. This matches the DH116 URDF / MANO convention.

## 3. Chain frames: minimal-rotation propagation

For unit bone directions `a → b`, with `v = a × b` and `c = a · b`:

    R_min(a → b) = I + [v]× + [v]×² / (1 + c)

exact for all `c > −1`, identity when `a = b`. Then, for a chain with bones
`b_1, b_2, b_3` (proximal to distal):

* root (index MCP, thumb CMC): `R_root = R_min(z_W → b_1) · R_W` — the palm
  frame rotated the shortest way so its z lies along the first bone; the
  palm's twist is inherited.
* along the chain: `R_{j+1} = R_min(b_j → b_{j+1}) · R_j`.
* the tip has no child bone and no rotation of its own; it carries the last
  bone's frame.

Every joint's local `z` is its child bone. There is **no reference vector**,
so no near-parallel degeneracy anywhere; the only singularity is antiparallel
consecutive bones (`c → −1`, a 180° fold no joint can make), guarded by a half
turn about the parent frame's `x`. A zero-length bone (< 0.1 mm) keeps the
parent's direction and is flagged `degenerate`.

Why not Gram–Schmidt against a fixed palm axis at every joint: it works for the
fingers (`|palm_y × bone|` p1 = 0.887 measured) but is weakest exactly on the
thumb (min 0.552), whose bones sweep toward the palm's y axis.

Property that matters: the relative rotation `R_jᵀ R_{j+1} = R_min(b_j → b_{j+1})`
expressed in the parent's frame is a pure rotation about an axis perpendicular
to both bones — **zero twist by construction**. Its angle is exactly
`tracker._angle_between` on the same points, so the representation is a strict
superset of today's angle signals (verified to 2.5e-13°).

## 4. Wrist-local expression

    q_i        = R_Wᵀ (p_i − p0)            positions
    R_i^local  = R_Wᵀ R_i                    joint rotations

The palm joint is the origin with identity rotation. Applying any rigid
transform `(R_g, t_g)` to the input leaves every `q_i` and `R_i^local`
unchanged (verified: 100 random SE(3) transforms, max |Δpos| 3e-16 m, max
|ΔR| 1e-14). Quaternions are `scipy` order `(x, y, z, w)`.

## 5. What can and cannot flip

Bone directions are ordered proximal → distal (no ambiguity). `R_min` has no
ambiguity. The SVD normal in §2 is the **single** sign-ambiguous quantity, and
it is disambiguated geometrically with a measured margin ≥ 0.996. Upstream of
that, the only sign decision is the adapter's chirality (§1), which is decided
by geometry with a measured margin of ±230 against a threshold of 20.

## 6. Confidence

MediaPipe sets no per-landmark visibility/presence for the hand task. Derived:

* palm: the sign margin, zeroed if planarity > 0.7;
* joints: each bone is rigid, so a length far outside its own recent spread is
  evidence the landmarks moved somewhere they could not. With `(median, scale)`
  per bone over the last 90 frames, `scale = max(1.4826·MAD, 8 % of median)` —
  the floor matches the ~9 % pose-dependent length variation MediaPipe shows
  on this rig, so the gate flags gross faults, not fists —
  a deviation under 3 scales keeps confidence 1.0 and 6 scales is 0.0. A joint
  takes the worst of its adjacent bones; a degenerate joint is 0.0.

## 7. What is deliberately not claimed

`relative_rotation(name)` (= `R_parentᵀ R_child`) and its axis-angle
`relative_rotvec(name)` are exposed **without** anatomical labels. Whether a
given component is "flexion", "abduction" or "opposition" for a given joint —
and which candidate (projected angles, swing–twist, axis-angle in the palm
frame, or the existing signals) maps most cleanly onto DH116 thumb abduction
and flexion — is to be established empirically, not assumed.

## 8. Validation

`python3 tests/test_handpose.py` — synthetic hands with known frames.

`python3 run.py frames --replay fk_session.npz` — every check on real data:
`det R`, `RᵀR`, wrist-locality, rigid invariance, palm-normal sign stability,
180° flips, bend angles vs `_angle_between`, geodesic jitter per joint, the
high-frequency residual of the new rotations against that of the existing
signals, and the chirality self-check (curled index tip on +x).
