/*****************************************************************************
 *  HapticsDirect.cpp  —  Linux replacement for the 3D Systems "Haptics Direct"
 *                        Unity native plugin.
 *
 *  Build:  g++ -O2 -fPIC -shared -o libHapticsDirect.so HapticsDirect.cpp -lHD -lHDU
 *  Drop:   Assets/Plugins/x86_64/libHapticsDirect.so   (importer -> Linux x86_64)
 *
 *  HapticPlugin.cs is NOT modified. This file exports the same 27 symbols with
 *  the same signatures, so Mono's DllImport("HapticsDirect") resolves against
 *  libHapticsDirect.so unchanged.
 *
 *  ---------------------------------------------------------------------------
 *  CONFIDENCE MARKERS. Read these before trusting anything.
 *
 *    [V] Verified against a source I actually read: HapticPlugin.cs (its own
 *        signatures and doc comments), HD/hdDefines.h, or the OpenHaptics
 *        examples AnchoredSpringForce.c and PointManipulation's
 *        HapticDeviceManager.cpp.
 *
 *    [A] Assumed. Plausible, consistent with the above, NOT confirmed. Every
 *        one of these is a place the Windows DLL may differ. Grep for "[A]".
 *
 *  The single largest [A] is the meaning of `magnitude` in setSpringValues and
 *  setConstantForceValues. See SPRING GAIN below.
 *  ---------------------------------------------------------------------------
 *
 *  PARTIALLY IMPLEMENTED: addContactPointInfo. The stiffness and damping terms
 *  are now written against FrictionlessSphere.cpp's penalty form. Static and
 *  dynamic friction and pop-through are still absent — deriving those from
 *  signatures alone would be invention. They report zero and are logged.
 *****************************************************************************/

#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <map>
#include <mutex>
#include <string>
#include <vector>

// ===========================================================================
// Coordinate conversion
// ===========================================================================
//
// [V] HapticPlugin.cs getPosition: "position in mm ... Left is + x, up is +y,
//     toward user is +z. (Unity CSys)". OpenHaptics native is right-handed with
//     +x to the RIGHT. Units are mm on BOTH sides — there is no metre scaling.
// [A] The conversion is therefore a pure x-negation, applied to every vector in
//     both directions. If the plugin also flips z, forces invert along depth;
//     that shows up immediately on a stiff contact, so it is cheap to test.

static inline void devToUnity3(const double* in, double* out) {
    out[0] = -in[0]; out[1] = in[1]; out[2] = in[2];
}
static inline void unityToDev3(const double* in, double* out) {
    out[0] = -in[0]; out[1] = in[1]; out[2] = in[2];
}

// [V] HD_CURRENT_TRANSFORM is column-major: HapticDeviceManager.cpp writes the
//     proxy translation into elements 12,13,14.
// [A] Handedness flip of a 4x4 is C*M*C with C = diag(-1,1,1,1), i.e. negate
//     every element whose row-or-column parity across x is odd. Verify against
//     the Windows build before relying on the rotation part; the translation
//     part (12,13,14) is just the x-negation and is safe.
static void devToUnityMat16(const double* m, double* out) {
    for (int c = 0; c < 4; ++c)
        for (int r = 0; r < 4; ++r) {
            const int i = c * 4 + r;
            const bool flip = ((r == 0) != (c == 0));   // exactly one index is x
            out[i] = flip ? -m[i] : m[i];
        }
}

// ===========================================================================
// Seqlock — Unity thread and 1 kHz servo thread exchange without blocking
// ===========================================================================
//
// AnchoredSpringForce.c uses hdScheduleSynchronous to change parameters safely.
// That is correct but blocks the caller until the next servo tick (up to 1 ms).
// setAnchorPosition is called every Unity frame per device, so a seqlock is
// used instead: writers bump an odd/even counter, readers retry on tear.

template <typename T>
class Seq {
public:
    void store(const T& v) {
        uint32_t s = seq_.load(std::memory_order_relaxed);
        seq_.store(s + 1, std::memory_order_release);
        std::atomic_thread_fence(std::memory_order_release);
        data_ = v;
        std::atomic_thread_fence(std::memory_order_release);
        seq_.store(s + 2, std::memory_order_release);
    }
    T load() const {
        T out;
        uint32_t before, after;
        do {
            before = seq_.load(std::memory_order_acquire);
            if (before & 1u) continue;
            std::atomic_thread_fence(std::memory_order_acquire);
            out = data_;
            std::atomic_thread_fence(std::memory_order_acquire);
            after = seq_.load(std::memory_order_acquire);
        } while (before != after || (before & 1u));
        return out;
    }
private:
    std::atomic<uint32_t> seq_{0};
    T data_{};
};

// ===========================================================================
// Per-device data
// ===========================================================================

struct Vec3 { double v[3] = {0, 0, 0}; };

// Fixed-size, deliberately. The seqlock copies Params wholesale and a reader
// can observe a torn copy mid-write; a std::vector in there would be a
// use-after-free waiting to happen. Overflow past kMaxContacts is dropped.
static const int kMaxContacts = 16;

struct Contact {
    double location[3]  = {0, 0, 0};   // device frame, mm
    double normal[3]    = {0, 0, 0};   // device frame, unit, outward from surface
    double depth        = 0.0;         // mm penetration
    double stiffness    = 0.0;         // N/mm
    double damping      = 0.0;         // N*s/mm
    double viscosity    = 0.0;
    double frictionSta  = 0.0;         // unused - see effectCB
    double frictionDyn  = 0.0;         // unused - see effectCB
};

// Parameters: written by Unity thread, read by servo thread.
struct Params {
    bool   springOn      = false;
    double springAnchor[3] = {0, 0, 0};   // device frame, mm — setSpringValues
    double springGain    = 0.0;           // see SPRING GAIN
    double proxyAnchor[3] = {0, 0, 0};    // device frame, mm — setAnchorPosition

    bool   constOn       = false;
    double constDir[3]   = {0, 0, 0};     // device frame, expected unit length
    double constMag      = 0.0;           // N

    bool   vibOn         = false;
    double vibDir[3]     = {0, 0, 0};
    double vibMag        = 0.0;           // N
    double vibFreq       = 0.0;           // Hz
    double vibRemaining  = 0.0;           // s; <=0 disables

    bool   gravityOn     = false;
    double gravity[3]    = {0, 0, 0};     // N, device frame

    bool   extraOn       = false;         // one-shot from setForce()
    double extraForce[3] = {0, 0, 0};
    double extraTorque[3]= {0, 0, 0};

    int     contactCount = 0;
    Contact contacts[kMaxContacts];
};

// State: written by servo thread, read by Unity thread.
struct State {
    double position[3]   = {0, 0, 0};
    double velocity[3]   = {0, 0, 0};
    double transform[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
    double jointAngles[3]= {0, 0, 0};
    double gimbalAngles[3]={0, 0, 0};
    int    buttons       = 0;
    int    lastButtons   = 0;
    int    inkwell       = 0;
    double force[3]      = {0, 0, 0};     // total commanded, device frame

    // Term breakdown -> getGlobalForces / getLocalForces.
    double fSpring[3]    = {0, 0, 0};
    double fConstant[3]  = {0, 0, 0};
    double fVibration[3] = {0, 0, 0};
    double fGravity[3]   = {0, 0, 0};
    double fStiffness[3] = {0, 0, 0};     // contact terms: always zero until
    double fViscosity[3] = {0, 0, 0};     // addContactPointInfo is implemented
    double fFrictionDyn[3]={0, 0, 0};
    double fFrictionSta[3]={0, 0, 0};
};

struct Device {
    std::string name;
    HHD hHD = HD_INVALID_HANDLE;


    Seq<Params> params;
    Seq<State>  state;

    // Unity-thread only: staged between resetContactPointInfo and
    // updateContactPointInfo, published to params on update.
    int     stagingCount = 0;
    Contact staging[kMaxContacts];

    // Servo-thread only.
    Params  p;
    State   s;
    double  accum[3]    = {0, 0, 0};
    double  vibPhase    = 0.0;
    int     diagTick    = 0;
    double  dt          = 1.0 / 1000.0;

    // Cached device constants.
    double maxStiffness = 1.0;   // N/mm
    double maxDamping   = 0.0;   // N*s/mm
    double maxForce     = 3.3;   // N
    double usable6[6]   = {0};
    double max6[6]      = {0};
    char   serial[64]   = {0};
    char   model[64]    = {0};
};

static std::map<std::string, Device*> gDevices;
static std::mutex        gDevicesMutex;
static bool              gSchedulerRunning = false;
static HDSchedulerHandle gServoHandle      = 0;
static std::atomic<int>  gLastError{HD_SUCCESS};
static char              gLastErrorMsg[256] = {0};

static Device* find(const char* name) {
    if (!name) return nullptr;
    std::lock_guard<std::mutex> lk(gDevicesMutex);
    auto it = gDevices.find(name);
    return it == gDevices.end() ? nullptr : it->second;
}


// ===========================================================================
// Call trace — set HAPTICSDIRECT_LOG=1 to record what the C# actually calls
// ===========================================================================
static FILE* gTrace = nullptr;
static bool  gTraceChecked = false;
static FILE* trace() {
    if (!gTraceChecked) {
        gTraceChecked = true;
        const char* e = getenv("HAPTICSDIRECT_LOG");
        if (e && *e == '1') gTrace = fopen("/tmp/hapticsdirect.log", "a");
    }
    return gTrace;
}
#define TRACE(...) do { FILE* f_ = trace(); \
    if (f_) { fprintf(f_, __VA_ARGS__); fputc('\n', f_); fflush(f_); } } while (0)

static void recordError(const char* context) {
    HDErrorInfo e = hdGetError();
    if (e.errorCode != HD_SUCCESS) {
        gLastError.store((int)e.errorCode);
        snprintf(gLastErrorMsg, sizeof(gLastErrorMsg), "%s: %s (0x%04x)",
                 context, hdGetErrorString(e.errorCode), e.errorCode);
    }
}

// ===========================================================================
// Servo callbacks — three per device, descending priority
// ===========================================================================
//
// [V] HapticDeviceManager.cpp uses exactly this shape:
//       BEGIN  = HD_MAX_SCHEDULER_PRIORITY       zero the accumulator
//       EFFECT = HD_MAX_SCHEDULER_PRIORITY - 1   m_effectForce += force
//       END    = HD_MAX_SCHEDULER_PRIORITY - 2   one hdSetDoublev, then endFrame
//     hdSetDoublev sets rather than accumulates, so this ordering is what keeps
//     a vibration from silently erasing a spring.
// [A] resetContactPointInfo / addContactPointInfo / updateContactPointInfo map
//     onto begin / accumulate / commit respectively. Structurally it fits, but
//     it is inference from the naming, not confirmed.

static void deviceBegin(Device* d) {
    // hdMakeCurrentDevice is MANDATORY here, not optional. Every hdGet*/hdSet*
    // below acts on whatever device is current, not on a device handle.
    hdMakeCurrentDevice(d->hHD);
    hdBeginFrame(d->hHD);

    hdGetDoublev(HD_CURRENT_POSITION,      d->s.position);
    hdGetDoublev(HD_CURRENT_VELOCITY,      d->s.velocity);
    hdGetDoublev(HD_CURRENT_TRANSFORM,     d->s.transform);
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES,  d->s.jointAngles);
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, d->s.gimbalAngles);
    hdGetIntegerv(HD_CURRENT_BUTTONS,      &d->s.buttons);
    hdGetIntegerv(HD_LAST_BUTTONS,         &d->s.lastButtons);
    // [V] HD_INVALID_INPUT_TYPE (0x0103) on the servo loop was this line read
    //     with hdGetIntegerv. The inkwell switch is a boolean parameter and
    //     needs hdGetBooleanv; HDboolean is unsigned char, not int.
    // [A] POLARITY: measured as 1 when the stylus is OUT of the inkwell and 0
    //     when docked — i.e. raw HD reports the inverse of "docked". Whether
    //     the Windows DLL passes this through or inverts it is unverified;
    //     HapticPlugin.cs only says "whether the inkwell switch is active".
    //     Passed through unchanged here. To invert, negate the ternary below.
    HDboolean ink = 0;
    hdGetBooleanv(HD_CURRENT_INKWELL_SWITCH, &ink);
    d->s.inkwell = ink ? 1 : 0;

    double rate = 1000.0;
    hdGetDoublev(HD_INSTANTANEOUS_UPDATE_RATE, &rate);
    d->dt = (rate > 1.0) ? 1.0 / rate : 0.001;

    d->accum[0] = d->accum[1] = d->accum[2] = 0.0;
    memset(d->s.fSpring,     0, sizeof(d->s.fSpring));
    memset(d->s.fConstant,   0, sizeof(d->s.fConstant));
    memset(d->s.fVibration,  0, sizeof(d->s.fVibration));
    memset(d->s.fGravity,    0, sizeof(d->s.fGravity));
    memset(d->s.fStiffness,  0, sizeof(d->s.fStiffness));
    memset(d->s.fViscosity,  0, sizeof(d->s.fViscosity));
    memset(d->s.fFrictionDyn,0, sizeof(d->s.fFrictionDyn));
    memset(d->s.fFrictionSta,0, sizeof(d->s.fFrictionSta));

    d->p = d->params.load();
}

static void deviceEffects(Device* d) {

    // ---- SPRING ----------------------------------------------------------
    // [V] AnchoredSpringForce.c:  F = k * (anchor - position), position in mm,
    //     k printed as N/mm, k clamped to HD_NOMINAL_MAX_STIFFNESS.
    //     HapticDeviceManager.cpp uses the same form for its proxy force and
    //     caps the practical stiffness at min(0.4, HD_NOMINAL_MAX_STIFFNESS).
    //
    // SPRING GAIN [V] — MEASURED AGAINST THE WINDOWS DLL.
    //     `magnitude` is a FRACTION OF HD_NOMINAL_MAX_FORCE, not raw N/mm:
    //         k[N/mm] = magnitude * HD_NOMINAL_MAX_FORCE
    //     Measured by running the same scene on both platforms and fitting
    //     |springForce| against separation over unsaturated samples: with
    //     magnitude = 0.012 the Windows DLL yields k = 0.0387 N/mm, an implied
    //     multiplier of 3.22 against a nominal max force of 3.3 N (2.3% off,
    //     within the scatter of a hand-moved fit).
    //
    //     An earlier note here claimed raw N/mm and marked itself verified on
    //     the strength of a standalone harness run. That test passed 0.1 to
    //     THIS code and measured 0.1 back out — it only ever confirmed internal
    //     consistency and could not say what the Windows DLL does, because the
    //     Windows DLL was never in the loop. The cross-platform run is the
    //     first real measurement.
    if (d->p.springOn && d->p.springGain != 0.0) {
        double k = d->p.springGain * d->maxForce;      // [V] see SPRING GAIN
        if (k > d->maxStiffness) k = d->maxStiffness;   // [V] example clamps
        for (int i = 0; i < 3; ++i) {
            d->s.fSpring[i] = k * (d->p.springAnchor[i] - d->s.position[i]);
            d->accum[i] += d->s.fSpring[i];
        }
    }

    // ---- CONSTANT FORCE --------------------------------------------------
    // [A] maxForce normalisation applied BY ANALOGY with the spring, not by
    //     measurement. The three global-effect setters form one family in the
    //     C#, with matching inspector fields SpringGMag / ConstForceGMag /
    //     VibrationGMag, so a shared convention is the natural reading — but
    //     only the spring has actually been measured against the Windows DLL.
    //     To settle it: spring off, no contacts, fixed magnitude, compare the
    //     probe's constmag column across platforms. No fitting needed, since
    //     constant force has no displacement term — just keep the commanded
    //     magnitude well under 3.3 N so the clamp stays out of it.
    if (d->p.constOn) {
        for (int i = 0; i < 3; ++i) {
            d->s.fConstant[i] = d->p.constDir[i] * d->p.constMag * d->maxForce;
            d->accum[i] += d->s.fConstant[i];
        }
    }

    // ---- VIBRATION -------------------------------------------------------
    // [V] Vibration.c: sinusoidal, and the timer is accumulated exactly as
    //     here — timer += 1.0 / HD_INSTANTANEOUS_UPDATE_RATE.
    //
    // [V] AND NOTE THE MISSING 2*pi. The example computes
    //         sin(timer * gVibrationFreq)
    //     with gVibrationFreq commented "/* Hz */" and its menu labelled
    //     "frequency". There is no 2*pi anywhere. So the argument is really
    //     angular frequency in rad/s, and the stated "Hz" is off by 2*pi —
    //     a nominal 100 "Hz" is about 15.9 Hz in reality. My first draft
    //     inserted the 2*pi and would have run every vibration 6.28x too fast.
    //     Matched to the example here on the assumption Haptics Direct
    //     inherited the same code. If your vibrations come out too slow on
    //     Linux, put the 2*pi back — that is the whole fix.
    //
    // [A] `time` still unresolved: Vibration.c has no duration concept at all,
    //     so that argument is Haptics Direct's own addition. Duration assumed.
    if (d->p.vibOn && d->p.vibRemaining > 0.0) {
        d->vibPhase += d->dt;                       // plain elapsed seconds
        // [A] Same maxForce normalisation as the spring and constant force,
        //     same analogy, same lack of measurement. Unmeasured because the
        //     2*pi question below means an amplitude comparison would confound
        //     gain with frequency unless the frequency is matched first.
        const double a = d->p.vibMag * d->maxForce * std::sin(d->vibPhase * d->p.vibFreq);
        for (int i = 0; i < 3; ++i) {
            d->s.fVibration[i] = d->p.vibDir[i] * a;
            d->accum[i] += d->s.fVibration[i];
        }
        d->p.vibRemaining -= d->dt;
    } else {
        d->vibPhase = 0.0;
    }

    // ---- GRAVITY ---------------------------------------------------------
    // NOT normalised, deliberately. setGravityForce takes a bare vector with no
    // separate magnitude argument, so it is a different shape from the three
    // effect setters above and there is no structural reason to assume it
    // shares their convention. The name also suggests it may be an
    // acceleration rather than a force. Left as passed until measured.
    if (d->p.gravityOn) {
        for (int i = 0; i < 3; ++i) {
            d->s.fGravity[i] = d->p.gravity[i];
            d->accum[i] += d->s.fGravity[i];
        }
    }

    // ---- setForce() extra ------------------------------------------------
    if (d->p.extraOn) {
        for (int i = 0; i < 3; ++i) d->accum[i] += d->p.extraForce[i];
        // Torque ignored: the Touch has no actuated gimbal. Touch X / Premium
        // 6DOF would need hdSetDoublev(HD_CURRENT_GIMBAL_TORQUE, ...).
    }

    // ---- CONTACT TERMS ---------------------------------------------------
    // [V] FrictionlessSphere.cpp is Hooke's law on penetration depth:
    //         f = k * penetrationDistance * outwardUnitNormal
    //     with k = 0.25 N/mm for a firm surface and position in mm. Unity has
    //     already done the collision detection, so it hands down the contact
    //     point and normal and there is no geometry query to do here.
    //
    // [A] MatStiffness arrives as a float and is taken as N/mm directly, the
    //     same reading as springGain. If it turns out to be normalised 0..1
    //     against HD_NOMINAL_MAX_STIFFNESS, multiply by d->maxStiffness here.
    //     Same single measurement resolves both: getLocalForces reports the
    //     stiffness term separately.
    //
    // [A] Penetration depth. ImpulseDepth is passed by C# and stored, but
    //     whether it is the depth in mm or something Unity-side is unverified,
    //     so depth is recomputed from the contact plane instead:
    //         depth = dot(location - position, normal)
    //     which is the distance the device has travelled past the surface.
    //     Cross-check the two against each other at the rig.
    for (int c = 0; c < d->p.contactCount; ++c) {
        const Contact& ct = d->p.contacts[c];

        double rel[3];
        for (int i = 0; i < 3; ++i) rel[i] = ct.location[i] - d->s.position[i];
        double depth = rel[0]*ct.normal[0] + rel[1]*ct.normal[1] + rel[2]*ct.normal[2];
        if (depth <= 0.0) continue;               // not penetrating

        double k = ct.stiffness;
        if (k > d->maxStiffness) k = d->maxStiffness;   // [V] examples clamp

        for (int i = 0; i < 3; ++i) {
            const double f = k * depth * ct.normal[i];
            d->s.fStiffness[i] += f;
            d->accum[i]        += f;
        }

        // [V] HapticDeviceManager.cpp's damping term: f = -b * velocity.
        // [A] Whether Haptics Direct applies damping along the normal only or
        //     in full 3D is unverified; full 3D here.
        if (ct.damping != 0.0) {
            for (int i = 0; i < 3; ++i) {
                const double f = -ct.damping * d->s.velocity[i];
                d->s.fViscosity[i] += f;
                d->accum[i]        += f;
            }
        }

        // NOT YET IMPLEMENTED: static/dynamic friction and pop-through. Both
        // are still owed for a complete port. Pop-through is currently passed
        // as a hardcoded 0.0f by HapticPlugin.cs, so no scene can exercise it
        // today — that lowers its test priority, not its priority. The
        // friction transition needs a stick-slip state machine per contact
        // (anchor point, break-away threshold, sliding regime) that I have no
        // grounded reference for. SlidingContact/main.cpp is the example to
        // read. fFrictionSta / fFrictionDyn therefore stay zero and
        // getCurrentFrictionForce returns zero.
    }
}

static void deviceEnd(Device* d) {

    // [A] Clamp target — evidence now points the other way from my first pass.
    //     TWO independent examples cap *effects* at HD_NOMINAL_MAX_CONTINUOUS_
    //     FORCE: HapticDeviceManager.cpp clamps its spring and damping terms to
    //     it, and Vibration.c uses it as the hard ceiling on amplitude (opening
    //     at 0.75 of it) with the explicit comment that the cap exists to keep
    //     the user away from dangerous limits. Continuous force on a Touch is
    //     far below peak.
    //
    //     Against that, getDeviceMaxValues exposes only stiffness, damping and
    //     force to C# with no continuous-force slot, which is why peak is still
    //     used here for the TOTAL. That asymmetry — effects capped at
    //     continuous, total at peak — is a guess, and it is the first suspect
    //     if forces feel stronger on Linux than they did on Windows.
    double mag = std::sqrt(d->accum[0]*d->accum[0] +
                           d->accum[1]*d->accum[1] +
                           d->accum[2]*d->accum[2]);
    if (mag > d->maxForce && mag > 0.0) {
        const double s = d->maxForce / mag;
        d->accum[0] *= s; d->accum[1] *= s; d->accum[2] *= s;
    }

    const double preMag = mag;   // magnitude before the clamp above

    hdSetDoublev(HD_CURRENT_FORCE, d->accum);
    memcpy(d->s.force, d->accum, sizeof(d->s.force));

    hdEndFrame(d->hHD);

    d->state.store(d->s);

    HDErrorInfo e = hdGetError();
    if (HD_DEVICE_ERROR(e)) {
        gLastError.store((int)e.errorCode);
        snprintf(gLastErrorMsg, sizeof(gLastErrorMsg), "servo: %s (0x%04x)",
                 hdGetErrorString(e.errorCode), e.errorCode);
        // Force errors mean HD limited or dropped what we asked for — that is
        // exactly the kind of thing that reads as "everything feels weak", so
        // it must be visible rather than swallowed.
        TRACE("SERVO-ERR %s 0x%04x %s", d->name.c_str(),
              e.errorCode, hdGetErrorString(e.errorCode));
    }

    // Throttled force diagnostic: ~2 Hz per device, only with logging on.
    if (++d->diagTick >= 500) {
        d->diagTick = 0;
        double dsp[3];
        for (int i = 0; i < 3; ++i) dsp[i] = d->p.springAnchor[i] - d->s.position[i];
        const double dspMag = std::sqrt(dsp[0]*dsp[0] + dsp[1]*dsp[1] + dsp[2]*dsp[2]);
        #define MAG3(v) std::sqrt((v)[0]*(v)[0] + (v)[1]*(v)[1] + (v)[2]*(v)[2])
        TRACE("DIAG %s pos=(%.1f,%.1f,%.1f) anchor=(%.1f,%.1f,%.1f) "
              "sep=%.1fmm k=%.5f | spring=%.3fN stiff=%.3fN visc=%.3fN "
              "const=%.3fN grav=%.3fN vib=%.3fN | contacts=%d "
              "preClamp=%.3fN postClamp=%.3fN maxF=%.3fN %s",
              d->name.c_str(),
              d->s.position[0], d->s.position[1], d->s.position[2],
              d->p.springAnchor[0], d->p.springAnchor[1], d->p.springAnchor[2],
              dspMag, d->p.springGain,
              MAG3(d->s.fSpring), MAG3(d->s.fStiffness), MAG3(d->s.fViscosity),
              MAG3(d->s.fConstant), MAG3(d->s.fGravity), MAG3(d->s.fVibration),
              d->p.contactCount, preMag, MAG3(d->s.force), d->maxForce,
              (preMag > d->maxForce) ? "<<CLAMPED" : "");
        #undef MAG3
    }
}

// ===========================================================================
// The one servo callback
// ===========================================================================
//
// [V] CoulombForceDual is the reference for multiple devices: ONE scheduler
//     callback, each device handled to completion in turn —
//         makeCurrent(A); beginFrame(A); ...; endFrame(A);
//         makeCurrent(B); beginFrame(B); ...; endFrame(B);
//
//     An earlier draft of this file scheduled begin/effect/end as three
//     separate callbacks at descending priority, copying HapticDeviceManager.
//     That example drives a SINGLE device and the pattern does not generalise:
//     with two devices the scheduler interleaves them, so device A's end
//     callback runs after device B's begin callback has already called
//     hdMakeCurrentDevice(B). A's force is then written to B and immediately
//     overwritten by B's own. Symptom: exactly one device produces force, and
//     which one depends on registration order rather than on the hardware.

static Device* gServoList[16];
static std::atomic<int> gServoCount{0};

static HDCallbackCode HDCALLBACK servoCB(void* /*userData*/) {
    const int n = gServoCount.load(std::memory_order_acquire);
    for (int i = 0; i < n; ++i) {
        Device* d = gServoList[i];
        if (!d) continue;
        deviceBegin(d);
        deviceEffects(d);
        deviceEnd(d);
    }
    return HD_CALLBACK_CONTINUE;
}

// ===========================================================================
// Exported API — matches HapticPlugin.cs exactly
// ===========================================================================

extern "C" {

void getVersionString(char* dest, int len) {
    if (!dest || len <= 0) return;
    const char* v = nullptr;
    // [A] HD_VERSION via hdGetString is the natural source; hdDefines.h defines
    //     HD_VERSION as a get-parameter enum (0x2500).
    v = (const char*)hdGetString(HD_VERSION);
    snprintf(dest, len, "%s", v ? v : "unknown");
}

int initDevice(const char* deviceName) {
    TRACE("initDevice \"%s\"", deviceName?deviceName:"(null)");
    // [V] hdInitDevice(HDstring), HD_DEFAULT_DEVICE == NULL (hdDefines.h).
    // Named init is the call the CISST group found broken on Linux — if this
    // returns HD_INVALID_HANDLE for a name that exists, check GTDD_HOME and
    // ~/.3dsystems/config before suspecting this code.
    const bool useDefault = (!deviceName || !*deviceName);
    HHD h = hdInitDevice(useDefault ? HD_DEFAULT_DEVICE : deviceName);
    recordError("hdInitDevice");
    if (h == HD_INVALID_HANDLE) return (int)HD_INVALID_HANDLE;

    Device* d = new Device();
    d->name = deviceName ? deviceName : "";
    d->hHD  = h;

    hdMakeCurrentDevice(h);
    hdEnable(HD_FORCE_OUTPUT);

    hdGetDoublev(HD_NOMINAL_MAX_STIFFNESS, &d->maxStiffness);
    hdGetDoublev(HD_NOMINAL_MAX_DAMPING,   &d->maxDamping);
    hdGetDoublev(HD_NOMINAL_MAX_FORCE,     &d->maxForce);
    hdGetDoublev(HD_USABLE_WORKSPACE_DIMENSIONS, d->usable6);
    hdGetDoublev(HD_MAX_WORKSPACE_DIMENSIONS,    d->max6);
    snprintf(d->serial, sizeof(d->serial), "%s",
             (const char*)hdGetString(HD_DEVICE_SERIAL_NUMBER));
    snprintf(d->model, sizeof(d->model), "%s",
             (const char*)hdGetString(HD_DEVICE_MODEL_TYPE));

    {
        std::lock_guard<std::mutex> lk(gDevicesMutex);
        gDevices[d->name] = d;
    }
    return (int)h;   // [V] HHD is unsigned int (hdDefines.h) -> fits the int return
}

void getDeviceSN(const char* configName, char* dest, int len) {
    Device* d = find(configName);
    if (dest && len > 0) snprintf(dest, len, "%s", d ? d->serial : "");
}

void getDeviceModel(const char* configName, char* dest, int len) {
    Device* d = find(configName);
    if (dest && len > 0) snprintf(dest, len, "%s", d ? d->model : "");
}

void getDeviceMaxValues(const char* configName, double* max_stiffness,
                        double* max_damping, double* max_force) {
    Device* d = find(configName);
    if (!d) return;
    if (max_stiffness) *max_stiffness = d->maxStiffness;
    if (max_damping)   *max_damping   = d->maxDamping;
    if (max_force)     *max_force     = d->maxForce;
}

void startSchedulers() {
    TRACE("startSchedulers");
    // [V] "Starts the Open Haptic schedulers and assigns the required internal
    //     callbacks" — the callbacks are internal, nothing managed is invoked
    //     from the servo thread. That is why no Mono attachment is needed here.
    std::lock_guard<std::mutex> lk(gDevicesMutex);

    // Publish a lock-free snapshot for the servo thread. Rebuilt on every call
    // so a second initDevice + startSchedulers picks up the new device; the
    // count is published last so the servo thread never sees a partial list.
    int n = 0;
    for (auto& kv : gDevices) {
        if (n >= (int)(sizeof(gServoList) / sizeof(gServoList[0]))) break;
        gServoList[n++] = kv.second;
    }
    gServoCount.store(n, std::memory_order_release);

    if (!gServoHandle) {
        gServoHandle = hdScheduleAsynchronous(servoCB, nullptr,
                                              HD_MAX_SCHEDULER_PRIORITY);
        recordError("hdScheduleAsynchronous");
    }
    if (!gSchedulerRunning) {
        hdStartScheduler();
        recordError("hdStartScheduler");
        gSchedulerRunning = true;
    }
}

void getWorkspaceArea(const char* configName, double* usable6, double* max6) {
    Device* d = find(configName);
    if (!d) return;
    // [A] Left as raw device-frame bounds. If the Windows build returns these
    //     x-flipped like the position getters, min/max on x need swapping too.
    if (usable6) memcpy(usable6, d->usable6, sizeof(d->usable6));
    if (max6)    memcpy(max6,    d->max6,    sizeof(d->max6));
}

void getPosition(const char* configName, double* position3) {
    Device* d = find(configName);
    if (!d || !position3) return;
    State s = d->state.load();
    devToUnity3(s.position, position3);
}

void getVelocity(const char* configName, double* velocity3) {
    Device* d = find(configName);
    if (!d || !velocity3) return;
    State s = d->state.load();
    devToUnity3(s.velocity, velocity3);
}

void getTransform(const char* configName, double* matrix16) {
    Device* d = find(configName);
    if (!d || !matrix16) return;
    State s = d->state.load();
    devToUnityMat16(s.transform, matrix16);
}

void getButtons(const char* configName, int* buttons4, int* last_buttons4,
                int* inkwell) {
    Device* d = find(configName);
    if (!d) return;
    State s = d->state.load();
    // [V] HD_DEVICE_BUTTON_1..4 are bits 0..3 (hdDefines.h). C# wants them
    //     unpacked into a 4-element array.
    for (int i = 0; i < 4; ++i) {
        if (buttons4)      buttons4[i]      = (s.buttons     & (1 << i)) ? 1 : 0;
        if (last_buttons4) last_buttons4[i] = (s.lastButtons & (1 << i)) ? 1 : 0;
    }
    if (inkwell) *inkwell = s.inkwell;
}

void getCurrentForce(const char* configName, double* currentforce3) {
    Device* d = find(configName);
    if (!d || !currentforce3) return;
    State s = d->state.load();
    devToUnity3(s.force, currentforce3);
}

void getJointAngles(const char* configName, double* jointAngles,
                    double* gimbalAngles) {
    Device* d = find(configName);
    if (!d) return;
    State s = d->state.load();
    // [A] Joint and gimbal angles are in the device's own joint space, not
    //     Cartesian, so no handedness flip is applied. But HapticPlugin.cs
    //     documents sign conventions ("Turret Left +, Thigh Up +, Shin Up +";
    //     gimbal "Right is +, Up is -, CW is +") which may not match raw HD
    //     output. Compare against the Windows build before trusting these.
    if (jointAngles)  memcpy(jointAngles,  s.jointAngles,  3 * sizeof(double));
    if (gimbalAngles) memcpy(gimbalAngles, s.gimbalAngles, 3 * sizeof(double));
}

// setForce is likewise NOT normalised: it takes a force vector directly, with
// no magnitude argument, so the effect-family convention does not obviously
// apply. Unmeasured.
void setForce(const char* configName, double* lateral3, double* torque3) {
    TRACE("setForce %s lat=(%.4f,%.4f,%.4f)", configName?configName:"?", lateral3?lateral3[0]:0, lateral3?lateral3[1]:0, lateral3?lateral3[2]:0);
    Device* d = find(configName);
    if (!d) return;
    Params p = d->params.load();
    p.extraOn = (lateral3 != nullptr);
    if (lateral3) unityToDev3(lateral3, p.extraForce);
    if (torque3)  memcpy(p.extraTorque, torque3, 3 * sizeof(double));
    d->params.store(p);
}

void setAnchorPosition(const char* configName, double* position3) {
    TRACE("setAnchorPosition %s (%.2f,%.2f,%.2f)", configName?configName:"?", position3?position3[0]:0, position3?position3[1]:0, position3?position3[2]:0);
    Device* d = find(configName);
    if (!d || !position3) return;
    Params p = d->params.load();
    // [V] This is the virtual stylus / proxy anchor, NOT the spring anchor.
    //     They are different things and the C# sets both every frame, in that
    //     order. An earlier draft wrote springAnchor here, so setAnchorPosition
    //     silently overwrote whatever setSpringValues had just configured and
    //     each device was sprung toward roughly its own position instead of its
    //     partner's. Stored separately now; nothing reads it yet, because no
    //     proxy rendering is implemented.
    unityToDev3(position3, p.proxyAnchor);
    d->params.store(p);
}

void setSpringValues(const char* configName, double* anchor, double magnitude) {
    TRACE("setSpringValues %s anchor=%s(%.2f,%.2f,%.2f) mag=%.6f", configName?configName:"?", anchor?"":"NULL", anchor?anchor[0]:0, anchor?anchor[1]:0, anchor?anchor[2]:0, magnitude);
    Device* d = find(configName);
    if (!d) return;
    Params p = d->params.load();
    if (anchor) unityToDev3(anchor, p.springAnchor);
    p.springGain = magnitude;      // see SPRING GAIN in effectCB
    p.springOn   = (magnitude != 0.0);
    d->params.store(p);
}

void setConstantForceValues(const char* configName, double* direction,
                            double magnitude) {
    TRACE("setConstantForceValues %s mag=%.6f", configName?configName:"?", magnitude);
    Device* d = find(configName);
    if (!d) return;
    Params p = d->params.load();
    if (direction) unityToDev3(direction, p.constDir);
    p.constMag = magnitude;
    p.constOn  = (magnitude != 0.0);
    d->params.store(p);
}

void setVibrationValues(const char* configName, double* direction3,
                        double magnitude, double frequency, double time) {
    TRACE("setVibrationValues %s mag=%.4f freq=%.4f time=%.4f", configName?configName:"?", magnitude, frequency, time);
    Device* d = find(configName);
    if (!d) return;
    Params p = d->params.load();
    if (direction3) unityToDev3(direction3, p.vibDir);
    p.vibMag       = magnitude;
    p.vibFreq      = frequency;
    p.vibRemaining = time;        // [A] duration, not timestamp
    p.vibOn        = (magnitude != 0.0 && time > 0.0);
    d->params.store(p);
}

void setGravityForce(const char* configName, double* gForce3) {
    TRACE("setGravityForce %s (%.4f,%.4f,%.4f)", configName?configName:"?", gForce3?gForce3[0]:0, gForce3?gForce3[1]:0, gForce3?gForce3[2]:0);
    Device* d = find(configName);
    if (!d || !gForce3) return;
    Params p = d->params.load();
    unityToDev3(gForce3, p.gravity);
    p.gravityOn = (p.gravity[0] || p.gravity[1] || p.gravity[2]);
    d->params.store(p);
}

// --- Force breakdown getters ----------------------------------------------
// [V] These have no HL or HD counterpart — HD reports only total commanded
//     force. The decomposition is the plugin's own bookkeeping, which is why
//     the accumulator above records each term separately as it adds it.
// Contact terms stay zero until addContactPointInfo is implemented.

void getCurrentFrictionForce(const char* configName, double* frictionForce) {
    Device* d = find(configName);
    if (!d || !frictionForce) return;
    State s = d->state.load();
    double sum[3];
    for (int i = 0; i < 3; ++i) sum[i] = s.fFrictionDyn[i] + s.fFrictionSta[i];
    devToUnity3(sum, frictionForce);
}

void getGlobalForces(const char* configName, double* vibrationForce,
                     double* constantForce, double* springForce) {
    Device* d = find(configName);
    if (!d) return;
    State s = d->state.load();
    if (vibrationForce) devToUnity3(s.fVibration, vibrationForce);
    if (constantForce)  devToUnity3(s.fConstant,  constantForce);
    if (springForce)    devToUnity3(s.fSpring,    springForce);
}

void getLocalForces(const char* configName, double* stiffnessForce,
                    double* viscosityForce, double* dynamicFrictionForce,
                    double* staticFrictionForce, double* constantForce,
                    double* springForce) {
    Device* d = find(configName);
    if (!d) return;
    State s = d->state.load();
    if (stiffnessForce)       devToUnity3(s.fStiffness,   stiffnessForce);
    if (viscosityForce)       devToUnity3(s.fViscosity,   viscosityForce);
    if (dynamicFrictionForce) devToUnity3(s.fFrictionDyn, dynamicFrictionForce);
    if (staticFrictionForce)  devToUnity3(s.fFrictionSta, staticFrictionForce);
    if (constantForce)        devToUnity3(s.fConstant,    constantForce);
    if (springForce)          devToUnity3(s.fSpring,      springForce);
}

// --- Contact point path — PARTIAL ------------------------------------------
// Stiffness and damping are rendered (see effectCB). Friction and pop-through
// are not. Logging is retained but off by default: set HAPTICSDIRECT_LOG=1 to
// dump every contact, which tells you which of the 23 arguments your scenes
// actually set to something other than zero. That is the list of what still
// needs implementing, and it is likely far shorter than the signature.
//
// [A] reset -> add(s) -> update is read as clear / stage / commit. The naming
//     fits and it mirrors the begin/accumulate/end priority structure in
//     HapticDeviceManager.cpp, but it is inference. If Unity calls update
//     without a preceding reset each frame, contacts will accumulate stale
//     entries and hit kMaxContacts — the log will show it immediately.

static void logOpen() { }
#define gLog trace()

void addContactPointInfo(const char* configName,
                         double* Location, double* Normal,
                         float MatStiffness, float MatDamping,
                         double* MatForce, float MatViscosity,
                         float MatFrictionStatic, float MatFrictionDynamic,
                         double* MatConstForceDir, float MatConstForceMag,
                         double* MatSpringDir, float MatSpringMag,
                         float MatPopThroughRel, float MatPopThroughAbs,
                         double MatMass, double RigBSpeed,
                         double* RigBVelocity, double* RigBAngularVelocity,
                         double RigBMass, double* ColImpulse,
                         double PhxDeltaTime, double ImpulseDepth) {
    // [V] These five are `double` in HapticPlugin.cs, not `float`. An earlier
    //     draft declared them float; float and double occupy the same argument
    //     slot on x86-64, so nothing crashed — the bits were just reinterpreted,
    //     which is why PhxDeltaTime read as -3.7e19 while MatStiffness (a real
    //     float) read correctly.
    logOpen();
    if (gLog) {
        fprintf(gLog, "addContact dev=%s loc=(%.2f,%.2f,%.2f) n=(%.2f,%.2f,%.2f) "
                      "k=%.3f b=%.3f visc=%.3f fs=%.3f fd=%.3f "
                      "popRel=%.3f popAbs=%.3f mass=%.3f rbMass=%.3f "
                      "dt=%.4f impDepth=%.3f\n",
                configName ? configName : "?",
                Location ? Location[0] : 0, Location ? Location[1] : 0,
                Location ? Location[2] : 0,
                Normal ? Normal[0] : 0, Normal ? Normal[1] : 0,
                Normal ? Normal[2] : 0,
                MatStiffness, MatDamping, MatViscosity,
                MatFrictionStatic, MatFrictionDynamic,
                MatPopThroughRel, MatPopThroughAbs,
                MatMass, RigBMass, PhxDeltaTime, ImpulseDepth);
        fflush(gLog);
    }

    Device* d = find(configName);
    if (!d || d->stagingCount >= kMaxContacts) return;

    Contact& c = d->staging[d->stagingCount++];
    if (Location) unityToDev3(Location, c.location);
    if (Normal)   unityToDev3(Normal,   c.normal);
    c.depth       = ImpulseDepth;
    c.stiffness   = MatStiffness;
    c.damping     = MatDamping;
    c.viscosity   = MatViscosity;
    c.frictionSta = MatFrictionStatic;
    c.frictionDyn = MatFrictionDynamic;
    // Unused for now, deliberately not stored: MatForce, MatConstForceDir/Mag,
    // MatSpringDir/Mag, MatPopThrough*, MatMass, RigB*, ColImpulse,
    // PhxDeltaTime. Add them when the log shows your scenes set them.
    (void)MatForce; (void)MatConstForceDir; (void)MatConstForceMag;
    (void)MatSpringDir; (void)MatSpringMag; (void)MatPopThroughRel;
    (void)MatPopThroughAbs; (void)MatMass; (void)RigBSpeed; (void)RigBVelocity;
    (void)RigBAngularVelocity; (void)RigBMass; (void)ColImpulse;
    (void)PhxDeltaTime;
}

void updateContactPointInfo(const char* configName) {
    TRACE("updateContactPointInfo %s", configName?configName:"?");
    Device* d = find(configName);
    if (!d) return;
    Params p = d->params.load();
    p.contactCount = d->stagingCount;
    for (int i = 0; i < d->stagingCount; ++i) p.contacts[i] = d->staging[i];
    d->params.store(p);
}

void resetContactPointInfo(const char* configName) {
    TRACE("resetContactPointInfo %s", configName?configName:"?");
    Device* d = find(configName);
    if (!d) return;
    d->stagingCount = 0;
}

// --- Teardown --------------------------------------------------------------

void disconnectAllDevices() {
    std::lock_guard<std::mutex> lk(gDevicesMutex);
    if (gSchedulerRunning) { hdStopScheduler(); gSchedulerRunning = false; }
    if (gServoHandle) { hdUnschedule(gServoHandle); gServoHandle = 0; }
    gServoCount.store(0, std::memory_order_release);
    for (auto& kv : gDevices) {
        Device* d = kv.second;
        // Known Linux hazard: hdDisableDevice has been reported to segfault at
        // exit on Linux, reproducible in the stock AnchoredSpringForce example
        // (H3D forums). Inside Unity that presents as an editor crash when
        // leaving play mode. If you hit it, this line is where it happens.
        if (d->hHD != HD_INVALID_HANDLE) hdDisableDevice(d->hHD);
        delete d;
    }
    gDevices.clear();

}

int getHDError(char* Info, int len) {
    if (Info && len > 0) snprintf(Info, len, "%s", gLastErrorMsg);
    int e = gLastError.exchange(HD_SUCCESS);
    gLastErrorMsg[0] = '\0';
    return e;
}

} // extern "C"
