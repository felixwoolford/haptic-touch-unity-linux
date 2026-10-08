/*  hdtest.c — standalone harness for libHapticsDirect.so
 *
 *  Loads the plugin the same way Mono does (dlopen + dlsym by exact name),
 *  so a failure here is a real failure and not a Unity configuration problem.
 *
 *  Build:  gcc -O2 -o hdtest hdtest.c -ldl -lm
 *  Run:    ./hdtest ./libHapticsDirect.so <configName> [springGain]
 *
 *  Hold button 1 to anchor a spring at the current position and feel it.
 *  Ctrl-C to exit cleanly (this also exercises disconnectAllDevices, which is
 *  where hdDisableDevice would segfault if that Linux bug is still present).
 */

#define _GNU_SOURCE
#include <dlfcn.h>
#include <sys/select.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>

typedef void (*fn_getVersionString)(char*, int);
typedef int  (*fn_initDevice)(const char*);
typedef void (*fn_getDeviceSN)(const char*, char*, int);
typedef void (*fn_getDeviceModel)(const char*, char*, int);
typedef void (*fn_getDeviceMaxValues)(const char*, double*, double*, double*);
typedef void (*fn_startSchedulers)(void);
typedef void (*fn_getWorkspaceArea)(const char*, double*, double*);
typedef void (*fn_getPosition)(const char*, double*);
typedef void (*fn_getVelocity)(const char*, double*);
typedef void (*fn_getButtons)(const char*, int*, int*, int*);
typedef void (*fn_getCurrentForce)(const char*, double*);
typedef void (*fn_setSpringValues)(const char*, double*, double);
typedef void (*fn_getGlobalForces)(const char*, double*, double*, double*);
typedef void (*fn_disconnectAllDevices)(void);
typedef int  (*fn_getHDError)(char*, int);

static fn_getVersionString      p_getVersionString;
static fn_initDevice            p_initDevice;
static fn_getDeviceSN           p_getDeviceSN;
static fn_getDeviceModel        p_getDeviceModel;
static fn_getDeviceMaxValues    p_getDeviceMaxValues;
static fn_startSchedulers       p_startSchedulers;
static fn_getWorkspaceArea      p_getWorkspaceArea;
static fn_getPosition           p_getPosition;
static fn_getVelocity           p_getVelocity;
static fn_getButtons            p_getButtons;
static fn_getCurrentForce       p_getCurrentForce;
static fn_setSpringValues       p_setSpringValues;
static fn_getGlobalForces       p_getGlobalForces;
static fn_disconnectAllDevices  p_disconnectAllDevices;
static fn_getHDError            p_getHDError;

static void*        gLib   = NULL;
static const char*  gCfg   = NULL;
static volatile sig_atomic_t gRun = 1;

static void onSigint(int s) { (void)s; gRun = 0; }

static void* need(const char* name) {
    void* p = dlsym(gLib, name);
    if (!p) fprintf(stderr, "  MISSING: %s\n", name);
    return p;
}

static void checkErr(const char* where) {
    char buf[256] = {0};
    int e = p_getHDError(buf, sizeof(buf));
    if (e != 0) printf("  [hd error @%s] %d: %s\n", where, e, buf);
}

int main(int argc, char** argv) {
    if (argc < 3) {
        fprintf(stderr, "usage: %s <libHapticsDirect.so> <configName> [gain]\n",
                argv[0]);
        return 2;
    }
    const char* libpath = argv[1];
    gCfg = argv[2];
    double gain = (argc > 3) ? atof(argv[3]) : 0.1;   /* N/mm, examples use .1-.25 */

    gLib = dlopen(libpath, RTLD_NOW | RTLD_LOCAL);
    if (!gLib) { fprintf(stderr, "dlopen: %s\n", dlerror()); return 1; }
    printf("dlopen ok: %s\n", libpath);

    printf("resolving symbols...\n");
    int bad = 0;
    #define R(x) do { p_##x = (fn_##x)need(#x); if (!p_##x) bad = 1; } while (0)
    R(getVersionString); R(initDevice); R(getDeviceSN); R(getDeviceModel);
    R(getDeviceMaxValues); R(startSchedulers); R(getWorkspaceArea);
    R(getPosition); R(getVelocity); R(getButtons); R(getCurrentForce);
    R(setSpringValues); R(getGlobalForces); R(disconnectAllDevices);
    R(getHDError);
    #undef R
    if (bad) { fprintf(stderr, "symbol resolution failed\n"); return 1; }
    printf("  all resolved\n\n");

    char ver[128] = {0};
    p_getVersionString(ver, sizeof(ver));
    printf("OpenHaptics version: %s\n", ver);

    printf("initDevice(\"%s\")...\n", gCfg);
    int h = p_initDevice(gCfg);
    printf("  handle = %d (0x%x)\n", h, (unsigned)h);
    checkErr("initDevice");
    if ((unsigned)h == 0xFFFFFFFFu) {
        fprintf(stderr,
            "\ninitDevice failed. Before suspecting the plugin, check:\n"
            "  - the config name matches Touch_Setup exactly (case-sensitive)\n"
            "  - GTDD_HOME / ~/.3dsystems/config is readable by this user\n"
            "  - LC_NUMERIC is not a comma-decimal locale\n");
        return 1;
    }

    char sn[128] = {0}, model[128] = {0};
    p_getDeviceSN(gCfg, sn, sizeof(sn));
    p_getDeviceModel(gCfg, model, sizeof(model));
    printf("  serial = '%s'  model = '%s'\n", sn, model);

    double ks = 0, kd = 0, fmax = 0;
    p_getDeviceMaxValues(gCfg, &ks, &kd, &fmax);
    printf("  max stiffness = %.4f N/mm   max damping = %.5f   max force = %.3f N\n",
           ks, kd, fmax);

    double usable[6] = {0}, maxws[6] = {0};
    p_getWorkspaceArea(gCfg, usable, maxws);
    printf("  usable workspace = [%.1f %.1f %.1f] .. [%.1f %.1f %.1f] mm\n",
           usable[0], usable[1], usable[2], usable[3], usable[4], usable[5]);

    printf("\nstartSchedulers()...\n");
    p_startSchedulers();
    checkErr("startSchedulers");

    signal(SIGINT, onSigint);

    /* Buttons are not used to drive the spring — anchor on a countdown instead,
       so a dead button can't block the measurement. Press Enter to re-anchor
       at the current position. */
    printf("\nHold the stylus still. Anchoring in");
    fflush(stdout);
    {
        int c;
        struct timespec one = { 1, 0 };
        for (c = 3; c > 0; --c) { printf(" %d", c); fflush(stdout); nanosleep(&one, NULL); }
    }
    printf(" — anchored.\n");

    double anchor[3] = {0, 0, 0};
    p_getPosition(gCfg, anchor);
    p_setSpringValues(gCfg, anchor, gain);
    printf("Spring at gain %.4f N/mm. Pull away and watch k~.\n", gain);
    printf("Enter re-anchors, Ctrl-C quits.\n\n");

    struct timespec ts = { 0, 50 * 1000 * 1000 };   /* 20 Hz print rate */

    while (gRun) {
        double pos[3] = {0}, vel[3] = {0}, force[3] = {0};
        int btn[4] = {0}, last[4] = {0}, ink = 0;

        p_getPosition(gCfg, pos);
        p_getVelocity(gCfg, vel);
        p_getButtons(gCfg, btn, last, &ink);
        p_getCurrentForce(gCfg, force);

        /* Non-blocking check for Enter. */
        {
            fd_set rfds;
            struct timeval tv = { 0, 0 };
            FD_ZERO(&rfds);
            FD_SET(0, &rfds);
            if (select(1, &rfds, NULL, NULL, &tv) > 0) {
                char line[64];
                if (fgets(line, sizeof(line), stdin)) {
                    memcpy(anchor, pos, sizeof(anchor));
                    p_setSpringValues(gCfg, anchor, gain);
                    printf("re-anchored at %.2f %.2f %.2f\n",
                           anchor[0], anchor[1], anchor[2]);
                }
            }
        }

        double vib[3] = {0}, con[3] = {0}, spr[3] = {0};
        p_getGlobalForces(gCfg, vib, con, spr);

        /* Displacement along the spring, and the gain it implies. This is the
           measurement that settles what `magnitude` means: if impliedGain
           tracks the gain you passed in, it is N/mm and the reading is right. */
        double d[3], dist = 0, sprMag = 0;
        int i;
        for (i = 0; i < 3; ++i) d[i] = anchor[i] - pos[i];
        dist   = sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2]);
        sprMag = sqrt(spr[0]*spr[0] + spr[1]*spr[1] + spr[2]*spr[2]);

        printf("\rpos %7.2f %7.2f %7.2f | vel %7.1f %7.1f %7.1f | "
               "btn %d%d%d%d ink %d | F %6.3f %6.3f %6.3f | "
               "spr %5.3fN d %6.2fmm k~%.4f   ",
               pos[0], pos[1], pos[2], vel[0], vel[1], vel[2],
               btn[0], btn[1], btn[2], btn[3], ink,
               force[0], force[1], force[2],
               sprMag, dist,
               (dist > 1e-6) ? sprMag / dist : 0.0);
        fflush(stdout);

        nanosleep(&ts, NULL);
    }

    printf("\n\ndisconnectAllDevices()...\n");
    p_disconnectAllDevices();
    printf("  returned cleanly\n");
    dlclose(gLib);
    return 0;
}
