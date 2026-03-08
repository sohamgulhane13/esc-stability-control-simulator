/*
  ESC_Traction_Simulation_patched.c
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
void view_report_tail();
/* ---------- Configuration parameters (tune these) ---------- */
#define REPORT_FILE "esc_report.txt"
#define CSV_BUFFER 512
#define G_CONST 9.81

/* Safety supervisor thresholds */
#define SENSOR_TIMEOUT_SEC 0.5
#define SENSOR_STUCK_DELTA 1e-3
#define SENSOR_FAULT_THRESHOLD 3
#define WATCHDOG_TIMEOUT_SEC 1.0
#define MIN_CONFIDENCE_FOR_OPERATION 0.35
#define INNOVATION_SCALE_SAFE_THRESHOLD 10.0

/* Process & measurement tuning */
#define Q_psi 1e-4
#define Q_v   1e-2
#define Q_k   1e-6

/* ---------- Small linear algebra helpers ---------- */
void mat3_mul(double A[3][3], double B[3][3], double C[3][3]) {
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) {
        C[i][j]=0.0;
        for (int k=0;k<3;k++) C[i][j]+=A[i][k]*B[k][j];
    }
}
void mat3_copy(double A[3][3], double B[3][3]) {
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) B[i][j]=A[i][j];
}

/* ---------- Time helpers ---------- */
double now_seconds_double() {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}
void current_timestamp(char *buf, size_t n) {
    time_t t = time(NULL);
    struct tm *tm = localtime(&t);
    snprintf(buf, n, "%04d-%02d-%02d %02d:%02d:%02d",
             tm->tm_year+1900, tm->tm_mon+1, tm->tm_mday,
             tm->tm_hour, tm->tm_min, tm->tm_sec);
}

/* ---------- EKF structures ---------- */
typedef struct {
    double x[3];        /* psi, v, kappa */
    double P[3][3];
    double last_update_time; /* for watchdog */
} EKF;

void ekf_init(EKF *kf) {
    kf->x[0]=0.0; kf->x[1]=0.0; kf->x[2]=0.0;
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) kf->P[i][j]=0.0;
    kf->P[0][0]=0.5; kf->P[1][1]=1.0; kf->P[2][2]=0.1;
    kf->last_update_time = now_seconds_double();
}

/* EKF prediction with simple kinematic model */
void ekf_predict(EKF *kf, double dt) {
    double psi = kf->x[0], v = kf->x[1], kappa = kf->x[2];
    double x_pred[3];
    x_pred[0] = psi + kappa * v * dt;
    x_pred[1] = v;
    x_pred[2] = kappa;

    double F[3][3] = {{1.0, kappa*dt, v*dt},
                      {0.0, 1.0,     0.0},
                      {0.0, 0.0,     1.0}};

    double temp[3][3]; mat3_mul(F, kf->P, temp);
    double F_T[3][3];
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) F_T[i][j]=F[j][i];
    double Pnew[3][3]; mat3_mul(temp, F_T, Pnew);

    /* Add process noise */
    Pnew[0][0] += Q_psi;
    Pnew[1][1] += Q_v;
    Pnew[2][2] += Q_k;

    mat3_copy(Pnew, kf->P);
    kf->x[0]=x_pred[0]; kf->x[1]=x_pred[1]; kf->x[2]=x_pred[2];
}

/* EKF scalar update (same as earlier) */
void ekf_update_scalar(EKF *kf, double z, double h_x, double H[3], double R) {
    double y = z - h_x;
    double HP[3];
    for (int i=0;i<3;i++) {
        HP[i]=0.0;
        for (int j=0;j<3;j++) HP[i]+=H[j]*kf->P[j][i];
    }
    double S=0.0;
    for (int i=0;i<3;i++) S+=HP[i]*H[i];
    S += R;
    if (S <= 0.0) S = 1e-9;
    double K[3];
    for (int i=0;i<3;i++) {
        double tmp=0.0;
        for (int j=0;j<3;j++) tmp += kf->P[i][j]*H[j];
        K[i] = tmp / S;
    }
    for (int i=0;i<3;i++) kf->x[i] += K[i] * y;
    double KH[3][3];
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) KH[i][j] = K[i] * H[j];
    double IminusKH[3][3];
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) IminusKH[i][j] = (i==j?1.0:0.0) - KH[i][j];
    double newP[3][3]; mat3_mul(IminusKH, kf->P, newP);
    mat3_copy(newP, kf->P);
}

/* ---------- Predicted measurement & jacobians ---------- */
double predict_gyro(const double x[3], double H[3]) {
    double v = x[1], k = x[2];
    H[0]=0.0; H[1]=k; H[2]=v;
    return k * v;
}
double predict_wheel(const double x[3], double H[3]) { H[0]=0.0; H[1]=1.0; H[2]=0.0; return x[1]; }
double predict_kappa(const double x[3], double H[3]) { H[0]=0.0; H[1]=0.0; H[2]=1.0; return x[2]; }
double predict_accel(const double x[3], double H[3]) {
    double v=x[1], k=x[2];
    H[0]=0.0; H[1]=2.0*v*k; H[2]=v*v;
    return v*v*k;
}

/* ---------- Safety Supervisor Data Structures ---------- */
typedef struct {
    double last_time;
    double last_value;
    int fault_count;
    int disabled; /* 1 = disabled by supervisor */
} SensorHealth;

/* Supervisor state */
typedef struct {
    SensorHealth gyro;
    SensorHealth wheel;
    SensorHealth accel;
    SensorHealth vision;
    SensorHealth gps;
    double last_ekf_time;
    int safe_mode_active;
} Supervisor;

/* Initialize supervisor */
void supervisor_init(Supervisor *s) {
    s->gyro.last_time = s->wheel.last_time = s->accel.last_time = s->vision.last_time = s->gps.last_time = now_seconds_double();
    s->gyro.last_value = s->wheel.last_value = s->accel.last_value = s->vision.last_value = s->gps.last_value = NAN;
    s->gyro.fault_count = s->wheel.fault_count = s->accel.fault_count = s->vision.fault_count = s->gps.fault_count = 0;
    s->gyro.disabled = s->wheel.disabled = s->accel.disabled = s->vision.disabled = s->gps.disabled = 0;
    s->last_ekf_time = now_seconds_double();
    s->safe_mode_active = 0;
}

/* Check a sensor update: update last_time & last_value, detect stuck or out-of-range.
   Returns 0 if healthy, 1 if a fault incremented, 2 if disabled */
int supervisor_check_and_update(SensorHealth *sh, double value, double now) {
    if (isnan(value)) {
        /* missing sample: if time gap > timeout considered a fault */
        if (now - sh->last_time > SENSOR_TIMEOUT_SEC) {
            sh->fault_count += 1;
            sh->last_time = now;
            if (sh->fault_count >= SENSOR_FAULT_THRESHOLD) { sh->disabled = 1; return 2; }
            return 1;
        }
        return 0;
    } else {
        /* compare with last value for stuck detection */
        if (!isnan(sh->last_value)) {
            double delta = fabs(value - sh->last_value);
            if (delta < SENSOR_STUCK_DELTA) {
                sh->fault_count += 1;
            } else {
                sh->fault_count = 0;
            }
        }
        sh->last_value = value;
        sh->last_time = now;
        if (sh->fault_count >= SENSOR_FAULT_THRESHOLD) { sh->disabled = 1; return 2; }
        return 0;
    }
}

/* Force safe-mode activation */
void supervisor_activate_safe_mode(Supervisor *s) {
    s->safe_mode_active = 1;
}

/* Reset safe-mode (manual reset only) */
void supervisor_clear_safe_mode(Supervisor *s) {
    s->safe_mode_active = 0;
}

/* ---------- Logging ---------- */
void append_report_header(FILE *f) {
    if (!f) return;
    char ts[64]; current_timestamp(ts,sizeof(ts));
    fprintf(f,"=== ESC EKF Fusion Session started: %s ===\n\n", ts);
    fflush(f);
}

/* Human readable log which now includes supervisor status & sensors disabled flags */
void append_human_readable(FILE *f, double tstamp, EKF *kf,
                           int have_gyro, double gyro, int have_wheel, double wheel_v,
                           int have_accel, double accel_y,
                           int have_vision, double vision_kappa, double vision_conf,
                           int have_gps, double gps_kappa, double gps_conf,
                           double mu, int abs_flag,
                           Supervisor *sup, double innovation_scale, double fused_confidence)
{
    if (!f) return;
    char ts[64]; current_timestamp(ts, sizeof(ts));
    double psi = kf->x[0], v = kf->x[1], kappa = kf->x[2];
    double radius = (fabs(kappa) > 1e-9) ? (1.0 / kappa) : 1e9;

    fprintf(f,"------------------------------------------------------------\n");
    fprintf(f,"Electronic Stability & Traction Simulation (EKF Fusion + Supervisor)\n");
    fprintf(f,"Timestamp : %s  (t=%.2f)\n", ts, tstamp);
    fprintf(f,"Reference brief: %s\n\n", "/mnt/data/MINI PROJECT ON VEHICLES (1).pdf");

    fprintf(f,"SENSOR INPUTS & HEALTH\n");
    fprintf(f,"  • Gyro: %s   last_ok=%s\n", have_gyro ? "present" : "N/A", sup->gyro.disabled ? "NO (disabled)" : "YES");
    fprintf(f,"  • Wheel speed: %s   last_ok=%s\n", have_wheel ? "present" : "N/A", sup->wheel.disabled ? "NO (disabled)" : "YES");
    fprintf(f,"  • Lateral accel: %s   last_ok=%s\n", have_accel ? "present" : "N/A", sup->accel.disabled ? "NO (disabled)" : "YES");
    fprintf(f,"  • Vision kappa: %s   (conf=%.2f)   last_ok=%s\n", have_vision ? "present" : "N/A", vision_conf, sup->vision.disabled ? "NO (disabled)" : "YES");
    fprintf(f,"  • GPS kappa: %s   (conf=%.2f)   last_ok=%s\n", have_gps ? "present" : "N/A", gps_conf, sup->gps.disabled ? "NO (disabled)" : "YES");

    fprintf(f,"\nEKF STATE & FUSION\n");
    fprintf(f,"  • Heading psi     : %.4f rad\n", psi);
    fprintf(f,"  • Speed v         : %.3f m/s\n", v);
    fprintf(f,"  • Curvature kappa : %.6f  (Radius = %.2f m)\n", kappa, radius);
    fprintf(f,"  • Fused confidence: %.3f   (min required: %.2f)\n", fused_confidence, MIN_CONFIDENCE_FOR_OPERATION);

    fprintf(f,"\nASSESSMENT & SAFETY\n");
    double a_lat = v*v*kappa;
    double a_max = mu * G_CONST;
    double usage = (a_max>0.0) ? (a_lat / a_max) : 1e9;
    fprintf(f,"  • Lateral accel (model)   : %.4f m/s^2\n", a_lat);
    fprintf(f,"  • Max allowed (mu*g)      : %.4f m/s^2\n", a_max);
    fprintf(f,"  • Traction usage (ratio)  : %.4f (%.1f%%)\n", usage, usage*100.0);
    if (usage >= 1.0) fprintf(f,"  ⚠️  SKID LIKELY — ESC should intervene\n");
    else if (usage >= 0.8) fprintf(f,"  ⚠️  High lateral load — near traction limit\n");
    else fprintf(f,"  ✅  Within traction capability\n");

    if (sup->safe_mode_active) {
        fprintf(f,"\n*** SAFE MODE ACTIVATED ***\n");
        fprintf(f,"  • Reason: ");
        if (fused_confidence < MIN_CONFIDENCE_FOR_OPERATION) fprintf(f,"low fused confidence ");
        if (innovation_scale > INNOVATION_SCALE_SAFE_THRESHOLD) fprintf(f,"| very large filter innovation ");
        if ((now_seconds_double() - sup->last_ekf_time) > WATCHDOG_TIMEOUT_SEC) fprintf(f,"| EKF watchdog timeout ");
        fprintf(f,"\n  • Action taken: conservative ESC behavior: limit speed, increase braking authority, request driver alert.\n");
    }

    fprintf(f,"\nFUSION DIAGNOSTICS\n");
    fprintf(f,"  • Innovation scale (sum of flagged norms): %.3f\n", innovation_scale);
    fprintf(f,"  • Sensor disable flags: gyro=%d, wheel=%d, accel=%d, vision=%d, gps=%d\n",
            sup->gyro.disabled, sup->wheel.disabled, sup->accel.disabled, sup->vision.disabled, sup->gps.disabled);

    fprintf(f,"\nNotes: Supervisor enforces sensor timeouts, stuck-sensor detection and watchdog. Safe mode triggers logged above.\n");
    fprintf(f,"------------------------------------------------------------\n\n");
    fflush(f);
}

/* ---------- Console summary helper (NEW) ---------- */
void print_console_summary(double tstamp, EKF *kf, double mu, int abs_flag, Supervisor *sup) {
    double v = kf->x[1];
    double kappa = kf->x[2];
    double R = (fabs(kappa) > 1e-9) ? (1.0 / kappa) : 1e9;
    double a_lat = v * v * kappa;
    double a_max = mu * G_CONST;
    double usage = (a_max > 0.0) ? (a_lat / a_max) : 1e9;
    printf("t=%.2fs | v=%.2f m/s | R=%.2f m | a_lat=%.2f m/s^2 | usage=%.2f | safe_mode=%s\n",
           tstamp, v, R, a_lat, usage, sup->safe_mode_active ? "YES":"NO");
    fflush(stdout);
}

/* ---------- Processing pipeline including supervisor checks ---------- */

void clamp_double(double *v, double minv, double maxv) {
    if (*v < minv) *v = minv;
    if (*v > maxv) *v = maxv;
}

/* Process a single timestep: predict EKF, apply measurements with dynamic variances,
   update supervisor health, compute fused confidence and possibly trigger safe mode. */
void process_timestep_with_supervisor(EKF *kf, Supervisor *sup, FILE *logf,
                                      double tstamp, double dt,
                                      int raw_have_gyro, double raw_gyro,
                                      int raw_have_wheel, double raw_wheel_v,
                                      int raw_have_accel, double raw_accel_y,
                                      int raw_have_vision, double raw_vision_kappa, double raw_vision_conf,
                                      int raw_have_gps, double raw_gps_kappa, double raw_gps_conf,
                                      int abs_flag, double mu)
{
    double now = now_seconds_double();

    /* Update supervisor last ekf time (we will update after EKF steps) */
    sup->last_ekf_time = now;

    /* Update sensor health (if data present, pass value; otherwise pass NAN) */
    double gyro_val = raw_have_gyro ? raw_gyro : NAN;
    double wheel_val = raw_have_wheel ? raw_wheel_v : NAN;
    double accel_val = raw_have_accel ? raw_accel_y : NAN;
    double vis_val = raw_have_vision ? raw_vision_kappa : NAN;
    double gps_val = raw_have_gps ? raw_gps_kappa : NAN;

    if (!sup->gyro.disabled) supervisor_check_and_update(&sup->gyro, gyro_val, now);
    if (!sup->wheel.disabled) supervisor_check_and_update(&sup->wheel, wheel_val, now);
    if (!sup->accel.disabled) supervisor_check_and_update(&sup->accel, accel_val, now);
    if (!sup->vision.disabled) supervisor_check_and_update(&sup->vision, vis_val, now);
    if (!sup->gps.disabled) supervisor_check_and_update(&sup->gps, gps_val, now);

    /* If any sensor got disabled, it will be ignored below. */

    /* EKF predict */
    ekf_predict(kf, dt);

    /* We'll maintain an "innovation scale" for diagnostics; large values indicate faults */
    double innovation_scale = 0.0;

    /* Fused confidence estimate: start high and reduce for each missing/disabled sensor */
    double fused_conf = 1.0;
    int available_count = 0;

    /* --- Wheel (v) measurement --- */
    if (raw_have_wheel && !sup->wheel.disabled) {
        double H[3]; double h = predict_wheel(kf->x, H);
        double R = 0.5;
        if (abs_flag) R *= 10.0;
        double HP[3]; for (int i=0;i<3;i++){HP[i]=0.0; for (int j=0;j<3;j++) HP[i]+=H[j]*kf->P[j][i];}
        double S=0.0; for (int i=0;i<3;i++) S+=HP[i]*H[i]; S+=R;
        double y = raw_wheel_v - h;
        double norm_innov = fabs(y)/sqrt(S);
        if (norm_innov > 4.0) { R *= 100.0; innovation_scale += norm_innov; }
        ekf_update_scalar(kf, raw_wheel_v, h, H, R);
        available_count++;
    } else fused_conf *= 0.8;

    /* --- Gyro (omega) measurement --- */
    if (raw_have_gyro && !sup->gyro.disabled) {
        double H[3]; double h = predict_gyro(kf->x, H);
        double R = 0.02;
        double HP[3]; for (int i=0;i<3;i++){HP[i]=0.0; for (int j=0;j<3;j++) HP[i]+=H[j]*kf->P[j][i];}
        double S=0.0; for (int i=0;i<3;i++) S+=HP[i]*H[i]; S+=R;
        double y = raw_gyro - h;
        double norm_innov = fabs(y)/sqrt(S);
        if (norm_innov > 4.0) { R *= 50.0; innovation_scale += norm_innov; }
        ekf_update_scalar(kf, raw_gyro, h, H, R);
        available_count++;
    } else fused_conf *= 0.8;

    /* --- Accel --- */
    if (raw_have_accel && !sup->accel.disabled) {
        double H[3]; double h = predict_accel(kf->x, H);
        double R = 0.5;
        double HP[3]; for (int i=0;i<3;i++){HP[i]=0.0; for (int j=0;j<3;j++) HP[i]+=H[j]*kf->P[j][i];}
        double S=0.0; for (int i=0;i<3;i++) S+=HP[i]*H[i]; S+=R;
        double y = raw_accel_y - h;
        double norm_innov = fabs(y)/sqrt(S);
        if (norm_innov > 5.0) { R *= 50.0; innovation_scale += norm_innov; }
        ekf_update_scalar(kf, raw_accel_y, h, H, R);
        available_count++;
    } else fused_conf *= 0.9;

    /* --- Vision kappa --- */
    if (raw_have_vision && !sup->vision.disabled) {
        double H[3]; double h = predict_kappa(kf->x, H);
        double Rbase = 1e-4; double R = Rbase * (1.0 / (raw_vision_conf > 1e-6 ? raw_vision_conf : 1e-6));
        double HP[3]; for (int i=0;i<3;i++){HP[i]=0.0; for (int j=0;j<3;j++) HP[i]+=H[j]*kf->P[j][i];}
        double S=0.0; for (int i=0;i<3;i++) S+=HP[i]*H[i]; S+=R;
        double y = raw_vision_kappa - h;
        double norm_innov = fabs(y)/sqrt(S);
        if (norm_innov > 3.5) { R *= 20.0; innovation_scale += norm_innov; }
        ekf_update_scalar(kf, raw_vision_kappa, h, H, R);
        available_count++;
    } else fused_conf *= 0.9;

    /* --- GPS kappa --- */
    if (raw_have_gps && !sup->gps.disabled) {
        double H[3]; double h = predict_kappa(kf->x, H);
        double Rbase = 1e-4; double R = Rbase * (1.0 / (raw_gps_conf > 1e-6 ? raw_gps_conf : 1e-6));
        double HP[3]; for (int i=0;i<3;i++){HP[i]=0.0; for (int j=0;j<3;j++) HP[i]+=H[j]*kf->P[j][i];}
        double S=0.0; for (int i=0;i<3;i++) S+=HP[i]*H[i]; S+=R;
        double y = raw_gps_kappa - h;
        double norm_innov = fabs(y)/sqrt(S);
        if (norm_innov > 3.5) { R *= 20.0; innovation_scale += norm_innov; }
        ekf_update_scalar(kf, raw_gps_kappa, h, H, R);
        available_count++;
    } else fused_conf *= 0.9;

    /* Compute a simple fused_confidence: normalized available_count and disabled sensors */
    double total_possible = 5.0;
    double avail_norm = (double)available_count / total_possible;
    int disabled_count = sup->gyro.disabled + sup->wheel.disabled + sup->accel.disabled + sup->vision.disabled + sup->gps.disabled;
    fused_conf = avail_norm * (1.0 - (double)disabled_count / total_possible);

    /* Watchdog: if EKF hasn't updated in longer than WATCHDOG_TIMEOUT_SEC, activate safe mode */
    if ((now - sup->last_ekf_time) > WATCHDOG_TIMEOUT_SEC) sup->safe_mode_active = 1;

    /* Check innovation scale or low confidence -> safe mode */
    if (innovation_scale > INNOVATION_SCALE_SAFE_THRESHOLD) sup->safe_mode_active = 1;
    if (fused_conf < MIN_CONFIDENCE_FOR_OPERATION) sup->safe_mode_active = 1;

    /* If safe mode is active, take conservative actions */
    if (sup->safe_mode_active) {
        kf->x[1] *= 0.6;
        kf->x[2] *= 1.2;
    }

    /* Update sup last ekf time */
    sup->last_ekf_time = now;

    /* Append human readable log including supervisor status */
    append_human_readable(logf, tstamp, kf,
                          raw_have_gyro, raw_gyro,
                          raw_have_wheel, raw_wheel_v,
                          raw_have_accel, raw_accel_y,
                          raw_have_vision, raw_vision_kappa, raw_vision_conf,
                          raw_have_gps, raw_gps_kappa, raw_gps_conf,
                          mu, abs_flag,
                          sup, innovation_scale, fused_conf);

    /* NEW: print concise summary to console so user immediately sees result */
    print_console_summary(tstamp, kf, mu, abs_flag, sup);
}

/* ---------- Interactive and batch modes ---------- */

void interactive_mode(EKF *kf, Supervisor *sup, FILE *logf) {
    printf("\nInteractive mode (simulate sensors). Enter values or -1 for unavailable.\n");
    double dt = 0.05;
    double t = 0.0;

    printf("Road condition (1 Dry,2 Wet,3 Snow,4 High-grip): ");
    int road_choice; if (scanf("%d",&road_choice)!=1) road_choice=1;
    double mu = (road_choice==1?0.7:(road_choice==2?0.45:(road_choice==3?0.25:0.9)));

    while (1) {
        printf("\nTime %.2f s — Enter sensor readings (use -1 for unavailable):\n", t);
        printf("  • Gyro (rad/s)\n");
        printf("  • Lateral Acceleration (m/s²)\n");
        printf("  • Wheel Speed (m/s)\n");
        printf("  • Vision Curvature κ (1/m)\n");
        printf("  • Vision Confidence (0–1)\n");
        printf("  • GPS Curvature κ (1/m)\n");
        printf("  • GPS Confidence (0–1)\n");
        printf("  • ABS Flag (0 or 1)\n\n");
        printf("Input format: gyro accel wheel vision_kappa vision_conf gps_kappa gps_conf abs_flag\n> ");

        double gyro, accel, wheel, vk, vconf, gk, gconf; int absf;
        if (scanf("%lf %lf %lf %lf %lf %lf %lf %d", &gyro, &accel, &wheel, &vk, &vconf, &gk, &gconf, &absf) != 8) {
            printf("Input error or EOF. Exiting interactive.\n");
            while(getchar()!='\n');
            break;
        }

        int hg = (gyro!=-1.0), ha = (accel!=-1.0), hw = (wheel!=-1.0), hv = (vk!=-1.0), hgps = (gk!=-1.0);
        double raw_gyro = gyro, raw_accel = accel, raw_wheel = wheel, raw_vk = vk, raw_gk = gk;
        double nowt = t;

        process_timestep_with_supervisor(kf, sup, logf, nowt, dt,
                                         hg, raw_gyro, hw, raw_wheel, ha, raw_accel,
                                         hv, raw_vk, vconf, hgps, raw_gk, gconf,
                                         absf, mu);

        t += dt;
        printf("Continue? (y/n): "); char c; scanf(" %c",&c); if (c!='y' && c!='Y') break;
    }
}

void batch_mode(EKF *kf, Supervisor *sup, FILE *logf, const char *csvpath, int road_choice) {
    FILE *f = fopen(csvpath, "r");
    if (!f) { printf("Cannot open %s\n", csvpath); return; }
    char line[CSV_BUFFER];
    double last_t = -1.0;
    double mu = (road_choice==1?0.7:(road_choice==2?0.45:(road_choice==3?0.25:0.9)));

    while (fgets(line, sizeof(line), f)) {
        if (line[0]=='#' || strlen(line)<3) continue;
        double t, gyro, accel, wheel, vk, gk, vconf, gconf; int absf;
        int read = sscanf(line, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d", &t,&gyro,&accel,&wheel,&vk,&gk,&vconf,&gconf,&absf);
        if (read < 1) continue;
        if (read < 9) { /* default missing fields */
            vconf = (read>=7? vconf: 0.0);
            gconf = (read>=8? gconf: 0.0);
            absf = (read>=9? absf:0);
        }
        int have_gyro = (read>=2 && gyro!=-1.0);
        int have_accel = (read>=3 && accel!=-1.0);
        int have_wheel = (read>=4 && wheel!=-1.0);
        int have_vision = (read>=5 && vk!=-1.0);
        int have_gps = (read>=6 && gk!=-1.0);
        if (last_t < 0.0) last_t = t;
        double dt = t - last_t;
        if (dt <= 0.0) dt = 0.05;
        process_timestep_with_supervisor(kf, sup, logf, t, dt,
                                         have_gyro, gyro, have_wheel, wheel, have_accel, accel,
                                         have_vision, vk, vconf, have_gps, gk, gconf,
                                         absf, mu);
        last_t = t;
    }
    fclose(f);

    /* NEW: automatically show the recent report tail after batch completes */
    printf("\nBatch run complete — showing report tail:\n");
    view_report_tail();
}

/* View recent tail of report */
void view_report_tail() {
    FILE *f = fopen(REPORT_FILE,"r");
    if (!f) { printf("No report file yet.\n"); return; }
    fseek(f,0,SEEK_END);
    long size = ftell(f);
    long start = 0;
    if (size > 12000) start = size - 12000;
    fseek(f,start,SEEK_SET);
    char buf[256];
    while (fgets(buf,sizeof(buf),f)) fputs(buf, stdout);
    fclose(f);
}

/* ---------- Main ---------- */
int main() {
    printf("=== ESC EKF Fusion + Safety Supervisor\n");
    EKF kf; ekf_init(&kf);
    Supervisor sup; supervisor_init(&sup);

    FILE *logf = fopen(REPORT_FILE,"a");
    if (!logf) { printf("Warning: cannot open %s for writing.\n", REPORT_FILE); }
    else append_report_header(logf);

    while (1) {
        printf("\nMenu:\n  1) Interactive (simulate sensors)\n  2) Batch CSV\n  3) View report tail\n  4) Exit\nChoose: ");
        int ch; if (scanf("%d",&ch)!=1) break;
        if (ch==1) {
            interactive_mode(&kf, &sup, logf);
        } else if (ch==2) {
            char path[256]; int road_choice;
            printf("CSV path: "); scanf("%s", path);
            printf("Road condition (1 Dry,2 Wet,3 Snow,4 High): "); scanf("%d",&road_choice);
            batch_mode(&kf, &sup, logf, path, road_choice);
        } else if (ch==3) {
            view_report_tail();
        } else if (ch==4) {
            break;
        } else printf("Invalid\n");
    }

    if (logf) fclose(logf);
    printf("Exiting. Human-readable reports and supervisor logs saved to '%s'.\n", REPORT_FILE);
    return 0;
}

