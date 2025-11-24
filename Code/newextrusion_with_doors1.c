/* newextrusion.c — C (MSVC) — floorplan to OBJ with rectangular door openings
   Units: 1 px = 1 inch. Wall height = 120", door opening = 48" wide × 84" tall.

   Build: cl /TC /O2 /W4 newextrusion.c
   Run  : newextrusion.exe input.pgm model.obj
*/

#define _CRT_SECURE_NO_WARNINGS

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* --- Parameters --- */
#define HEIGHT_PX 120 /* wall height (inches) */
#define OTSU_LEVELS 256

/* Door opening controls */
#define DOOR_W_PX 48     /* target door clear width (inches) */
#define DOOR_H_PX 84     /* door height to cut (inches) */
#define DOOR_DILATE_R 24 /* half-band for mask dilation: 24 px ≈ 48" total */

/* --- Basic image/point types --- */
typedef struct {
  int w, h;
  uint8_t* p;
} Gray;
typedef struct {
  int x, y;
} Px;

/* Detected door arc (plus points). We also compute a chord, but mask-based cut
 * is used. */
typedef struct {
  double cx, cy, r;
  double span_deg;
  int nin;
  Px* points;

  /* chord endpoints (not required for mask method, kept for completeness) */
  double ax, ay, bx, by;
  int chord_ready;
} DoorArc;

/* --- Utilities / forward decls --- */
static void die(const char* msg);
static Gray read_pgm(const char* path);
static int cclabel(const uint8_t* mask, int w, int h, int conn, int* labels);
static void flood_outside(const uint8_t* free_mask, int w, int h,
                          uint8_t* outside);
static bool circle_from3(int x1, int y1, int x2, int y2, int x3, int y3,
                         double* cx, double* cy, double* r);
static int compare_doubles(const void* a, const void* b);
static bool validate_arc_brightness(const Gray* img, const DoorArc* arc);
static int otsu(const Gray* img);
static void erode3x3(const uint8_t* in, uint8_t* out, int w, int h,
                     int iterations);
static void compute_arc_chord_from_points(DoorArc* arc);

/* --- Vertex accumulator that also writes OBJ 'v' lines immediately --- */
typedef struct {
  FILE* out;
  double** coords;
  size_t* count;
  size_t* cap;
} AddVertexCtx;

static size_t add_vertex_impl(double x, double y, double z, AddVertexCtx* c) {
  if (*(c->count) == *(c->cap)) {
    *(c->cap) = (*(c->cap) == 0 ? (size_t)1024 : (*(c->cap) * 2));
    *(c->coords) =
        (double*)realloc(*(c->coords), (*(c->cap)) * 3 * sizeof(double));
    if (!*(c->coords)) die("OOM vert_coords");
  }
  (*c->coords)[3 * (*(c->count)) + 0] = x;
  (*c->coords)[3 * (*(c->count)) + 1] = y;
  (*c->coords)[3 * (*(c->count)) + 2] = z;
  fprintf(c->out, "v %.6f %.6f %.6f\n", x, y, z);
  (*(c->count))++;
  return *(c->count); /* 1-based index */
}

/* --------- Impl --------- */
static void die(const char* msg) {
  fprintf(stderr, "Error: %s\n", msg);
  exit(1);
}

static Gray read_pgm(const char* path) {
  FILE* f = fopen(path, "rb");
  if (!f) die("cannot open input PGM");
  char magic[3] = {0};
  if (fscanf(f, "%2s", magic) != 1 || strcmp(magic, "P5") != 0)
    die("expecting P5 (binary PGM)");
  int w, h, maxv;
  if (fscanf(f, "%d %d %d", &w, &h, &maxv) != 3) die("bad PGM header");
  if (maxv != 255) die("expect 8-bit PGM (maxval 255)");
  fgetc(f); /* consume single whitespace after header */
  size_t N = (size_t)w * h;
  uint8_t* p = (uint8_t*)malloc(N);
  if (!p) die("OOM reading image");
  if (fread(p, 1, N, f) != N) die("short read of image data");
  fclose(f);
  Gray g = {w, h, p};
  return g;
}

static int cclabel(const uint8_t* mask, int w, int h, int conn, int* labels) {
  memset(labels, 0, (size_t)w * h * sizeof(int));
  int num_labels = 0;
  int dirs8[8][2] = {{1, 0}, {-1, 0}, {0, 1},  {0, -1},
                     {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
  int dirs4[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
  int maxQ = w * h;
  int* qx = (int*)malloc((size_t)maxQ * sizeof(int));
  int* qy = (int*)malloc((size_t)maxQ * sizeof(int));
  if (!qx || !qy) die("OOM cclabel");

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      size_t idx = (size_t)y * w + x;
      if (mask[idx] && labels[idx] == 0) {
        int label = ++num_labels;
        labels[idx] = label;
        int qb = 0, qe = 0;
        qx[qe] = x;
        qy[qe] = y;
        qe++;
        int (*dirs)[2] = (conn == 8 ? dirs8 : dirs4);
        int ndir = (conn == 8 ? 8 : 4);
        while (qb < qe) {
          int cx = qx[qb], cy = qy[qb];
          qb++;
          for (int k = 0; k < ndir; ++k) {
            int nx = cx + dirs[k][1];
            int ny = cy + dirs[k][0];
            if (nx >= 0 && nx < w && ny >= 0 && ny < h) {
              size_t nidx = (size_t)ny * w + nx;
              if (mask[nidx] && labels[nidx] == 0) {
                labels[nidx] = label;
                qx[qe] = nx;
                qy[qe] = ny;
                qe++;
              }
            }
          }
        }
      }
    }
  }
  free(qx);
  free(qy);
  return num_labels;
}

static void flood_outside(const uint8_t* free_mask, int w, int h,
                          uint8_t* outside) {
  memset(outside, 0, (size_t)w * h);
  int maxQ = w * h;
  int* qx = (int*)malloc((size_t)maxQ * sizeof(int));
  int* qy = (int*)malloc((size_t)maxQ * sizeof(int));
  if (!qx || !qy) die("OOM flood_outside");
  int qb = 0, qe = 0;

  for (int x = 0; x < w; ++x) {
    if (free_mask[0 * w + x]) {
      outside[0 * w + x] = 1;
      qx[qe] = x;
      qy[qe] = 0;
      qe++;
    }
    if (free_mask[(h - 1) * w + x]) {
      outside[(h - 1) * w + x] = 1;
      qx[qe] = x;
      qy[qe] = h - 1;
      qe++;
    }
  }
  for (int y = 0; y < h; ++y) {
    if (free_mask[y * w + 0]) {
      outside[y * w + 0] = 1;
      qx[qe] = 0;
      qy[qe] = y;
      qe++;
    }
    if (free_mask[y * w + (w - 1)]) {
      outside[y * w + (w - 1)] = 1;
      qx[qe] = w - 1;
      qy[qe] = y;
      qe++;
    }
  }

  int dirs[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
  while (qb < qe) {
    int cx = qx[qb], cy = qy[qb];
    qb++;
    for (int k = 0; k < 4; ++k) {
      int nx = cx + dirs[k][1];
      int ny = cy + dirs[k][0];
      if (nx >= 0 && nx < w && ny >= 0 && ny < h) {
        size_t nidx = (size_t)ny * w + nx;
        if (free_mask[nidx] && !outside[nidx]) {
          outside[nidx] = 1;
          qx[qe] = nx;
          qy[qe] = ny;
          qe++;
        }
      }
    }
  }
  free(qx);
  free(qy);
}

static bool circle_from3(int x1, int y1, int x2, int y2, int x3, int y3,
                         double* cx, double* cy, double* r) {
  double temp = (double)(x2 - x1) * (y3 - y1) - (double)(y2 - y1) * (x3 - x1);
  if (fabs(temp) < 1e-9) return false;
  double A1 = (double)x2 - x1, B1 = (double)y2 - y1;
  double A2 = (double)x3 - x1, B2 = (double)y3 - y1;
  double C1 = (A1 * (x1 + x2) + B1 * (y1 + y2)) / 2.0;
  double C2 = (A2 * (x1 + x3) + B2 * (y1 + y3)) / 2.0;
  double det = A1 * B2 - A2 * B1;
  if (fabs(det) < 1e-12) return false;
  *cx = (C1 * B2 - C2 * B1) / det;
  *cy = (A1 * C2 - A2 * C1) / det;
  *r = hypot(*cx - x1, *cy - y1);
  return true;
}

static int compare_doubles(const void* a, const void* b) {
  double da = *(const double*)a, db = *(const double*)b;
  return (da < db) ? -1 : (da > db) ? 1 : 0;
}

static bool validate_arc_brightness(const Gray* img, const DoorArc* arc) {
  int n = arc->nin;
  if (n < 5) return false;
  double* angles = (double*)malloc((size_t)n * sizeof(double));
  if (!angles) die("OOM validate_arc");

  for (int i = 0; i < n; ++i) {
    double dy = (double)arc->points[i].y - arc->cy;
    double dx = (double)arc->points[i].x - arc->cx;
    double ang = atan2(dy, dx);
    if (ang < 0) ang += 2 * M_PI;
    angles[i] = ang;
  }
  qsort(angles, (size_t)n, sizeof(double), compare_doubles);

  const double perc[5] = {0.10, 0.30, 0.50, 0.70, 0.90};
  int ok = 0;
  for (int pi = 0; pi < 5; ++pi) {
    int idx = (int)floor(perc[pi] * (n - 1) + 0.5);
    if (idx < 0) idx = 0;
    if (idx >= n) idx = n - 1;
    double ang = angles[idx];

    double ix = arc->cx + (arc->r - 10.0) * cos(ang);
    double iy = arc->cy + (arc->r - 10.0) * sin(ang);
    double ox = arc->cx + (arc->r + 10.0) * cos(ang);
    double oy = arc->cy + (arc->r + 10.0) * sin(ang);

    int iix = (int)llround(ix), iiy = (int)llround(iy);
    int oxx = (int)llround(ox), oyy = (int)llround(oy);

    if (iix >= 0 && iix < img->w && iiy >= 0 && iiy < img->h && oxx >= 0 &&
        oxx < img->w && oyy >= 0 && oyy < img->h) {
      uint8_t insideVal = img->p[(size_t)iiy * img->w + iix];
      uint8_t outsideVal = img->p[(size_t)oyy * img->w + oxx];
      if ((int)insideVal - (int)outsideVal > 4 && insideVal > 150 &&
          outsideVal < 245)
        ok++;
    }
  }
  free(angles);
  return (ok >= 2);
}

static int otsu(const Gray* img) {
  size_t N = (size_t)img->w * img->h;
  unsigned long long hist[OTSU_LEVELS] = {0};
  for (size_t i = 0; i < N; ++i) hist[img->p[i]]++;

  unsigned long long total = N;
  double sum = 0.0;
  for (int t = 0; t < OTSU_LEVELS; ++t) sum += (double)t * (double)hist[t];

  unsigned long long wB = 0;
  double sumB = 0.0, varMax = -1.0;
  int thr = 0;
  for (int t = 0; t < OTSU_LEVELS; ++t) {
    wB += hist[t];
    if (wB == 0) continue;
    unsigned long long wF = total - wB;
    if (wF == 0) break;

    sumB += (double)(t * hist[t]);
    double mB = sumB / (double)wB;
    double mF = (sum - sumB) / (double)wF;

    double varBetween = (double)wB * (double)wF * (mB - mF) * (mB - mF);
    if (varBetween > varMax) {
      varMax = varBetween;
      thr = t;
    }
  }
  return thr;
}

static void erode3x3(const uint8_t* in, uint8_t* out, int w, int h,
                     int iterations) {
  if (iterations <= 0) {
    memcpy(out, in, (size_t)w * h);
    return;
  }
  uint8_t* curr = (uint8_t*)malloc((size_t)w * h);
  uint8_t* temp = (uint8_t*)malloc((size_t)w * h);
  if (!curr || !temp) die("OOM erode");
  memcpy(curr, in, (size_t)w * h);

  for (int it = 0; it < iterations; ++it) {
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        int all1 = 1;
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            int nx = x + dx, ny = y + dy;
            if (nx < 0 || nx >= w || ny < 0 || ny >= h) {
              all1 = 0;
            } else if (!curr[(size_t)ny * w + nx]) {
              all1 = 0;
            }
            if (!all1) break;
          }
          if (!all1) break;
        }
        temp[(size_t)y * w + x] = (uint8_t)(all1 ? 1 : 0);
      }
    }
    uint8_t* sw = curr;
    curr = temp;
    temp = sw;
  }
  memcpy(out, curr, (size_t)w * h);
  free(curr);
  free(temp);
}

static void compute_arc_chord_from_points(DoorArc* arc) {
  if (!arc || !arc->points || arc->nin <= 1) {
    arc->chord_ready = 0;
    return;
  }
  double a_min = 1e9, a_max = -1e9;
  for (int i = 0; i < arc->nin; ++i) {
    double ang = atan2((double)arc->points[i].y - arc->cy,
                       (double)arc->points[i].x - arc->cx);
    if (ang < 0) ang += 2.0 * M_PI;
    if (ang < a_min) a_min = ang;
    if (ang > a_max) a_max = ang;
  }
  arc->ax = arc->cx + arc->r * cos(a_min);
  arc->ay = arc->cy + arc->r * sin(a_min);
  arc->bx = arc->cx + arc->r * cos(a_max);
  arc->by = arc->cy + arc->r * sin(a_max);
  arc->chord_ready = 1;
}

/* -------------------- MAIN -------------------- */
int main(int argc, char** argv) {
  srand((unsigned)time(NULL));

  if (argc < 3) {
    fprintf(stderr, "Usage: %s <input.pgm> <output.obj>\n", argv[0]);
    return 1;
  }
  const char* pgmPath = argv[1];
  const char* objPath = argv[2];

  Gray img = read_pgm(pgmPath);
  int W = img.w, H = img.h;
  size_t N = (size_t)W * H;

  uint8_t* walls = (uint8_t*)calloc(N, 1);
  uint8_t* free_space = (uint8_t*)calloc(N, 1);
  if (!walls || !free_space) die("OOM masks");

  /* 1) Thresholding */
  int thr = otsu(&img);
  for (size_t i = 0; i < N; ++i) {
    uint8_t v = img.p[i];
    walls[i] = (uint8_t)(v < thr ? 1 : 0);
    free_space[i] = (uint8_t)((v > thr + 10 && v > 180) ? 1 : 0);
  }
  for (size_t i = 0; i < N; ++i) {
    if (!walls[i] && !free_space[i]) {
      if (img.p[i] > thr)
        free_space[i] = 1;
      else
        walls[i] = 1;
    }
  }

  /* 2) Interior regions */
  uint8_t* outside = (uint8_t*)calloc(N, 1);
  uint8_t* interior = (uint8_t*)calloc(N, 1);
  if (!outside || !interior) die("OOM interior/outside");

  flood_outside(free_space, W, H, outside);
  for (size_t i = 0; i < N; ++i)
    interior[i] = (uint8_t)((free_space[i] && !outside[i]) ? 1 : 0);

  int* labels_int = (int*)malloc(N * sizeof(int));
  if (!labels_int) die("OOM labels_int");
  int num_int = cclabel(interior, W, H, 4, labels_int);

  typedef struct {
    int x0, y0, x1, y1, w, h;
  } BBox;
  BBox* boxes = (BBox*)malloc((size_t)(num_int + 1) * sizeof(BBox));
  bool* is_window = (bool*)calloc((size_t)(num_int + 1), 1);
  bool* valid_comp = (bool*)calloc((size_t)(num_int + 1), 1);
  if (!boxes || !is_window || !valid_comp) die("OOM component info");

  int largest_dim = 0;
  for (int lab = 1; lab <= num_int; ++lab) {
    int minx = W, miny = H, maxx = -1, maxy = -1;
    bool any = false;
    for (int y = 0; y < H; ++y)
      for (int x = 0; x < W; ++x) {
        if (labels_int[y * W + x] == lab) {
          any = true;
          if (x < minx) minx = x;
          if (x > maxx) maxx = x;
          if (y < miny) miny = y;
          if (y > maxy) maxy = y;
        }
      }
    if (!any) {
      valid_comp[lab] = false;
      continue;
    }
    valid_comp[lab] = true;
    boxes[lab].x0 = minx;
    boxes[lab].y0 = miny;
    boxes[lab].x1 = maxx;
    boxes[lab].y1 = maxy;
    boxes[lab].w = maxx - minx + 1;
    boxes[lab].h = maxy - miny + 1;
    if (boxes[lab].w > largest_dim) largest_dim = boxes[lab].w;
    if (boxes[lab].h > largest_dim) largest_dim = boxes[lab].h;
  }
  for (int lab = 1; lab <= num_int; ++lab) {
    if (!valid_comp[lab]) continue;
    int w = boxes[lab].w, h = boxes[lab].h;
    int minSide = (w < h ? w : h);
    if ((double)minSide <= 0.10 * (double)largest_dim) is_window[lab] = true;
  }
  int count_rooms = 0, count_windows = 0;
  for (int lab = 1; lab <= num_int; ++lab) {
    if (!valid_comp[lab]) continue;
    if (is_window[lab])
      count_windows++;
    else
      count_rooms++;
  }

  /* 3) Wall edges & door arcs */
  uint8_t* walls_eroded = (uint8_t*)calloc(N, 1);
  uint8_t* edge_mask = (uint8_t*)calloc(N, 1);
  if (!walls_eroded || !edge_mask) die("OOM edges");

  erode3x3(walls, walls_eroded, W, H, 1);
  for (size_t i = 0; i < N; ++i)
    edge_mask[i] = (uint8_t)((walls[i] && !walls_eroded[i]) ? 1 : 0);

  int* labels_edge = (int*)malloc(N * sizeof(int));
  if (!labels_edge) die("OOM labels_edge");
  int num_edges = cclabel(edge_mask, W, H, 8, labels_edge);

  DoorArc* door_arcs = NULL;
  int arcs_count = 0, arcs_cap = 0;
#define ADD_ARC(cx_, cy_, r_, span_, pts_, npts_)                           \
  do {                                                                      \
    if (arcs_count == arcs_cap) {                                           \
      arcs_cap = (arcs_cap == 0 ? 8 : arcs_cap * 2);                        \
      door_arcs =                                                           \
          (DoorArc*)realloc(door_arcs, (size_t)arcs_cap * sizeof(DoorArc)); \
      if (!door_arcs) die("OOM realloc arcs");                              \
    }                                                                       \
    door_arcs[arcs_count].cx = (cx_);                                       \
    door_arcs[arcs_count].cy = (cy_);                                       \
    door_arcs[arcs_count].r = (r_);                                         \
    door_arcs[arcs_count].span_deg = (span_);                               \
    door_arcs[arcs_count].nin = (npts_);                                    \
    door_arcs[arcs_count].points = (pts_);                                  \
    door_arcs[arcs_count].chord_ready = 0;                                  \
    arcs_count++;                                                           \
  } while (0)

  for (int lab = 1; lab <= num_edges; ++lab) {
    int count = 0;
    for (size_t i = 0; i < N; ++i)
      if (labels_edge[i] == lab) count++;
    if (count < 50) continue;

    Px* pts = (Px*)malloc((size_t)count * sizeof(Px));
    if (!pts) die("OOM pts");
    int k = 0;
    for (int y = 0; y < H; ++y)
      for (int x = 0; x < W; ++x)
        if (labels_edge[y * W + x] == lab) {
          pts[k].x = x;
          pts[k].y = y;
          k++;
        }

    uint8_t* used = (uint8_t*)calloc((size_t)count, 1);
    if (!used) die("OOM used");
    int arcs_found = 0, max_arcs = 3;

    while (arcs_found < max_arcs) {
      int avail = 0;
      for (int i = 0; i < count; ++i)
        if (!used[i]) avail++;
      if (avail < 50) break;
      int* avail_idx = (int*)malloc((size_t)avail * sizeof(int));
      if (!avail_idx) die("OOM avail_idx");
      int ai = 0;
      for (int i = 0; i < count; ++i)
        if (!used[i]) avail_idx[ai++] = i;

      int best_inl = 0;
      int* best_list = NULL;
      double best_cx = 0, best_cy = 0, best_r = 0, best_span = 0;

      for (int it = 0; it < 4000; ++it) {
        if (avail < 3) break;
        int i1 = avail_idx[rand() % avail], i2 = avail_idx[rand() % avail],
            i3 = avail_idx[rand() % avail];
        if (i1 == i2 || i1 == i3 || i2 == i3) continue;

        double cx, cy, r;
        if (!circle_from3(pts[i1].x, pts[i1].y, pts[i2].x, pts[i2].y, pts[i3].x,
                          pts[i3].y, &cx, &cy, &r))
          continue;
        if (r < 5.0 || r > 85.0) continue;

        int* inl = (int*)malloc((size_t)avail * sizeof(int));
        if (!inl) die("OOM inliers");
        int ic = 0;
        for (int t = 0; t < avail; ++t) {
          int idx = avail_idx[t];
          double dx = (double)pts[idx].x - cx;
          double dy = (double)pts[idx].y - cy;
          double dist = fabs(hypot(dx, dy) - r);
          if (dist < 4.5) inl[ic++] = idx;
        }
        if (ic < 18) {
          free(inl);
          continue;
        }

        double minA = 360.0, maxA = 0.0;
        for (int m = 0; m < ic; ++m) {
          double ang =
              atan2((double)pts[inl[m]].y - cy, (double)pts[inl[m]].x - cx);
          if (ang < 0) ang += 2 * M_PI;
          double deg = ang * 180.0 / M_PI;
          if (deg < minA) minA = deg;
          if (deg > maxA) maxA = deg;
        }
        double span = maxA - minA;
        if (span < 0) span += 360.0;
        if (!((span >= 10.0 && span <= 80.0) ||
              (span >= 100.0 && span <= 120.0))) {
          free(inl);
          continue;
        }

        if (ic > best_inl) {
          if (best_list) free(best_list);
          best_list = inl;
          best_inl = ic;
          best_cx = cx;
          best_cy = cy;
          best_r = r;
          best_span = span;
        } else {
          free(inl);
        }
      }

      free(avail_idx);
      if (best_inl == 0) {
        if (best_list) free(best_list);
        break;
      }

      Px* arc_pts = (Px*)malloc((size_t)best_inl * sizeof(Px));
      if (!arc_pts) die("OOM arc_pts");
      for (int m = 0; m < best_inl; ++m) {
        int idx = best_list[m];
        arc_pts[m] = pts[idx];
        used[idx] = 1;
      }
      ADD_ARC(best_cx, best_cy, best_r, best_span, arc_pts, best_inl);
      arcs_found++;
      free(best_list);
    }

    free(used);
    free(pts);
  }

  /* Chords (not essential for mask method, but computed) */
  for (int i = 0; i < arcs_count; ++i)
    compute_arc_chord_from_points(&door_arcs[i]);

  /* Validate arcs; only validated arcs contribute to door mask */
  bool* is_door_arc = (bool*)malloc((size_t)arcs_count * sizeof(bool));
  if (!is_door_arc) die("OOM door flags");
  int door_count = 0;
  for (int i = 0; i < arcs_count; ++i) {
    bool ok = validate_arc_brightness(&img, &door_arcs[i]);
    is_door_arc[i] = ok;
    if (!ok)
      door_arcs[i].chord_ready = 0;
    else
      door_count++;
  }

  /* 3b) Build door pixel mask from arc inliers and dilate to ~48" */
  uint8_t* door_mask = (uint8_t*)calloc((size_t)W * H, 1);
  if (!door_mask) die("OOM door_mask");
  for (int i = 0; i < arcs_count; ++i) {
    if (!is_door_arc[i]) continue;
    DoorArc* A = &door_arcs[i];
    for (int m = 0; m < A->nin; ++m) {
      int x = A->points[m].x, y = A->points[m].y;
      if (x >= 0 && x < W && y >= 0 && y < H) door_mask[(size_t)y * W + x] = 1;
    }
  }
  /* Chebyshev dilation by DOOR_DILATE_R */
  uint8_t* tmp = (uint8_t*)calloc((size_t)W * H, 1);
  if (!tmp) die("OOM door_tmp");
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      if (!door_mask[(size_t)y * W + x]) continue;
      int x0 = (x - DOOR_DILATE_R < 0) ? 0 : x - DOOR_DILATE_R;
      int x1 = (x + DOOR_DILATE_R >= W) ? W - 1 : x + DOOR_DILATE_R;
      int y0 = (y - DOOR_DILATE_R < 0) ? 0 : y - DOOR_DILATE_R;
      int y1 = (y + DOOR_DILATE_R >= H) ? H - 1 : y + DOOR_DILATE_R;
      for (int yy = y0; yy <= y1; ++yy) {
        memset(&tmp[(size_t)yy * W + x0], 1, (size_t)(x1 - x0 + 1));
      }
    }
  }
  memcpy(door_mask, tmp, (size_t)W * H);
  free(tmp);

  /* 4) Write OBJ */
  FILE* fobj = fopen(objPath, "w");
  if (!fobj) die("cannot open output OBJ");

  double* verts = NULL;
  size_t vcount = 0, vcap = 0;
  AddVertexCtx avc;
  avc.out = fobj;
  avc.coords = &verts;
  avc.count = &vcount;
  avc.cap = &vcap;

  /* Floors */
  for (int lab = 1; lab <= num_int; ++lab) {
    if (!valid_comp[lab] || is_window[lab]) continue;
    for (int y = 0; y < H; ++y) {
      for (int x = 0; x < W; ++x) {
        if (labels_int[y * W + x] == lab) {
          size_t v1 = add_vertex_impl((double)x, (double)y, 0.0, &avc);
          size_t v2 = add_vertex_impl((double)(x + 1), (double)y, 0.0, &avc);
          size_t v3 =
              add_vertex_impl((double)(x + 1), (double)(y + 1), 0.0, &avc);
          size_t v4 = add_vertex_impl((double)x, (double)(y + 1), 0.0, &avc);
          fprintf(fobj, "f %zu %zu %zu %zu\n", v1, v2, v3, v4);
        }
      }
    }
  }

  /* Walls with rectangular door cuts (48"×84") wherever neighbor wall-pixel is
   * in door_mask */
  for (int lab = 1; lab <= num_int; ++lab) {
    if (!valid_comp[lab] || is_window[lab]) continue;
    for (int y = 0; y < H; ++y) {
      for (int x = 0; x < W; ++x) {
        if (labels_int[y * W + x] != lab) continue;

        struct {
          int dx, dy;
        } dirs[4] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
        for (int d = 0; d < 4; ++d) {
          int nx = x + dirs[d].dx, ny = y + dirs[d].dy;
          bool neighbor_is_wall = false;
          if (nx < 0 || nx >= W || ny < 0 || ny >= H)
            neighbor_is_wall = true;
          else if (walls[ny * W + nx])
            neighbor_is_wall = true;
          if (!neighbor_is_wall) continue;

          bool skip_window = false;
          if (nx >= 0 && nx < W && ny >= 0 && ny < H) {
            int nlab = labels_int[ny * W + nx];
            if (nlab > 0 && is_window[nlab]) skip_window = true;
          }
          if (skip_window) continue;

          /* Door mask test on the neighbor wall pixel */
          int in_door = 0;
          if (nx >= 0 && nx < W && ny >= 0 && ny < H) {
            if (door_mask[(size_t)ny * W + nx]) in_door = 1;
          }

          double z0 = in_door ? DOOR_H_PX : 0.0;
          double z1 = HEIGHT_PX;
          if (z0 >= z1 - 1e-9) continue;

          double x0 = x + (dirs[d].dx == 1 ? 1.0 : 0.0);
          double y0 = y + (dirs[d].dy == 1 ? 1.0 : 0.0);
          double x1 = x0 + (dirs[d].dy != 0 ? 1.0 : 0.0);
          double y1 = y0 + (dirs[d].dx != 0 ? 1.0 : 0.0);

          size_t vb1 = add_vertex_impl(x0, y0, z0, &avc);
          size_t vb2 = add_vertex_impl(x1, y1, z0, &avc);
          size_t vt1 = add_vertex_impl(x0, y0, z1, &avc);
          size_t vt2 = add_vertex_impl(x1, y1, z1, &avc);
          fprintf(fobj, "f %zu %zu %zu %zu\n", vb1, vb2, vt2, vt1);
        }
      }
    }
  }

  fclose(fobj);

  /* Cleanup */
  for (int i = 0; i < arcs_count; ++i) free(door_arcs[i].points);
  free(door_arcs);
  free(is_door_arc);
  free(door_mask);
  free(labels_edge);
  free(edge_mask);
  free(walls_eroded);
  free(is_window);
  free(valid_comp);
  free(boxes);
  free(labels_int);
  free(interior);
  free(outside);
  free(free_space);
  free(walls);
  free(img.p);
  free(verts);

  printf("Detected %d rooms, %d windows, %d doors. 3D model saved to %s\n",
         count_rooms, count_windows, door_count, objPath);
  return 0;
}
