/*3D Building Reconstruction from Floor Plans Using C and Classical Algorithms

Subject: CSE 6010
Submitted By:

Zhang, Mengping
Ma, Hsu, Chieh
Gao, Tianxiang
Nation, Ryan T
Imran Aziz

*/
#include <math.h>
#include <omp.h>
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
#define HEIGHT_PX 180 /* wall height (inches) */
#define OTSU_LEVELS 256
#define DOOR_H_PX 84           /* door opening height in pixels (~7 ft) */
#define DOOR_LINE_MAX_THICK 84 /* max thickness (px) for straight red lines */

#define MIN_ROOM_FILL_RATIO 0.55

typedef struct {
  int w, h;
  uint8_t* p;         /* grayscale / luminance */
  uint8_t *r, *g, *b; /* color planes for PPM, NULL for PGM */
  int is_color;
} Image;

typedef struct {
  int xmin, ymin, xmax, ymax;
} Box;

typedef struct {
  int x, y;
} Node;

/* Simple grayscale wrapper + door-arc types for RANSAC-based detection */

typedef struct {
  int w, h;
  uint8_t* p;
} Gray;

typedef struct {
  int x, y;
} Px;

typedef struct {
  double cx, cy, r; /* circle center and radius */
  double span_deg;  /* arc span in degrees */
  int nin;          /* number of inlier points */
  Px* points;
  double ax, ay, bx, by; /* chord endpoints */
  int chord_ready;
} DoorArc;

/* OBJ writer helper */

typedef struct {
  FILE* f;
  int v_offset;
} ObjWriter;

/* Utility */
static void die(const char* msg) {
  fprintf(stderr, "FATAL: %s\n", msg);
  exit(1);
}

/* ---- PNM reading ---- */

static Image read_pnm_with_doors(const char* path, uint8_t** door_seed_out) {
  FILE* f = fopen(path, "rb");
  if (!f) die("Cannot open input image");

  char header[3] = {0};
  if (fscanf(f, "%2s", header) != 1) die("Bad PNM header");

  int is_color = 0;
  if (!strcmp(header, "P5"))
    is_color = 0;
  else if (!strcmp(header, "P6"))
    is_color = 1;
  else
    die("Only P5/P6 supported");

  int c;
  do {
    c = fgetc(f);
  } while (c == ' ' || c == '\n' || c == '\r' || c == '\t');
  if (c == '#') {
    do {
      c = fgetc(f);
    } while (c != '\n' && c != EOF);
  } else {
    ungetc(c, f);
  }

  int W, H, maxval;
  if (fscanf(f, "%d %d", &W, &H) != 2) die("Bad size");
  if (fscanf(f, "%d", &maxval) != 1) die("Bad maxval");
  if (maxval != 255) die("Expect maxval 255");
  fgetc(f); /* consume whitespace */

  size_t N = (size_t)W * H;

  Image img;
  img.w = W;
  img.h = H;
  img.is_color = is_color;
  img.p = (uint8_t*)malloc(N);
  img.r = img.g = img.b = NULL;
  if (!img.p) die("OOM img.p");

  uint8_t* door_seed = (uint8_t*)calloc(N, 1);
  if (!door_seed) die("OOM door_seed");

  if (!is_color) {
    if (fread(img.p, 1, N, f) != N) die("Short read P5");
  } else {
    uint8_t* rgb = (uint8_t*)malloc(3 * N);
    if (!rgb) die("OOM rgb");
    if (fread(rgb, 1, 3 * N, f) != 3 * N) die("Short read P6");

    img.r = (uint8_t*)malloc(N);
    img.g = (uint8_t*)malloc(N);
    img.b = (uint8_t*)malloc(N);
    if (!img.r || !img.g || !img.b) die("OOM color planes");

    for (size_t i = 0; i < N; ++i) {
      uint8_t R = rgb[3 * i + 0];
      uint8_t G = rgb[3 * i + 1];
      uint8_t B = rgb[3 * i + 2];
      img.r[i] = R;
      img.g[i] = G;
      img.b[i] = B;
      uint8_t Y = (uint8_t)(0.299 * R + 0.587 * G + 0.114 * B);
      img.p[i] = Y;

      /* robust-ish red-door detection */
      if (R > 140 && R >= G + 30 && R >= B + 30) {
        door_seed[i] = 1;
      }
    }
    free(rgb);
  }

  fclose(f);
  *door_seed_out = door_seed;
  return img;
}

/* ----- Otsu threshold ----- */

static int otsu_threshold(const uint8_t* gray, int W, int H) {
  size_t N = (size_t)W * H;
  long hist[OTSU_LEVELS] = {0};

  for (size_t i = 0; i < N; ++i) {
    hist[gray[i]]++;
  }

  long total = (long)N;
  double sumB = 0.0;
  double wB = 0.0;
  double maximum = 0.0;
  double sum1 = 0.0;
  for (int i = 0; i < OTSU_LEVELS; ++i) {
    sum1 += i * (double)hist[i];
  }

  int threshold = 0;
  for (int t = 0; t < OTSU_LEVELS; ++t) {
    wB += hist[t];
    if (wB == 0) continue;
    double wF = (double)total - wB;
    if (wF == 0) break;
    sumB += t * (double)hist[t];
    double mB = sumB / wB;
    double mF = (sum1 - sumB) / wF;
    double between = wB * wF * (mB - mF) * (mB - mF);
    if (between > maximum) {
      maximum = between;
      threshold = t;
    }
  }
  return threshold;
}

/* ---- BFS flood for outside ---- */

static void flood_outside(const uint8_t* free_space, int W, int H,
                          uint8_t* outside) {
  int maxQ = W * H;
  Node* queue = (Node*)malloc((size_t)maxQ * sizeof(Node));
  if (!queue) die("OOM queue");

  int qhead = 0, qtail = 0;

  /* seed with frame pixels that are free_space */
  for (int x = 0; x < W; ++x) {
    if (free_space[x]) {
      outside[x] = 1;
      queue[qtail++] = (Node){x, 0};
    }
    if (free_space[(H - 1) * W + x]) {
      outside[(H - 1) * W + x] = 1;
      queue[qtail++] = (Node){x, H - 1};
    }
  }
  for (int y = 0; y < H; ++y) {
    if (free_space[y * W]) {
      outside[y * W] = 1;
      queue[qtail++] = (Node){0, y};
    }
    if (free_space[y * W + (W - 1)]) {
      outside[y * W + (W - 1)] = 1;
      queue[qtail++] = (Node){W - 1, y};
    }
  }

  static const int dx[4] = {1, -1, 0, 0};
  static const int dy[4] = {0, 0, 1, -1};

  while (qhead < qtail) {
    Node n = queue[qhead++];
    for (int k = 0; k < 4; ++k) {
      int nx = n.x + dx[k];
      int ny = n.y + dy[k];
      if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
      int idx = ny * W + nx;
      if (!outside[idx] && free_space[idx]) {
        outside[idx] = 1;
        queue[qtail++] = (Node){nx, ny};
      }
    }
  }
  free(queue);
}

/* Connected component labeling (4- or 8-connected) */

static int cclabel(const uint8_t* mask, int W, int H, int conn4, int* labels) {
  int N = W * H;
  for (int i = 0; i < N; ++i) labels[i] = 0;

  Node* stack = (Node*)malloc((size_t)N * sizeof(Node));
  if (!stack) die("OOM stack");

  static const int dx4[4] = {1, -1, 0, 0};
  static const int dy4[4] = {0, 0, 1, -1};
  static const int dx8[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static const int dy8[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  const int* dx = conn4 ? dx4 : dx8;
  const int* dy = conn4 ? dy4 : dy8;
  int neigh = conn4 ? 4 : 8;

  int max_label = 0;

  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      int idx0 = y * W + x;
      if (!mask[idx0] || labels[idx0] != 0) continue;

      ++max_label;
      int sp = 0;
      stack[sp++] = (Node){x, y};
      labels[idx0] = max_label;

      while (sp > 0) {
        Node n = stack[--sp];
        int ix = n.x;
        int iy = n.y;
        int i = iy * W + ix;
        for (int k = 0; k < neigh; ++k) {
          int nx = ix + dx[k];
          int ny = iy + dy[k];
          if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
          int j = ny * W + nx;
          if (mask[j] && labels[j] == 0) {
            labels[j] = max_label;
            stack[sp++] = (Node){nx, ny};
          }
        }
      }
    }
  }

  free(stack);
  return max_label;
}

/* ===== RANSAC-BASED DOOR ARC DETECTION ===== */

static int compare_doubles(const void* a, const void* b) {
  double da = *(const double*)a;
  double db = *(const double*)b;
  return (da < db) ? -1 : (da > db) ? 1 : 0;
}

static bool circle_from3(int x1, int y1, int x2, int y2, int x3, int y3,
                         double* cx, double* cy, double* r) {
  double temp = (double)(x2 - x1) * (y3 - y1) - (double)(y2 - y1) * (x3 - x1);
  if (fabs(temp) < 1e-9) return false;

  double A1 = (double)x2 - x1;
  double B1 = (double)y2 - y1;
  double A2 = (double)x3 - x1;
  double B2 = (double)y3 - y1;
  double C1 = (A1 * (x1 + x2) + B1 * (y1 + y2)) / 2.0;
  double C2 = (A2 * (x1 + x3) + B2 * (y1 + y3)) / 2.0;
  double det = A1 * B2 - A2 * B1;
  if (fabs(det) < 1e-12) return false;

  *cx = (C1 * B2 - C2 * B1) / det;
  *cy = (A1 * C2 - A2 * C1) / det;
  *r = hypot(*cx - x1, *cy - y1);
  return true;
}

static bool validate_arc_brightness(const Gray* img, const DoorArc* arc) {
  int n = arc->nin;
  if (n < 5) return false;

  double* angles = (double*)malloc((size_t)n * sizeof(double));
  if (!angles) die("OOM validate_arc_brightness");

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

    int iix = (int)llround(ix);
    int iiy = (int)llround(iy);
    int oxx = (int)llround(ox);
    int oyy = (int)llround(oy);

    if (iix >= 0 && iix < img->w && iiy >= 0 && iiy < img->h && oxx >= 0 &&
        oxx < img->w && oyy >= 0 && oyy < img->h) {
      uint8_t insideVal = img->p[(size_t)iiy * img->w + iix];
      uint8_t outsideVal = img->p[(size_t)oyy * img->w + oxx];
      if ((int)insideVal - (int)outsideVal > 4 && insideVal > 150 &&
          outsideVal < 245) {
        ok++;
      }
    }
  }

  free(angles);
  return (ok >= 2);
}

/* simple 3x3 erosion used to get a 1-px thick edge from walls */
static void erode3x3(const uint8_t* in, uint8_t* out, int w, int h,
                     int iterations) {
  if (iterations <= 0) {
    memcpy(out, in, (size_t)w * h);
    return;
  }

  uint8_t* curr = (uint8_t*)malloc((size_t)w * h);
  uint8_t* temp = (uint8_t*)malloc((size_t)w * h);
  if (!curr || !temp) die("OOM erode3x3");

  memcpy(curr, in, (size_t)w * h);

  for (int it = 0; it < iterations; ++it) {
    int y;
#pragma omp parallel for
    for (y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        int all1 = 1;
        for (int dy = -1; dy <= 1 && all1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            int nx = x + dx;
            int ny = y + dy;
            if (nx < 0 || nx >= w || ny < 0 || ny >= h ||
                !curr[(size_t)ny * w + nx]) {
              all1 = 0;
              break;
            }
          }
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

/* Run RANSAC on wall edges restricted to a neighborhood of door_seed_arcs.
 * Returns number of arcs that pass brightness validation.
 */
static int detect_doors_ransac(const Gray* img, const uint8_t* walls,
                               const uint8_t* door_seed_arcs, int W, int H,
                               int* out_total_arcs) {
  size_t N = (size_t)W * H;

  uint8_t* walls_eroded = (uint8_t*)calloc(N, 1);
  uint8_t* edge_mask = (uint8_t*)calloc(N, 1);
  if (!walls_eroded || !edge_mask) die("OOM detect_doors_ransac");

  erode3x3(walls, walls_eroded, W, H, 1);

  /* 1-pixel-wide wall edges */
  int y;
#pragma omp parallel for
  for (y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      size_t i = (size_t)y * W + x;
      edge_mask[i] = (uint8_t)(walls[i] && !walls_eroded[i]);
    }
  }

  /* Restrict edge_mask to a small neighborhood around red door arcs */
  uint8_t* door_influence = (uint8_t*)calloc(N, 1);
  if (!door_influence) die("OOM door_influence");

  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      size_t idx = (size_t)y * W + x;
      if (!door_seed_arcs[idx]) continue;
      for (int dy = -3; dy <= 3; ++dy) {
        int ny = y + dy;
        if (ny < 0 || ny >= H) continue;
        for (int dx = -3; dx <= 3; ++dx) {
          int nx = x + dx;
          if (nx < 0 || nx >= W) continue;
          door_influence[(size_t)ny * W + nx] = 1;
        }
      }
    }
  }

  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      size_t idx = (size_t)y * W + x;
      if (!door_influence[idx]) edge_mask[idx] = 0;
    }
  }
  free(door_influence);

  int* labels_edge = (int*)malloc(N * sizeof(int));
  if (!labels_edge) die("OOM labels_edge");
  int num_edges = cclabel(edge_mask, W, H, 0, labels_edge); /* 8-connected */

  DoorArc* door_arcs = NULL;
  int arcs_count = 0;
  int arcs_cap = 0;

#define ADD_ARC(cx_, cy_, r_, span_, pts_, npts_)                           \
  do {                                                                      \
    if (arcs_count == arcs_cap) {                                           \
      arcs_cap = (arcs_cap == 0 ? 8 : arcs_cap * 2);                        \
      door_arcs =                                                           \
          (DoorArc*)realloc(door_arcs, (size_t)arcs_cap * sizeof(DoorArc)); \
      if (!door_arcs) die("OOM realloc door_arcs");                         \
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

  /* For each edge component, run RANSAC to find up to a few arcs */
  for (int lab = 1; lab <= num_edges; ++lab) {
    int count = 0;
    for (size_t i = 0; i < N; ++i) {
      if (labels_edge[i] == lab) count++;
    }
    if (count < 50) continue;

    Px* pts = (Px*)malloc((size_t)count * sizeof(Px));
    if (!pts) die("OOM pts");
    int k = 0;
    for (int y = 0; y < H; ++y) {
      for (int x = 0; x < W; ++x) {
        if (labels_edge[y * W + x] == lab) {
          pts[k].x = x;
          pts[k].y = y;
          k++;
        }
      }
    }

    uint8_t* used = (uint8_t*)calloc((size_t)count, 1);
    if (!used) die("OOM used");
    int arcs_found = 0;
    int max_arcs_per_comp = 1; /* limit per component */

    while (arcs_found < max_arcs_per_comp) {
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
      double best_cx = 0.0, best_cy = 0.0, best_r = 0.0, best_span = 0.0;

      for (int it = 0; it < 4000; ++it) {
        if (avail < 3) break;
        int i1 = avail_idx[rand() % avail];
        int i2 = avail_idx[rand() % avail];
        int i3 = avail_idx[rand() % avail];
        if (i1 == i2 || i1 == i3 || i2 == i3) continue;

        double cx_, cy_, r_;
        if (!circle_from3(pts[i1].x, pts[i1].y, pts[i2].x, pts[i2].y, pts[i3].x,
                          pts[i3].y, &cx_, &cy_, &r_))
          continue;
        if (r_ < 5.0 || r_ > 85.0) continue;

        int* inl = (int*)malloc((size_t)avail * sizeof(int));
        if (!inl) die("OOM inliers");
        int ic = 0;
        for (int t = 0; t < avail; ++t) {
          int idx = avail_idx[t];
          double dx = (double)pts[idx].x - cx_;
          double dy = (double)pts[idx].y - cy_;
          double dist = fabs(hypot(dx, dy) - r_);
          if (dist < 4.5) inl[ic++] = idx;
        }
        if (ic < 18) {
          free(inl);
          continue;
        }

        double minA = 360.0, maxA = 0.0;
        for (int m = 0; m < ic; ++m) {
          double ang =
              atan2((double)pts[inl[m]].y - cy_, (double)pts[inl[m]].x - cx_);
          if (ang < 0) ang += 2 * M_PI;
          double deg = ang * 180.0 / M_PI;
          if (deg < minA) minA = deg;
          if (deg > maxA) maxA = deg;
        }
        double span = maxA - minA;
        if (span < 0) span += 360.0;

        /* accept moderate arcs */
        if (!((span >= 10.0 && span <= 80.0) ||
              (span >= 100.0 && span <= 120.0))) {
          free(inl);
          continue;
        }

        if (ic > best_inl) {
          if (best_list) free(best_list);
          best_list = inl;
          best_inl = ic;
          best_cx = cx_;
          best_cy = cy_;
          best_r = r_;
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

#undef ADD_ARC

  for (int i = 0; i < arcs_count; ++i) {
    compute_arc_chord_from_points(&door_arcs[i]);
  }

  bool* is_door_arc = (bool*)malloc((size_t)arcs_count * sizeof(bool));
  if (!is_door_arc) die("OOM is_door_arc");

  int door_count = 0;
  for (int i = 0; i < arcs_count; ++i) {
    bool ok = validate_arc_brightness(img, &door_arcs[i]);
    is_door_arc[i] = ok;
    if (ok) {
      door_count++;
    } else {
      door_arcs[i].chord_ready = 0;
    }
  }

  if (out_total_arcs) *out_total_arcs = arcs_count;

  for (int i = 0; i < arcs_count; ++i) {
    free(door_arcs[i].points);
  }
  free(door_arcs);
  free(is_door_arc);
  free(labels_edge);
  free(edge_mask);
  free(walls_eroded);

  return door_count;
}

/* --- OBJ writer helpers --- */

static void obj_begin(ObjWriter* ow, const char* path) {
  ow->f = fopen(path, "w");
  if (!ow->f) die("Cannot open OBJ for writing");
  ow->v_offset = 0;
  fprintf(ow->f, "# OBJ generated from floorplan\n");
}

static int obj_add_vertex(ObjWriter* ow, double x, double y, double z) {
  fprintf(ow->f, "v %.3f %.3f %.3f\n", x, y, z);
  return ++ow->v_offset;
}

static void obj_add_quad(ObjWriter* ow, int v1, int v2, int v3, int v4) {
  fprintf(ow->f, "f %d %d %d %d\n", v1, v2, v3, v4);
}

static void obj_end(ObjWriter* ow) {
  if (ow->f) fclose(ow->f);
}

/* --- main --- */

int main(int argc, char** argv) {
  if (argc < 3) {
    fprintf(stderr, "Usage: %s input.pgm/ppm output_basename\n", argv[0]);
    return 1;
  }

  const char* inPath = argv[1];
  const char* objBase = argv[2];

  uint8_t* door_seed = NULL;

  double t_start = omp_get_wtime();

  Image img = read_pnm_with_doors(inPath, &door_seed);
  int W = img.w;
  int H = img.h;
  size_t N = (size_t)W * H;

  int thr = otsu_threshold(img.p, W, H);

  uint8_t* walls = (uint8_t*)calloc(N, 1);
  uint8_t* free_space = (uint8_t*)calloc(N, 1);
  if (!walls || !free_space) die("OOM walls/free_space");

  size_t total_bytes = 0;
  total_bytes += N * sizeof(uint8_t); /* walls */
  total_bytes += N * sizeof(uint8_t); /* free_space */
  total_bytes += N * sizeof(uint8_t); /* door_seed */

  int x, y;

#pragma omp parallel for private(x)
  for (y = 0; y < H; ++y) {
    for (x = 0; x < W; ++x) {
      size_t i = (size_t)y * W + x;
      uint8_t g = img.p[i];
      if (g < thr)
        walls[i] = 1;
      else if (g > thr + 10 && g > 180)
        free_space[i] = 1;
    }
  }

  /* if any pixel is neither wall nor free_space, nudge */
#pragma omp parallel for private(x)
  for (y = 0; y < H; ++y) {
    for (x = 0; x < W; ++x) {
      size_t i = (size_t)y * W + x;
      if (!walls[i] && !free_space[i]) {
        if (img.p[i] > thr)
          free_space[i] = 1;
        else
          walls[i] = 1;
      }
    }
  }

  /* 1b) door mask / labels */

  uint8_t* door_wall_mask = (uint8_t*)calloc(N, 1);
  if (!door_wall_mask) die("OOM door_wall_mask");
  total_bytes += N * sizeof(uint8_t);

  int* door_labels = (int*)malloc(N * sizeof(int));
  if (!door_labels) die("OOM door_labels");
  total_bytes += N * sizeof(int);

  uint8_t* door_seed_lines = (uint8_t*)calloc(N, 1);
  uint8_t* door_seed_arcs = (uint8_t*)calloc(N, 1);
  if (!door_seed_lines || !door_seed_arcs) die("OOM door_seed split");
  total_bytes += 2 * N * sizeof(uint8_t);

  int door_segments_all =
      cclabel(door_seed, W, H, 1, door_labels); /* 4-connected */
  int* door_is_line = (int*)malloc((door_segments_all + 1) * sizeof(int));
  if (!door_is_line) die("OOM door_is_line");
  for (int k = 0; k <= door_segments_all; ++k) door_is_line[k] = 0;

  int door_segments = 0;

  /* Classify each red component as straight line vs curved arc based on
   * thickness */
  for (int dlab = 1; dlab <= door_segments_all; ++dlab) {
    int minx = W, miny = H, maxx = -1, maxy = -1;
    int any = 0;
    for (y = 0; y < H; ++y) {
      for (x = 0; x < W; ++x) {
        if (door_labels[y * W + x] == dlab) {
          any = 1;
          if (x < minx) minx = x;
          if (x > maxx) maxx = x;
          if (y < miny) miny = y;
          if (y > maxy) maxy = y;
        }
      }
    }
    if (!any) continue;

    int w = maxx - minx + 1;
    int h = maxy - miny + 1;
    if (w < 2 && h < 2) continue; /* too tiny */

    int horizontal = (w >= h);
    int thickness = horizontal ? h : w;
    int is_line = (thickness <= DOOR_LINE_MAX_THICK);

    door_is_line[dlab] = is_line;
    if (is_line) door_segments++;
  }

  /* Populate door_seed_lines and door_seed_arcs */
  for (int dlab = 1; dlab <= door_segments_all; ++dlab) {
    int is_line = door_is_line[dlab];
    for (y = 0; y < H; ++y) {
      for (x = 0; x < W; ++x) {
        if (door_labels[y * W + x] != dlab) continue;
        size_t idx = (size_t)y * W + x;
        if (is_line)
          door_seed_lines[idx] = 1;
        else
          door_seed_arcs[idx] = 1;
      }
    }
  }

  /* build door_wall_mask only from straight-door-line components */
  for (int dlab = 1; dlab <= door_segments_all; ++dlab) {
    if (!door_is_line[dlab]) continue;

    int minx = W, miny = H, maxx = -1, maxy = -1;
    int any = 0;
    for (y = 0; y < H; ++y) {
      for (x = 0; x < W; ++x) {
        if (door_labels[y * W + x] == dlab) {
          any = 1;
          if (x < minx) minx = x;
          if (x > maxx) maxx = x;
          if (y < miny) miny = y;
          if (y > maxy) maxy = y;
        }
      }
    }
    if (!any) continue;

    int w = maxx - minx + 1;
    int h = maxy - miny + 1;
    if (w < 2 && h < 2) continue;

    int horizontal = (w >= h);
    if (horizontal) {
      int cy = (miny + maxy) / 2;
      if (cy < 0 || cy >= H) continue;

      int door_minx = W, door_maxx = -1;
      for (int xx = 0; xx < W; ++xx) {
        if (door_labels[cy * W + xx] == dlab) {
          if (xx < door_minx) door_minx = xx;
          if (xx > door_maxx) door_maxx = xx;
        }
      }
      if (door_maxx < door_minx) {
        door_minx = minx;
        door_maxx = maxx;
      }

      int xL = -1, xR = -1;
      for (int xx = door_minx; xx >= 0; --xx) {
        if (walls[cy * W + xx]) {
          xL = xx;
          break;
        }
      }
      for (int xx = door_maxx; xx < W; ++xx) {
        if (walls[cy * W + xx]) {
          xR = xx;
          break;
        }
      }
      if (xL < 0 || xR < 0 || xR <= xL) continue;

      for (int dy = -1; dy <= 1; ++dy) {
        int yy = cy + dy;
        if (yy < 0 || yy >= H) continue;
        for (int xx = xL; xx <= xR; ++xx) {
          if (walls[yy * W + xx]) door_wall_mask[(size_t)yy * W + xx] = 1;
        }
      }
    } else {
      int cx = (minx + maxx) / 2;
      if (cx < 0 || cx >= W) continue;

      int door_miny = H, door_maxy = -1;
      for (int yy = 0; yy < H; ++yy) {
        if (door_labels[yy * W + cx] == dlab) {
          if (yy < door_miny) door_miny = yy;
          if (yy > door_maxy) door_maxy = yy;
        }
      }
      if (door_maxy < door_miny) {
        door_miny = miny;
        door_maxy = maxy;
      }

      int yT = -1, yB = -1;
      for (int yy = door_miny; yy >= 0; --yy) {
        if (walls[yy * W + cx]) {
          yT = yy;
          break;
        }
      }
      for (int yy = door_maxy; yy < H; ++yy) {
        if (walls[yy * W + cx]) {
          yB = yy;
          break;
        }
      }
      if (yT < 0 || yB < 0 || yB <= yT) continue;

      for (int dx = -1; dx <= 1; ++dx) {
        int xx = cx + dx;
        if (xx < 0 || xx >= W) continue;
        for (int yy = yT; yy <= yB; ++yy) {
          if (walls[yy * W + xx]) door_wall_mask[(size_t)yy * W + xx] = 1;
        }
      }
    }
  }

  free(door_is_line);
  free(door_labels);

  /* RANSAC-based detection on curved door arcs */
  int ransac_total_arcs = 0;
  int ransac_door_arcs = 0;
  if (img.is_color) {
    Gray gimg;
    gimg.w = W;
    gimg.h = H;
    gimg.p = img.p;
    ransac_door_arcs = detect_doors_ransac(&gimg, walls, door_seed_arcs, W, H,
                                           &ransac_total_arcs);
  }

  /* Dilate door_wall_mask slightly so cut is a few pixels thick */
  uint8_t* door_wall_mask_dil = (uint8_t*)calloc(N, 1);
  if (!door_wall_mask_dil) die("OOM door_wall_mask_dil");
  total_bytes += N * sizeof(uint8_t);

  for (y = 0; y < H; ++y) {
    for (x = 0; x < W; ++x) {
      size_t idx = (size_t)y * W + x;
      if (!door_wall_mask[idx]) continue;
      for (int dy = -1; dy <= 1; ++dy) {
        int ny = y + dy;
        if (ny < 0 || ny >= H) continue;
        for (int dx = -1; dx <= 1; ++dx) {
          int nx = x + dx;
          if (nx < 0 || nx >= W) continue;
          door_wall_mask_dil[(size_t)ny * W + nx] = 1;
        }
      }
    }
  }
  free(door_wall_mask);
  door_wall_mask = door_wall_mask_dil;

  /* 2) interior vs outside */

  uint8_t* outside = (uint8_t*)calloc(N, 1);
  uint8_t* interior = (uint8_t*)calloc(N, 1);
  if (!outside || !interior) die("OOM outside/interior");
  total_bytes += 2 * N * sizeof(uint8_t);

  flood_outside(free_space, W, H, outside);

#pragma omp parallel for private(x)
  for (y = 0; y < H; ++y) {
    for (x = 0; x < W; ++x) {
      size_t i = (size_t)y * W + x;
      interior[i] = free_space[i] && !outside[i];
    }
  }

  int* labels_int = (int*)malloc(N * sizeof(int));
  if (!labels_int) die("OOM labels_int");
  total_bytes += N * sizeof(int);

  int num_int = cclabel(interior, W, H, 1, labels_int); /* 4-connected */

  Box* boxes = (Box*)malloc((num_int + 1) * sizeof(Box));
  double* comp_area = (double*)calloc(num_int + 1, sizeof(double));
  double* comp_fill_ratio = (double*)calloc(num_int + 1, sizeof(double));
  uint8_t* is_window = (uint8_t*)calloc(num_int + 1, 1);
  uint8_t* is_feature = (uint8_t*)calloc(num_int + 1, 1);
  uint8_t* valid_comp = (uint8_t*)calloc(num_int + 1, 1);
  if (!boxes || !comp_area || !comp_fill_ratio || !is_window || !is_feature ||
      !valid_comp)
    die("OOM comp arrays");

  total_bytes += (num_int + 1) * sizeof(Box);
  total_bytes += (num_int + 1) * sizeof(double) * 2;
  total_bytes += (num_int + 1) * sizeof(uint8_t) * 3;

  for (int lab = 1; lab <= num_int; ++lab) {
    boxes[lab].xmin = W;
    boxes[lab].xmax = -1;
    boxes[lab].ymin = H;
    boxes[lab].ymax = -1;
  }

  double largest_area = 0.0;

  for (y = 0; y < H; ++y) {
    for (x = 0; x < W; ++x) {
      int lab = labels_int[y * W + x];
      if (lab <= 0) continue;
      comp_area[lab] += 1.0;
      if (x < boxes[lab].xmin) boxes[lab].xmin = x;
      if (x > boxes[lab].xmax) boxes[lab].xmax = x;
      if (y < boxes[lab].ymin) boxes[lab].ymin = y;
      if (y > boxes[lab].ymax) boxes[lab].ymax = y;
    }
  }

  for (int lab = 1; lab <= num_int; ++lab) {
    if (comp_area[lab] <= 0.0) continue;
    double w = (double)(boxes[lab].xmax - boxes[lab].xmin + 1);
    double h = (double)(boxes[lab].ymax - boxes[lab].ymin + 1);
    double box_area = w * h;
    comp_fill_ratio[lab] = comp_area[lab] / box_area;
    if (comp_area[lab] > largest_area) largest_area = comp_area[lab];
  }

  double area_threshold = 0.5 * largest_area;

  int count_rooms = 0;
  int count_windows = 0;
  int count_features = 0;

  for (int lab = 1; lab <= num_int; ++lab) {
    if (comp_area[lab] < 5.0) continue;
    double w = (double)(boxes[lab].xmax - boxes[lab].xmin + 1);
    double h = (double)(boxes[lab].ymax - boxes[lab].ymin + 1);
    double box_area = w * h;
    double fill = comp_area[lab] / box_area;

    valid_comp[lab] = 1;

    if (comp_area[lab] >= area_threshold && fill >= MIN_ROOM_FILL_RATIO) {
      is_window[lab] = 0;
      is_feature[lab] = 0;
      count_rooms++;
    } else {
      double thinness = (w < h ? w / h : h / w);
      if (thinness < 0.2 && comp_area[lab] > 20.0) {
        is_window[lab] = 1;
        is_feature[lab] = 0;
        count_windows++;
      } else {
        is_window[lab] = 0;
        is_feature[lab] = 1;
        count_features++;
      }
    }
  }

  /* 3) Generate 3D mesh (two files: noRoof, withRoof) */

  char obj_noRoof[1024];
  char obj_withRoof[1024];
  snprintf(obj_noRoof, sizeof(obj_noRoof), "%s_noRoof.obj", objBase);
  snprintf(obj_withRoof, sizeof(obj_withRoof), "%s_withRoof.obj", objBase);

  uint8_t* walls_mesh = (uint8_t*)malloc(N);
  if (!walls_mesh) die("OOM walls_mesh");
  memcpy(walls_mesh, walls, N);

  /* Neglect red pixels extrusion (lines or arcs) */
  if (img.is_color && door_seed) {
    for (y = 0; y < H; ++y) {
      for (x = 0; x < W; ++x) {
        size_t idx = (size_t)y * W + x;
        if (door_seed[idx]) {
          walls_mesh[idx] = 0;
        }
      }
    }
  }

  ObjWriter ow_noRoof, ow_withRoof;
  obj_begin(&ow_noRoof, obj_noRoof);
  obj_begin(&ow_withRoof, obj_withRoof);

  /* Floor quad: bounding box of walls */
  int minx = W, miny = H, maxx = -1, maxy = -1;
  for (y = 0; y < H; ++y) {
    for (x = 0; x < W; ++x) {
      if (walls[y * W + x]) {
        if (x < minx) minx = x;
        if (x > maxx) maxx = x;
        if (y < miny) miny = y;
        if (y > maxy) maxy = y;
      }
    }
  }

  if (minx < maxx && miny < maxy) {
    int v1 = obj_add_vertex(&ow_noRoof, (double)minx, 0.0, (double)miny);
    int v2 = obj_add_vertex(&ow_noRoof, (double)maxx, 0.0, (double)miny);
    int v3 = obj_add_vertex(&ow_noRoof, (double)maxx, 0.0, (double)maxy);
    int v4 = obj_add_vertex(&ow_noRoof, (double)minx, 0.0, (double)maxy);
    obj_add_quad(&ow_noRoof, v1, v2, v3, v4);

    int v1b = obj_add_vertex(&ow_withRoof, (double)minx, 0.0, (double)miny);
    int v2b = obj_add_vertex(&ow_withRoof, (double)maxx, 0.0, (double)miny);
    int v3b = obj_add_vertex(&ow_withRoof, (double)maxx, 0.0, (double)maxy);
    int v4b = obj_add_vertex(&ow_withRoof, (double)minx, 0.0, (double)maxy);
    obj_add_quad(&ow_withRoof, v1b, v2b, v3b, v4b);
  }

  /* Wall extrusion */
  static const int dx4[4] = {1, -1, 0, 0};
  static const int dy4[4] = {0, 0, 1, -1};

  for (y = 0; y < H; ++y) {
    for (x = 0; x < W; ++x) {
      if (!walls_mesh[y * W + x]) continue;

      for (int k = 0; k < 4; ++k) {
        int nx = x + dx4[k];
        int ny = y + dy4[k];
        if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
        if (walls_mesh[ny * W + nx]) continue; /* interior edge */

        /* edge between (x,y) and (nx,ny) => wall face */
        double x0 = (double)x;
        double z0 = (double)y;
        double x1 = (double)nx;
        double z1 = (double)ny;

        if (k == 0) { /* +x */
          x0 = (double)(x + 1);
          x1 = (double)(x + 1);
          z0 = (double)y;
          z1 = (double)(y + 1);
        } else if (k == 1) { /* -x */
          x0 = (double)x;
          x1 = (double)x;
          z0 = (double)y;
          z1 = (double)(y + 1);
        } else if (k == 2) { /* +y */
          z0 = (double)(y + 1);
          z1 = (double)(y + 1);
          x0 = (double)x;
          x1 = (double)(x + 1);
        } else if (k == 3) { /* -y */
          z0 = (double)y;
          z1 = (double)y;
          x0 = (double)x;
          x1 = (double)(x + 1);
        }

        size_t idx = (size_t)y * W + x;
        int in_door = 0;
        if (door_wall_mask[idx]) {
          in_door = 1;
        } else {
          int nx2 = x + dx4[k];
          int ny2 = y + dy4[k];
          if (nx2 >= 0 && nx2 < W && ny2 >= 0 && ny2 < H) {
            size_t idx2 = (size_t)ny2 * W + nx2;
            if (door_wall_mask[idx2]) in_door = 1;
          }
        }

        double y_bottom = 0.0;
        double y_top = (double)HEIGHT_PX;
        if (in_door) {
          y_bottom = (double)DOOR_H_PX;
        }

        int v1 = obj_add_vertex(&ow_noRoof, x0, y_bottom, z0);
        int v2 = obj_add_vertex(&ow_noRoof, x1, y_bottom, z1);
        int v3 = obj_add_vertex(&ow_noRoof, x1, y_top, z1);
        int v4 = obj_add_vertex(&ow_noRoof, x0, y_top, z0);
        obj_add_quad(&ow_noRoof, v1, v2, v3, v4);

        int v1b = obj_add_vertex(&ow_withRoof, x0, y_bottom, z0);
        int v2b = obj_add_vertex(&ow_withRoof, x1, y_bottom, z1);
        int v3b = obj_add_vertex(&ow_withRoof, x1, y_top, z1);
        int v4b = obj_add_vertex(&ow_withRoof, x0, y_top, z0);
        obj_add_quad(&ow_withRoof, v1b, v2b, v3b, v4b);
      }
    }
  }

  /* Roof only in withRoof file */
  if (minx < maxx && miny < maxy) {
    int vr1 = obj_add_vertex(&ow_withRoof, (double)minx, (double)HEIGHT_PX,
                             (double)miny);
    int vr2 = obj_add_vertex(&ow_withRoof, (double)maxx, (double)HEIGHT_PX,
                             (double)miny);
    int vr3 = obj_add_vertex(&ow_withRoof, (double)maxx, (double)HEIGHT_PX,
                             (double)maxy);
    int vr4 = obj_add_vertex(&ow_withRoof, (double)minx, (double)HEIGHT_PX,
                             (double)maxy);
    obj_add_quad(&ow_withRoof, vr1, vr2, vr3, vr4);
  }

  obj_end(&ow_noRoof);
  obj_end(&ow_withRoof);

  /* Stats */
  size_t wall_pixels = 0;
  size_t interior_pixels = 0;
  size_t outside_pixels = 0;
  size_t door_seed_pixels = 0;
  size_t door_wall_pixels = 0;

  for (size_t i = 0; i < N; ++i) {
    if (walls[i]) wall_pixels++;
    if (outside[i]) outside_pixels++;
    if (interior[i]) interior_pixels++;
    if (door_seed[i]) door_seed_pixels++;
    if (door_wall_mask[i]) door_wall_pixels++;
  }

  double t_end = omp_get_wtime();
  double total_time = t_end - t_start;
  double time_per_pix = (N > 0) ? (total_time / (double)N) : 0.0;

  int num_procs = omp_get_num_procs();
  int max_threads = omp_get_max_threads();
  int actual_threads = max_threads;

  printf("===== IMAGE / FLOORPLAN SUMMARY =====\n");
  printf("Image size: %d x %d  =>  %zu pixels\n", W, H, N);
  printf("Otsu threshold: %d\n\n", thr);

  printf("--- Pixel category counts ---\n");
  printf("Wall pixels:               %zu\n", wall_pixels);
  printf("Interior free-space pixels:%zu\n", interior_pixels);
  printf("Outside free-space pixels: %zu\n", outside_pixels);
  printf("Door red-seed pixels:      %zu\n", door_seed_pixels);
  printf("RANSAC total arc candidates:   %d\n", ransac_total_arcs);
  printf("RANSAC arcs validated as doors:%d\n", ransac_door_arcs);
  printf("Door wall-mask pixels:     %zu\n\n", door_wall_pixels);

  printf("Area threshold for rooms (0.5 * largest room): %.0f pixels\n",
         area_threshold);
  printf("Minimum fill ratio for rooms:                   %.2f\n\n",
         MIN_ROOM_FILL_RATIO);

  printf(
      "--- Connected interior components (BFS/CCL 'BDF search output') ---\n");
  printf(
      "Total interior/feature components (rooms + windows + doors + other "
      "features): %d\n",
      num_int + door_segments);
  printf("Rooms (large solid components):                         %d\n",
         count_rooms);
  printf("Windows (thin small strips, e.g. glazing):              %d\n",
         count_windows);
  printf("Door segments (from red seeds):                         %d\n",
         door_segments);
  printf("Door arcs (RANSAC-based, validated):                    %d\n",
         ransac_door_arcs);
  printf("Other small features (arcs / tiny regions):             %d\n\n",
         count_features);

  printf("Per-component areas and fill ratios:\n");
  for (int lab = 1; lab <= num_int; ++lab) {
    if (!valid_comp[lab]) continue;
    /* can print per-room details here if desired */
  }

  printf("\n===== PARALLEL / PERFORMANCE INFO =====\n");
  printf("OpenMP reported OS processors:   %d\n", num_procs);
  printf("OpenMP max available threads:    %d\n", max_threads);
  printf("OpenMP threads actually used:    %d\n", actual_threads);
  printf("Total runtime (wall-clock):      %.6f seconds\n", total_time);
  printf("Average time per pixel:          %.9e seconds/pixel\n", time_per_pix);
  printf("Approx. peak data-array memory:  %.2f MB (%zu bytes)\n",
         (double)total_bytes / (1024.0 * 1024.0), total_bytes);

  printf("\n===== ASYMPTOTIC COMPLEXITY (THEORETICAL) =====\n");
  printf("Let N = W*H = number of pixels in the input image.\n");
  printf("Major steps:\n");
  printf("  - Thresholding and masks:      O(N)\n");
  printf("  - Flood fill outside (BFS):    O(N)\n");
  printf("  - Connected-component labeling:O(N)\n");
  printf(
      "  - Mesh generation:             O(N) (worst case, every pixel used)\n");
  printf("Overall time complexity:         O(N)\n");
  printf(
      "Overall space complexity:        O(N) for image + masks + labels.\n\n");

  printf(
      "Segmentation algorithm:  Otsu thresholding + BFS/CCL (4-connected).\n");
  printf(
      "Door detection algorithm: Connected red-door blobs (R>140, R>=G+30, "
      "R>=B+30)\n");
  printf(
      "  split into straight lines (for wall cuts) and curved arcs "
      "(RANSAC-based),\n");
  printf("  projected onto walls with 3x3 dilation for door openings.\n\n");

  printf(
      "Detected %d rooms, %d windows, %d other features, %d door segments. 3D "
      "models saved to:\n",
      count_rooms, count_windows, count_features, door_segments);
  printf("  %s_noRoof.obj\n", objBase);
  printf("  %s_withRoof.obj\n\n", objBase);

  /* cleanup */
  free(img.p);
  if (img.r) free(img.r);
  if (img.g) free(img.g);
  if (img.b) free(img.b);

  free(walls);
  free(walls_mesh);
  free(free_space);
  free(door_seed);
  free(door_seed_lines);
  free(door_seed_arcs);
  free(door_wall_mask);

  free(outside);
  free(interior);
  free(labels_int);
  free(boxes);
  free(comp_area);
  free(comp_fill_ratio);
  free(is_window);
  free(is_feature);
  free(valid_comp);

  return 0;
}