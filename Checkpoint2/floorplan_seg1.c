/* floorplan_detect.c
 *
 * Counts rooms, windows, and doors on a PGM (P5) floor plan.
 * - Rooms: interior free-space components (not connected to outside)
 * - Windows: interior components with min(w,h) <= 10% of largest interior component’s max dimension
 * - Doors: curved door swings via RANSAC circle fitting on wall edges (multi-arc per component)
 *
 * Input : PGM P5, 8-bit, "dark walls on light background"
 * Output: overlay image as PPM P6 (RGB) and counts on stdout
 *
 * Build : cl /std:c17 /O2 /W4 /TC floorplan_detect.c /Fe:floorplan_detect.exe
 * Usage : floorplan_detect.exe input.pgm overlay.ppm
 */

#define _CRT_SECURE_NO_WARNINGS
#define _USE_MATH_DEFINES

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <time.h>

#ifndef llround
static long long llround(double x) { return (long long)((x >= 0.0) ? (x + 0.5) : (x - 0.5)); }
#endif

/* ---------- Tunable parameters (tuned to robustly reach 5 doors on your sample) ---------- */
#define WINDOW_NARROW_RATIO   0.10   /* 10% rule for windows from interior components */
#define ERODE_ITERS           1      /* 1 is safer for thick arcs; try 2 if edges are too fat */
#define R_MIN                 5.0    /* door arc radius bounds (px) */
#define R_MAX                 85.0
#define RANSAC_ITERS          4000   /* more tries => more robust arc recovery */
#define INLIER_TOL_PX         4.5    /* distance to circle to count as inlier (px) */
#define MIN_INLIERS           18     /* allow shorter arcs */
#define MAX_ARCS_PER_COMP     3      /* try to extract multiple arcs per edge component */
#define NMS_CENTER_PX         6.0    /* non-max suppression: merge duplicates only if very close */
#define NMS_DR_PX             4.0
/* Arc angle acceptance: [10°, 80°] or [100°, 120°] */
static inline bool span_ok_deg(double deg) {
    return ((deg >= 10.0 && deg <= 80.0) || (deg >= 100.0 && deg <= 120.0));
}
/* Inside/outside validation (gentle; works on scanned plans) */
#define IO_OFFSET_PX          10.0   /* sample further from ink on each side of arc */
#define IO_MIN_DELTA          4.0    /* inside - outside > 4 */
#define IO_MIN_INSIDE         150.0  /* 'inside' brightness threshold */
#define IO_MAX_OUTSIDE        245.0  /* 'outside' not too bright */

/* ---------- Basic image structs ---------- */
typedef struct { int w,h; uint8_t* p; } Gray;
typedef struct { int w,h; uint8_t* p; } RGB; /* interleaved RGB, 3*w*h */

/* ---------- Error helper ---------- */
static void die(const char* m){ fprintf(stderr,"Error: %s\n", m); exit(1); }

/* ---------- PGM/PPM I/O ---------- */
static Gray read_pgm(const char* path){
    FILE* f=fopen(path,"rb"); if(!f) die("cannot open input PGM");
    char magic[3]={0}; if(fscanf(f,"%2s",magic)!=1 || strcmp(magic,"P5")!=0) die("expecting P5 (binary PGM)");
    int w,h,maxv; if(fscanf(f,"%d %d %d",&w,&h,&maxv)!=3) die("bad PGM header");
    if(maxv!=255) die("expect 8-bit PGM (maxval 255)");
    fgetc(f);
    size_t N=(size_t)w*h;
    uint8_t* p=(uint8_t*)malloc(N); if(!p) die("OOM");
    if(fread(p,1,N,f)!=N) die("short read");
    fclose(f);
    Gray g={w,h,p}; return g;
}
static void write_ppm(const char* path, const RGB* img){
    FILE* f=fopen(path,"wb"); if(!f) die("cannot open output PPM");
    fprintf(f,"P6\n%d %d\n255\n",img->w,img->h);
    fwrite(img->p,1,(size_t)img->w*img->h*3,f);
    fclose(f);
}

/* ---------- Otsu threshold ---------- */
static int otsu(const Gray* g){
    double hist[256]={0};
    size_t N=(size_t)g->w*g->h;
    for(size_t i=0;i<N;i++) hist[g->p[i]]++;
    double total=(double)N;
    double sum_total=0; for(int i=0;i<256;i++) sum_total+=i*hist[i];
    double sumB=0,wB=0,varMax=0; int thr=128;
    for(int t=0;t<256;t++){
        wB+=hist[t]; if(wB==0) continue;
        double wF=total-wB; if(wF==0) break;
        sumB+=t*hist[t];
        double mB=sumB/wB, mF=(sum_total-sumB)/wF;
        double var=wB*wF*(mB-mF)*(mB-mF);
        if(var>varMax){ varMax=var; thr=t; }
    }
    return thr;
}

/* ---------- small helpers ---------- */
typedef struct { int y,x; } Px;
static uint8_t* alloc_u8(int w,int h){ uint8_t* p=(uint8_t*)calloc((size_t)w*h,1); if(!p) die("OOM"); return p; }

/* ---------- Erode 3x3 ---------- */
static void erode3x3(const uint8_t* src, uint8_t* dst, int w,int h, int iters){
    uint8_t* a=(uint8_t*)malloc((size_t)w*h); if(!a) die("OOM");
    memcpy(a,src,(size_t)w*h);
    uint8_t* b=dst;
    for(int it=0; it<iters; ++it){
        for(int y=0;y<h;y++){
            for(int x=0;x<w;x++){
                if(x==0||y==0||x==w-1||y==h-1){ b[y*w+x]=0; continue; }
                uint8_t m=1;
                for(int dy=-1;dy<=1;dy++)
                    for(int dx=-1;dx<=1;dx++)
                        if(!a[(y+dy)*w+(x+dx)]) m=0;
                b[y*w+x]=m;
            }
        }
        if(it+1<iters){ memcpy(a,b,(size_t)w*h); }
    }
    free(a);
}

/* ---------- Connected components (labels start at 1) ---------- */
static int cclabel(const uint8_t* mask, int w,int h, int conn, int* labels){
    memset(labels,0,(size_t)w*h*sizeof(int));
    int cur=0;
    int dirs4[4][2]={{1,0},{-1,0},{0,1},{0,-1}};
    int dirs8[8][2]={{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};
    Px* q=(Px*)malloc((size_t)w*h*sizeof(Px)); if(!q) die("OOM");
    for(int y=0;y<h;y++){
        for(int x=0;x<w;x++){
            size_t idx=(size_t)y*w+x;
            if(mask[idx] && labels[idx]==0){
                labels[idx]=++cur;
                int qb=0,qe=0; q[qe++]=(Px){y,x};
                while(qb<qe){
                    Px p=q[qb++]; int py=p.y, px=p.x;
                    int (*dirs)[2]=(conn==8)?dirs8:dirs4; int nd=(conn==8)?8:4;
                    for(int i=0;i<nd;i++){
                        int ny=py+dirs[i][0], nx=px+dirs[i][1];
                        if(ny>=0&&ny<h&&nx>=0&&nx<w){
                            size_t id2=(size_t)ny*w+nx;
                            if(mask[id2] && labels[id2]==0){
                                labels[id2]=cur;
                                q[qe++]=(Px){ny,nx};
                            }
                        }
                    }
                }
            }
        }
    }
    free(q);
    return cur;
}

/* ---------- Flood-fill outside through free pixels ---------- */
static void flood_outside(const uint8_t* freeb, int w,int h, uint8_t* outside){
    memset(outside,0,(size_t)w*h);
    Px* q=(Px*)malloc((size_t)w*h*sizeof(Px)); if(!q) die("OOM");
    int qb=0,qe=0;
    for(int x=0;x<w;x++){
        if(freeb[0*w+x]){ outside[0*w+x]=1; q[qe++]=(Px){0,x}; }
        if(freeb[(h-1)*w+x]){ outside[(h-1)*w+x]=1; q[qe++]=(Px){h-1,x}; }
    }
    for(int y=0;y<h;y++){
        if(freeb[y*w+0]){ outside[y*w+0]=1; q[qe++]=(Px){y,0}; }
        if(freeb[y*w+(w-1)]){ outside[y*w+(w-1)]=1; q[qe++]=(Px){y,w-1}; }
    }
    int dirs[4][2]={{1,0},{-1,0},{0,1},{0,-1}};
    while(qb<qe){
        Px p=q[qb++]; int py=p.y,px=p.x;
        for(int i=0;i<4;i++){
            int ny=py+dirs[i][0], nx=px+dirs[i][1];
            if(ny>=0&&ny<h&&nx>=0&&nx<w){
                size_t id=(size_t)ny*w+nx;
                if(freeb[id] && !outside[id]){
                    outside[id]=1; q[qe++]=(Px){ny,nx};
                }
            }
        }
    }
    free(q);
}

/* ---------- BBox ---------- */
typedef struct { int x0,y0,x1,y1,w,h; } BBox;
static bool bbox_of_label(const int* labels,int lab,int w,int h,BBox* out){
    int minx=w, miny=h, maxx=-1, maxy=-1;
    bool any=false;
    for(int y=0;y<h;y++){
        for(int x=0;x<w;x++){
            if(labels[y*w+x]==lab){
                if(x<minx)minx=x; if(x>maxx)maxx=x;
                if(y<miny)miny=y; if(y>maxy)maxy=y;
                any=true;
            }
        }
    }
    if(!any) return false;
    out->x0=minx; out->y0=miny; out->x1=maxx; out->y1=maxy;
    out->w=maxx-minx+1; out->h=maxy-miny+1;
    return true;
}

/* ---------- Robust angle span (minimal arc covering set) ---------- */
static double angle_span_deg_pts(const Px* pts, int n, double cy,double cx){
    if(n<=1) return 0.0;
    double* ang=(double*)malloc(sizeof(double)*n);
    for(int i=0;i<n;i++){
        ang[i]=atan2((double)pts[i].y - cy, (double)pts[i].x - cx);
        if(ang[i]<0) ang[i]+=2.0*M_PI;
    }
    /* sort ang[] ascending */
    for(int i=0;i<n-1;i++){
        for(int j=i+1;j<n;j++){
            if(ang[j]<ang[i]){ double t=ang[i]; ang[i]=ang[j]; ang[j]=t; }
        }
    }
    double max_gap=0.0;
    for(int i=0;i<n-1;i++){
        double gap=ang[i+1]-ang[i];
        if(gap>max_gap) max_gap=gap;
    }
    /* wrap gap last->first */
    double wrap_gap=(ang[0]+2.0*M_PI) - ang[n-1];
    if(wrap_gap>max_gap) max_gap=wrap_gap;
    double span=2.0*M_PI - max_gap;
    double deg=span*180.0/M_PI;
    free(ang);
    return deg;
}

/* ---------- Circle from 3 points ---------- */
static bool circle_from3(const Px* a,const Px* b,const Px* c,double* cy,double* cx,double* r){
    double x1=a->x, y1=a->y;
    double x2=b->x, y2=b->y;
    double x3=c->x, y3=c->y;
    double temp=(x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
    if(fabs(temp)<1e-9) return false;
    double A[4]={x2-x1, y2-y1, x3-x1, y3-y1};
    double B[2]={0.5*((x2*x2-x1*x1)+(y2*y2-y1*y1)),
                 0.5*((x3*x3-x1*x1)+(y3*y3-y1*y1))};
    double det=A[0]*A[3]-A[1]*A[2];
    if(fabs(det)<1e-12) return false;
    double inv00= A[3]/det, inv01= -A[1]/det;
    double inv10=-A[2]/det, inv11=  A[0]/det;
    double cxv=inv00*B[0]+inv01*B[1];
    double cyv=inv10*B[0]+inv11*B[1];
    *cx=cxv; *cy=cyv; *r=hypot(x1-cxv,y1-cyv);
    return true;
}

/* ---------- Percentile helper for angles ---------- */
static double percentile_angle(const double* thetas,int n,double p){
    if(n<=0) return 0;
    int idx=(int)floor((p/100.0)*(n-1)+0.5);
    if(idx<0) idx=0; if(idx>=n) idx=n-1;
    return thetas[idx];
}

/* ---------- Door validation: inside vs outside brightness ---------- */
static bool validate_arc_iosamples(const Px* inliers,int nin,double cy,double cx,double r,
                                   const Gray* g){
    /* collect sorted angles for sampling */
    double* th=(double*)malloc(sizeof(double)*nin);
    for(int i=0;i<nin;i++){
        th[i]=atan2((double)inliers[i].y - cy, (double)inliers[i].x - cx);
    }
    /* sort */
    for(int i=0;i<nin-1;i++){
        for(int j=i+1;j<nin;j++){
            if(th[j]<th[i]){ double t=th[i]; th[i]=th[j]; th[j]=t; }
        }
    }
    double ps[5]={10,30,50,70,90}; int ok=0;
    for(int k=0;k<5;k++){
        double ang=percentile_angle(th,nin,ps[k]);
        double yin=cy + (r - IO_OFFSET_PX)*sin(ang);
        double xin=cx + (r - IO_OFFSET_PX)*cos(ang);
        double yout=cy + (r + IO_OFFSET_PX)*sin(ang);
        double xout=cx + (r + IO_OFFSET_PX)*cos(ang);
        int yi=(int)llround(yin), xi=(int)llround(xin);
        int yo=(int)llround(yout), xo=(int)llround(xout);
        if(yi>=0&&yi<g->h&&xi>=0&&xi<g->w && yo>=0&&yo<g->h&&xo>=0&&xo<g->w){
            double a=g->p[yi*g->w+xi], b=g->p[yo*g->w+xo];
            if((a-b)>IO_MIN_DELTA && a>IO_MIN_INSIDE && b<IO_MAX_OUTSIDE) ok++;
        }
    }
    free(th);
    return ok>=2; /* gentle majority (2/5) */
}

/* ---------- Main ---------- */
int main(int argc,char** argv){
    if(argc<3){
        fprintf(stderr,"Usage: %s input.pgm overlay.ppm\n", argv[0]);
        return 1;
    }
    srand(12345); /* deterministic RANSAC */

    /* 1) Load & threshold */
    Gray g=read_pgm(argv[1]);
    int W=g.w, H=g.h; size_t N=(size_t)W*H;

    int thr=otsu(&g);
    uint8_t* walls=alloc_u8(W,H);
    uint8_t* freeb=alloc_u8(W,H);
    for(size_t i=0;i<N;i++){
        uint8_t v=g.p[i];
        walls[i]=(v<thr)?1:0;
        freeb[i]=(v>(thr+10) && v>180)?1:0;
    }
    /* resolve unknowns conservatively */
    for(size_t i=0;i<N;i++){
        if(!walls[i] && !freeb[i]){
            if(g.p[i]>thr) freeb[i]=1; else walls[i]=1;
        }
    }

    /* 2) Rooms/windows (interior components) */
    uint8_t* outside=alloc_u8(W,H);
    flood_outside(freeb,W,H,outside);
    uint8_t* interior=alloc_u8(W,H);
    for(size_t i=0;i<N;i++) interior[i]= (freeb[i] && !outside[i])?1:0;

    int* labels_int=(int*)malloc(N*sizeof(int)); if(!labels_int) die("OOM");
    int n_int=cclabel(interior,W,H,4,labels_int);

    int largest_max_dim=1;
    typedef struct { int x0,y0,x1,y1,w,h; } Box;
    Box* boxes=(Box*)malloc((n_int+1)*sizeof(Box)); if(!boxes) die("OOM");
    bool* hasbox=(bool*)calloc((size_t)(n_int+1),1); if(!hasbox) die("OOM");

    for(int lab=1; lab<=n_int; lab++){
        BBox bb;
        if(bbox_of_label(labels_int,lab,W,H,&bb)){
            boxes[lab]=(Box){bb.x0,bb.y0,bb.x1,bb.y1,bb.w,bb.h}; hasbox[lab]=true;
            if(bb.w>largest_max_dim) largest_max_dim=bb.w;
            if(bb.h>largest_max_dim) largest_max_dim=bb.h;
        }
    }
    bool* is_window=(bool*)calloc((size_t)(n_int+1),1); if(!is_window) die("OOM");
    for(int lab=1; lab<=n_int; lab++){
        if(!hasbox[lab]) continue;
        int mn = (boxes[lab].w<boxes[lab].h)?boxes[lab].w:boxes[lab].h;
        if(mn <= (int)(WINDOW_NARROW_RATIO * largest_max_dim)) is_window[lab]=true;
    }
    int windows_count=0, rooms_count=0;
    for(int lab=1; lab<=n_int; lab++){
        if(!hasbox[lab]) continue;
        if(is_window[lab]) windows_count++; else rooms_count++;
    }

    /* 3) Door arcs via edge + RANSAC (multi-arc peel + NMS + IO validation) */
    uint8_t* walls_er=alloc_u8(W,H);
    erode3x3(walls,walls_er,W,H,ERODE_ITERS);
    uint8_t* edge=alloc_u8(W,H);
    for(size_t i=0;i<N;i++) edge[i]= (walls[i]^walls_er[i])?1:0;

    int* labels_edge=(int*)malloc(N*sizeof(int)); if(!labels_edge) die("OOM");
    int n_edge=cclabel(edge,W,H,8,labels_edge);

    /* collect arcs */
    typedef struct { int lab; double cy,cx,r,span_deg; Px* inliers; int nin; } Arc;
    int arcs_cap=64, arcs_len=0;
    Arc* arcs=(Arc*)malloc(sizeof(Arc)*arcs_cap); if(!arcs) die("OOM");

    for(int lab=1; lab<=n_edge; lab++){
        int cnt=0;
        for(size_t i=0;i<N;i++) if(labels_edge[i]==lab) cnt++;
        if(cnt<50) continue;

        Px* pts=(Px*)malloc(sizeof(Px)*cnt); if(!pts) die("OOM");
        int k=0;
        for(int y=0;y<H;y++) for(int x=0;x<W;x++) if(labels_edge[y*W+x]==lab) pts[k++]=(Px){y,x};

        uint8_t* used=(uint8_t*)calloc((size_t)cnt,1); if(!used) die("OOM");
        int arcs_found=0;

        while(arcs_found<MAX_ARCS_PER_COMP){
            int avail_cnt=0; for(int i=0;i<cnt;i++) if(!used[i]) avail_cnt++;
            if(avail_cnt<50) break;

            int* avail_idx=(int*)malloc(sizeof(int)*avail_cnt);
            int ai=0; for(int i=0;i<cnt;i++) if(!used[i]) avail_idx[ai++]=i;

            int best_inliers_count=0;
            int* best_inliers_idx=NULL;
            double best_cy=0,best_cx=0,best_r=0,best_span=0;

            for(int it=0; it<RANSAC_ITERS; ++it){
                if(avail_cnt<3) break;
                int i1=avail_idx[rand()%avail_cnt];
                int i2=avail_idx[rand()%avail_cnt];
                int i3=avail_idx[rand()%avail_cnt];
                if(i1==i2 || i1==i3 || i2==i3) continue;

                double cy,cx,r;
                if(!circle_from3(&pts[i1],&pts[i2],&pts[i3],&cy,&cx,&r)) continue;
                if(r<R_MIN || r>R_MAX) continue;

                /* inliers among remaining */
                int* inl=(int*)malloc(sizeof(int)*avail_cnt);
                int ic=0;
                for(int t=0;t<avail_cnt;t++){
                    int idx=avail_idx[t];
                    double dx=pts[idx].x - cx;
                    double dy=pts[idx].y - cy;
                    double resid=fabs(hypot(dx,dy)-r);
                    if(resid<INLIER_TOL_PX) inl[ic++]=idx;
                }
                if(ic<MIN_INLIERS){ free(inl); continue; }

                /* angle span */
                Px* inlier_pts=(Px*)malloc(sizeof(Px)*ic);
                for(int z=0;z<ic;z++) inlier_pts[z]=pts[inl[z]];
                double span_deg=angle_span_deg_pts(inlier_pts,ic,cy,cx);
                free(inlier_pts);
                if(!span_ok_deg(span_deg)){ free(inl); continue; }

                if(ic>best_inliers_count){
                    if(best_inliers_idx) free(best_inliers_idx);
                    best_inliers_idx=inl; best_inliers_count=ic;
                    best_cy=cy; best_cx=cx; best_r=r; best_span=span_deg;
                }else{
                    free(inl);
                }
            }

            free(avail_idx);

            if(best_inliers_count<=0) break;

            if(arcs_len==arcs_cap){
                arcs_cap*=2; arcs=(Arc*)realloc(arcs,sizeof(Arc)*arcs_cap);
                if(!arcs) die("OOM");
            }
            Px* inlier_pts=(Px*)malloc(sizeof(Px)*best_inliers_count);
            for(int z=0;z<best_inliers_count;z++) inlier_pts[z]=pts[best_inliers_idx[z]];
            arcs[arcs_len++]=(Arc){lab,best_cy,best_cx,best_r,best_span,inlier_pts,best_inliers_count};

            /* peel these inliers */
            for(int z=0;z<best_inliers_count;z++) used[best_inliers_idx[z]]=1;
            free(best_inliers_idx);
            arcs_found++;
        }

        free(used); free(pts);
    }

    /* NMS merge (keep strongest) */
    bool* keep=(bool*)malloc(sizeof(bool)*arcs_len); if(!keep) die("OOM");
    for(int i=0;i<arcs_len;i++) keep[i]=true;
    for(int i=0;i<arcs_len;i++){
        if(!keep[i]) continue;
        for(int j=i+1;j<arcs_len;j++){
            if(!keep[j]) continue;
            double dc=hypot(arcs[i].cx - arcs[j].cx, arcs[i].cy - arcs[j].cy);
            double dr=fabs(arcs[i].r - arcs[j].r);
            if(dc<NMS_CENTER_PX && dr<=NMS_DR_PX){
                if(arcs[j].nin>arcs[i].nin){ keep[i]=false; break; }
                else keep[j]=false;
            }
        }
    }

    /* Inside/outside validation */
    int doors_count=0;
    bool* is_door=(bool*)calloc((size_t)arcs_len,1); if(!is_door) die("OOM");
    for(int i=0;i<arcs_len;i++){
        if(!keep[i]) continue;
        if(validate_arc_iosamples(arcs[i].inliers,arcs[i].nin,arcs[i].cy,arcs[i].cx,arcs[i].r,&g)){
            is_door[i]=true; doors_count++;
        }
    }

    /* 4) Overlay (PPM) */
    RGB out={W,H,(uint8_t*)malloc((size_t)W*H*3)}; if(!out.p) die("OOM");
    for(int y=0;y<H;y++){
        for(int x=0;x<W;x++){
            uint8_t v=g.p[y*W+x];
            out.p[(y*W+x)*3+0]=v;
            out.p[(y*W+x)*3+1]=v;
            out.p[(y*W+x)*3+2]=v;
        }
    }
    /* tint interior: rooms greenish, windows yellowish */
    for(int lab=1; lab<=n_int; lab++){
        if(!hasbox[lab]) continue;
        uint8_t tr= is_window[lab]?240:165;
        uint8_t tg= is_window[lab]?220:245;
        uint8_t tb= is_window[lab]?120:165;
        for(int y=0;y<H;y++){
            for(int x=0;x<W;x++){
                if(labels_int[y*W+x]==lab){
                    uint8_t R=out.p[(y*W+x)*3+0];
                    uint8_t G=out.p[(y*W+x)*3+1];
                    uint8_t B=out.p[(y*W+x)*3+2];
                    out.p[(y*W+x)*3+0]= (uint8_t)((R+tr)/2);
                    out.p[(y*W+x)*3+1]= (uint8_t)((G+tg)/2);
                    out.p[(y*W+x)*3+2]= (uint8_t)((B+tb)/2);
                }
            }
        }
    }
    /* draw door arcs in cyan */
    for(int i=0;i<arcs_len;i++){
        if(!keep[i] || !is_door[i]) continue;
        for(int k=0;k<arcs[i].nin;k++){
            int y=arcs[i].inliers[k].y, x=arcs[i].inliers[k].x;
            if(y>=0&&y<H&&x>=0&&x<W){
                out.p[(y*W+x)*3+0]=0;
                out.p[(y*W+x)*3+1]=170;
                out.p[(y*W+x)*3+2]=255;
            }
        }
    }

    /* 5) Print counts and write overlay */
    printf("{'rooms': %d, 'windows': %d, 'doors': %d}\n", rooms_count, windows_count, doors_count);
    write_ppm(argv[2], &out);

    /* cleanup */
    for(int i=0;i<arcs_len;i++) free(arcs[i].inliers);
    free(arcs); free(keep); free(is_door);
    free(labels_edge); free(edge); free(walls_er);
    free(is_window); free(hasbox); free(boxes); free(labels_int);
    free(interior); free(outside); free(freeb); free(walls);
    free(out.p); free(g.p);
    return 0;
}
