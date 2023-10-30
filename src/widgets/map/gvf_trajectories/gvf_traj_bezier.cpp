#include "gvf_traj_bezier.h"
#include <stdio.h>
#include <stdlib.h>

// Path to files where data is saved. Choose the path you like.
const char x_val[]  = "var/conf/gvf_parametric_curve_x_values.data";
const char y_val[]  = "var/conf/gvf_parametric_curve_y_values.data";
const char ks_val[] = "var/conf/gvf_parametric_curve_ctrl_values.data";

GVF_traj_bezier::GVF_traj_bezier(QString id, QList<float> param, QList<float> _phi,
                                 float wb, int order, QVector<int> *gvf_settings) :
  GVF_trajectory(id, gvf_settings)
{
  set_param(param, _phi, wb, order);
  generate_trajectory();
}

// 2D bezier trajectory (parametric representation)
void GVF_traj_bezier::genTraj()
{

  QList<QPointF> points;
  // 0 <= t <= n_segments of the spline
  float max_t = (n_seg > 0) ? n_seg : 1; // At least one spline must be defined
  // 100 pts per each segment
  float num_pts = n_seg * 100;
  float dt = max_t / num_pts;

  for (float t = 0; t <= max_t; t += dt) {
    points.append(eval_traj(t,0));
  }
  createTrajItem(points);
}


// 2D bezier GVF
void GVF_traj_bezier::genVField()
{

  FILE *file_x; FILE *file_y; FILE *file_ks;

  // Only draw the file when data is available
  int cont = 0;
  if ((file_x = fopen(x_val, "r")) != NULL) {
    cont++;
    fclose(file_x);
  }
  if ((file_y = fopen(y_val, "r")) != NULL) {
    cont++;
    fclose(file_y);
  }
  if ((file_ks = fopen(ks_val, "r")) != NULL) {
    cont++;
    fclose(file_ks);
  }

  if (cont == 3) {
    QList<QPointF> vxy_mesh;
    float xmin, xmax, ymin, ymax;
    xmin = xx[0]; xmax = xx[0]; ymin = yy[0]; ymax = yy[0];
    int n_pts;
    n_pts = (bez_order == 3) ? (3 * n_seg + 2) : 3 * (n_seg + 1) + 1;
    for (int k = 0; k < n_pts; k++) {
      xmax = (xx[k] > xmax) ? xx[k] : xmax;
      xmin = (xx[k] < xmin) ? xx[k] : xmin;
      ymax = (yy[k] > ymax) ? yy[k] : ymax;
      ymin = (yy[k] < ymin) ? yy[k] : ymin;
    }

    float bound_area = 20 * (xmax - xmin) * (ymax - ymin);
    emit DispatcherUi::get()->gvf_defaultFieldSettings(ac_id, round(bound_area), 60, 60);
    xy_mesh = meshGrid();

    foreach (const QPointF &point, xy_mesh) {

      float phix = point.x() - eval_traj(w,0).x(); // Normal component
      float phiy = point.y() - eval_traj(w,0).y(); // Normal Component
      float sigx = beta * eval_traj(w,1).x(); // Tangential Component
      float sigy = beta * eval_traj(w,1).y(); // Tangential Component
      float vx = sigx - kx * phix;
      float vy = sigy - ky * phiy;
      float norm = sqrt(pow(vx, 2) + pow(vy, 2));
      norm = (norm > 0) ? norm : 1; // Avoid division by zero
      vxy_mesh.append(QPointF(vx / norm, vy / norm));

    }
    createVFieldItem(xy_mesh, vxy_mesh);
  } else {
    fprintf(stderr, "Field cannot be created yet, waiting for complete data...\n");
  }

}


/////////////// PRIVATE FUNCTIONS ///////////////
void GVF_traj_bezier::set_param(QList<float> param, QList<float> _phi, float wb, int order)
{


  FILE *file_x; FILE *file_y; FILE *file_ks;
  bez_order = order;
  int k; int n_pts;

  
  // Write:
  if (param[0] < 0) {
    n_seg = -(int)param[0];
    n_pts = (bez_order == 3) ? (3 * n_seg + 2) : 3 * (n_seg + 1) + 1;
    if ((file_x = fopen(x_val, "w+")) != NULL) {
      for (k = 0; k < n_pts; k++) {
        fprintf(file_x, "%f ", param[k]);
      }
      fclose(file_x);
    }
  } else if (param[0] > 0) {
    n_seg = (int)param[0];
    n_pts = (bez_order == 3) ? (3 * n_seg + 2) : 3 * (n_seg + 1) + 1;
    if ((file_y = fopen(y_val, "w+")) != NULL) {
      for (k = 0; k < n_pts; k++) {
        fprintf(file_y, "%f ", param[k]);
      }
      fclose(file_y);
    }
  } else {
    if ((file_ks = fopen(ks_val, "w+")) != NULL) {
      fprintf(file_ks, "%f %f %f", param[1], param[2], param[3]);
      fclose(file_ks);
    }
  }

  float N_SEG;

  // Read:
  if ((file_x = fopen(x_val, "r")) != NULL) {
    fscanf(file_x, "%f ", &N_SEG);
    n_seg = -(int)N_SEG;
    n_pts = (bez_order == 3) ? (3 * n_seg + 2) : 3 * (n_seg + 1) + 1;
    for (k = 0; k < n_pts; k++) {
      fscanf(file_x, "%f ", &xx[k]);
    }
    fclose(file_x);
  }
  if ((file_y = fopen(y_val, "r")) != NULL) {
    fscanf(file_y, "%f ", &N_SEG);
    n_seg = (int)N_SEG;
    n_pts = (bez_order == 3) ? (3 * n_seg + 2) : 3 * (n_seg + 1) + 1;
    for (k = 0; k < n_pts; k++) {
      fscanf(file_y, "%f ", &yy[k]);
    }
    fclose(file_y);
  }
  if ((file_ks = fopen(ks_val, "r")) != NULL) {
    fscanf(file_ks, "%f", &kx);
    fscanf(file_ks, "%f", &ky);
    fscanf(file_ks, "%f", &beta);
    fclose(file_ks);
  }

  phi = QPointF(_phi[0], _phi[1]);   //TODO: Display error in GVF viewer??
  w = (beta > 0) ? wb / beta : wb;   // gvf_parametric_w = wb/beta (wb = w*beta)
}

// t is time, p is derivative order
QPointF GVF_traj_bezier::eval_traj(float t, int p){

  // Just in case w from telemetry is not between bounds
  if (t < 0.0) {
    t = 0.0;
  } else if (t >= n_seg) {
    t = (float)(n_seg);
  }

  int k = (int)t;

  /* Each Bézier spline must be evaluated between 0 and 1.
  Remove integer part, get fractional
  */
  t = t - k; // This is between 0 and 1
  
  // Third order (0->3 points). Fith order (0->5 points)
  k = (bez_order == 3) ? 3*k : 5*k;
  int aux = k;
  int n = bez_order;
  int nu; int l; int j;
  
  // Compute curve
  float fx = 0;
  float fy = 0;
  float berst;
  
  if(bez_order == 3){
    // p = 0 curve. p = 1 first deriv
    for(k = 0; k <= n - p; k++){
      for(nu = 0; nu <= p; nu++){
        berst = berstein_poly(t, k, n-p);
        fx += berst * powf((-1), nu) * binom(nu, p) * xx[k+p-nu+aux];
        fy += berst * powf((-1), nu) * binom(nu, p) * yy[k+p-nu+aux];
      }
    }
    for(l = 0; l <= p-1; l++){
      fx *= (n-l);
      fy *= (n-l);
    }
  }
  else{
    // If it is order 5 then create the good poimnts
    float px[n_seg][6];
    float py[n_seg][6];
    
    // Init first segment
    int which_seg = 0;
    for(j = 0; j <= 5; j++){
      px[which_seg][j] = xx[j]; // Segment zero, points
      py[which_seg][j] = yy[j];
    }
    // Now init rest of segments
	  j = 5;
	  for(which_seg = 1; which_seg < n_seg; which_seg++){
	    px[which_seg][0] = xx[j];
	    py[which_seg][0] = yy[j];
	    // p1 and p2 are fixed by continuity conditions
	    px[which_seg][3] = xx[j+1];
	    py[which_seg][3] = yy[j+1];
	    
      px[which_seg][4] = xx[j+2];
      py[which_seg][4] = yy[j+2];
      
      px[which_seg][5] = xx[j+3];
      py[which_seg][5] = yy[j+3];
	    j += 3;
	  }

    // Continuity conditions
    for(which_seg = 1; which_seg < n_seg; which_seg++){
      px[which_seg][1] =  2*px[which_seg-1][5] - px[which_seg-1][4];
      py[which_seg][1] =  2*py[which_seg-1][5] - py[which_seg-1][4];
      px[which_seg][2] =  4*px[which_seg-1][5] - 4*px[which_seg-1][4] + px[which_seg-1][3];
      py[which_seg][2] =  4*py[which_seg-1][5] - 4*py[which_seg-1][4] + py[which_seg-1][3];
    }
    
    // Now everything is created. compute the trajectory for the zth segment
    // Which segment? Take integer part from r
    n = bez_order;
    for(k = 0; k <= n - p; k++){
      for(nu = 0; nu <= p; nu++){
        berst = berstein_poly(t, k, n-p);
        fx += berst * powf((-1), nu) * binom(nu, p) * px[auxi][k+p-nu];
        fy += berst * powf((-1), nu) * binom(nu, p) * py[auxi][k+p-nu];
      }
    }
    for(l = 0; l <= p-1; l++){
      fx *= (n-l);
      fy *= (n-l);
    }
  
  }
  return QPointF(fx, fy);
}
// Compute the berstein polynomial
//b_{mu,n}(x) = (mu n) * x^mu * (1-x)^(n-mu)
float GVF_traj_bezier::berstein_poly(float t, int mu, int n){
  return binom(mu,n) * powf(t,mu) * powf(1-t, n - mu);
}

// Compute the binomial coefficient
float GVF_traj_bezier::binom(float mu, float n){
  int i;
  float fact_n = 1;
  float fact_mu = 1;
  float fact_mun = 1;
  for(i = 0; i < n; i++)
    fact_n *= (n-i);
  for(i = 0; i < mu; i++)
    fact_mu *= (mu - i);
  for(i = 0; i < (n-mu); i++)
    fact_mun *= (n-mu-i); 
  return fact_n/(fact_mu * fact_mun);
}


