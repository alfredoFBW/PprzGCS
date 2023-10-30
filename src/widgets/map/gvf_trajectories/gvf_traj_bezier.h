#ifndef GVF_TRAJ_BEZIER_H
#define GVF_TRAJ_BEZIER_H

#include "gvf_trajectory.h"

class GVF_traj_bezier : public GVF_trajectory
{
  Q_OBJECT
public:
  explicit GVF_traj_bezier(QString id, QList<float> param, QList<float> _phi,
                           float wb, int order, QVector<int> *gvf_settings);

protected:
  virtual void genTraj() override;
  virtual void genVField() override;

private:
  void set_param(QList<float> param, QList<float> _phi, float wb, int order); // GVF PARAMETRIC
  QPointF eval_traj(float t, int p);
  float berstein_poly(float t, int mu, int n);
  float binom(float mu, float n);
  
  // Maximum data size from gvf_parametric is 16 elements. But sometimes we need more
  float xx[32];
  float yy[32];
  int bez_order;
  int n_seg;
  float w;
  float kx;
  float ky;
  float beta;
  QPointF phi;
};

#endif // GVF_TRAJ_bezier_H
