#ifndef GVF_TRAJ_PNORM_H
#define GVF_TRAJ_PNORM_H

#include "gvf_trajectory.h"


class GVF_traj_pnorm : public GVF_trajectory
{
    Q_OBJECT
public:
    explicit GVF_traj_pnorm(QString id, QList<float> param, int8_t _s, float _ke, QVector<int> *gvf_settings);

protected:
    virtual void genTraj() override;
    virtual void genVField() override;

private:
    void set_param(QList<float> param, int8_t _s, float _ke);

    float r;

    int8_t s;
    float ke;
    float lp_norm;
};

#endif // GVF_TRAJ_pnorm_H
