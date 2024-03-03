#ifndef GVF_TRAJ_SQUARE_H
#define GVF_TRAJ_SQUARE_H

#include "gvf_trajectory.h"

class GVF_traj_square : public GVF_trajectory
{
    Q_OBJECT
public:
    explicit GVF_traj_square(QString id, QList<float> param, int8_t _s, float _ke, QVector<int> *gvf_settings);

protected:
    virtual void genTraj() override;
    virtual void genVField() override;

private:
    void set_param(QList<float> param, int8_t _s, float _ke);

    float r;

    int8_t s;
    float ke;
};

#endif // GVF_TRAJ_square_H
