#include "gvf_traj_pnorm.h"

GVF_traj_pnorm::GVF_traj_pnorm(QString id, QList<float> param, int8_t _s, float _ke, QVector<int> *gvf_settings) :
    GVF_trajectory(id, gvf_settings)
{   
    set_param(param, _s, _ke);
    generate_trajectory();
    
}

void GVF_traj_pnorm::set_param(QList<float> param, int8_t _s, float _ke) {
   // if (param.size()>3) { // gvf_pnorm_wp()
   //     auto ac = pprzApp()->toolbox()->aircraftManager()->getAircraft(ac_id);
   //     Waypoint::WpFrame frame = ac->getFlightPlan()->getFrame();
   //     ac->getFlightPlan()->getWaypoint((uint8_t)param[3])->getRelative(frame, xy_off.rx(), xy_off.ry());

   // } else { // gvf_pnorm_XY()
    xy_off = QPointF(param[0], param[1]);
    //}

    s = _s;
    ke = _ke;
    r = param[2];       // Pasarlo
    lp_norm = param[3]; // The p norm
}

// pnorm trajectory (standard pnorm parametric representation)
void GVF_traj_pnorm::genTraj() { 
    QList<QPointF> points;

    int Niter = 100;
    float dt = 2.0 * M_PI / (float)Niter;
    float x, y;
    float sc, ss;
    for(int k = 0; k < Niter; k++){
        sc = (cosf(k * dt) > 0) ? 1 : ( (cosf (k * dt) < 0) ? -1 : 0);
        ss = (sinf(k * dt) > 0) ? 1 : ( (sinf (k * dt) < 0) ? -1 : 0);
        x = powf(fabs(cosf(k * dt)), 2.0/lp_norm) * sc * r + xy_off.x();
        y = powf(fabs(sinf(k * dt)), 2.0/lp_norm) * ss * r + xy_off.y();
        points.append(QPointF(x,y));
    }
    createTrajItem(points);
}

// Let's do this!
// pnorm GVF (standard pnorm implicit function)
void GVF_traj_pnorm::genVField() { 
    QList<QPointF> vxy_mesh; 

  
    float bound_area = 1200; // to scale the arrows

    emit DispatcherUi::get()->gvf_defaultFieldSettings(ac_id, round(bound_area), 50, 50);
    xy_mesh = meshGrid();
    
    foreach (const QPointF &point, xy_mesh) {
        float px = point.x();
        float py = point.y();
        float wx = xy_off.x();
        float wy = xy_off.y();
        float dx = powf(fabs(px-wx),lp_norm);
        float dy = powf(fabs(py-wy),lp_norm);
        float dr = powf(dx + dy, 1.0/lp_norm);        

        float nx, ny;

        nx = powf( fabs(px-wx), lp_norm - 2.0) * (px - wx);
        ny = powf( fabs(py-wy), lp_norm - 2.0) * (py - wy);

        float tx =  s*ny;
        float ty = -s*nx;

        float e = dr - r;

        float vx = tx - ke*e*nx;
        float vy = ty - ke*e*ny;

        float norm = sqrtf(pow(vx,2) + pow(vy,2));
        if(norm == 0)
          norm = 1;
        vxy_mesh.append(QPointF(vx/norm, vy/norm));
    }
  
    createVFieldItem(xy_mesh, vxy_mesh);
    
}
