#include "gvf_traj_rhomboid.h"

GVF_traj_rhomboid::GVF_traj_rhomboid(QString id, QList<float> param, int8_t _s, float _ke, QVector<int> *gvf_settings) :
    GVF_trajectory(id, gvf_settings)
{   
    set_param(param, _s, _ke);
    generate_trajectory();
    
}

void GVF_traj_rhomboid::set_param(QList<float> param, int8_t _s, float _ke) {
   // if (param.size()>3) { // gvf_rhomboid_wp()
   //     auto ac = pprzApp()->toolbox()->aircraftManager()->getAircraft(ac_id);
   //     Waypoint::WpFrame frame = ac->getFlightPlan()->getFrame();
   //     ac->getFlightPlan()->getWaypoint((uint8_t)param[3])->getRelative(frame, xy_off.rx(), xy_off.ry());

   // } else { // gvf_rhomboid_XY()
    xy_off = QPointF(param[0], param[1]);
    //}

    s = _s;
    ke = _ke;
    r = param[2]; // Pasarlo
}

// rhomboid trajectory (rotated standard rhomboid parametric representation)
void GVF_traj_rhomboid::genTraj() { 
    QList<QPointF> points;
    // This is working now
    float x_off[4] = {r, 0, -r, 0};
    float y_off[4] = {0, r, 0, -r};
    float dt = 0.01;
    int aux = 0;
    float x, y;
    for (int i = 0; i < 4; i++) {
      for(float k = 0; k <= 1; k += dt){
          x = xy_off.x() + (1 - k) * x_off[aux] + k * x_off[(aux + 1) % 4];
          y = xy_off.y() + (1 - k) * y_off[aux] + k * y_off[(aux + 1) % 4];
          points.append(QPointF(x,y));  
        }
        aux = (aux + 1) % 4;

    }

    createTrajItem(points);
}

// Let's do this!
// rhomboid GVF (standard rhomboid implicit function)
void GVF_traj_rhomboid::genVField() { 
    QList<QPointF> vxy_mesh; 

  
    float bound_area = 1200; // to scale the arrows

    emit DispatcherUi::get()->gvf_defaultFieldSettings(ac_id, round(bound_area), 50, 50);
    xy_mesh = meshGrid();
    
    // Rotate the space -alpha deg, draw the rhomboid vector field for each point in the mesh and then
    // rotate alpha to come back to the original space
    foreach (const QPointF &point, xy_mesh) {
        //float xel =  (point.x() - xy_off.x())*cos(-alpha) + (point.y() - xy_off.y())*sin(-alpha); 
        //float yel = -(point.x() - xy_off.x())*sin(-alpha) + (point.y() - xy_off.y())*cos(-alpha); 

        //float nx =  2*xel*cos(alpha)/pow(a,2) + 2*yel*sin(alpha)/pow(b,2);
        //float ny = -2*xel*sin(alpha)/pow(a,2) + 2*yel*cos(alpha)/pow(b,2);
        
        float g1 = ((point.x() - xy_off.x()) >= 0) ? 1 : -1;
        float g2 = ((point.y() - xy_off.y()) >= 0) ? 1 : -1;
        float nx = g1;
        float ny = g2;
        float tx =  s*ny;
        float ty = -s*nx;

        float e = fabs(point.x() - xy_off.x()) + fabs(point.y() - xy_off.y()) - r;

        float vx = tx - ke*e*nx;
        float vy = ty - ke*e*ny;

        float norm = sqrt(pow(vx,2) + pow(vy,2));
        if(norm == 0)
          norm = 1;
        vxy_mesh.append(QPointF(vx/norm, vy/norm));
    }
  
    createVFieldItem(xy_mesh, vxy_mesh);
    
}
