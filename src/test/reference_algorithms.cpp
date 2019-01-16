#include <Eigen/Core>

#include "multirotor_sim/ephemeris.h"
#include "multirotor_sim/gtime.h"
#include "multirotor_sim/datetime.h"
#include "multirotor_sim/test_common.h"

using namespace Eigen;

#define SQR(x) (x*x)

void eph2pos(const GTime& t, const Ephemeris* eph, Vector3d& pos, double* dts)
{
    // From RTKLIB eph2pos() in ephemeris.c
    static const int MAX_ITER_KEPLER = 30;
    static const double RTOL_KEPLER = 1E-13;
    static const double MU_GPS = 3.9860050E14;
    static const double OMGE = 7.2921151467E-5;
    static const double CLIGHT = 299792458.0;

    double tk,M,E,Ek,sinE,cosE,u,r,i,O,sin2u,cos2u,x,y,sinO,cosO,cosi,mu,omge;
    double xg,yg,zg,sino,coso;
    int n,sys,prn;

//    trace(4,"eph2pos : time=%s sat=%2d\n",time_str(time,3),eph->sat);

//    if (eph->A<=0.0) {
//        rs[0]=rs[1]=rs[2]=*dts=*var=0.0;
//        return;
//    }
    tk= (t -eph->toe).toSec();

//    switch ((sys=satsys(eph->sat,&prn))) {
//    case SYS_GAL: mu=MU_GAL; omge=OMGE_GAL; break;
//    case SYS_CMP: mu=MU_CMP; omge=OMGE_CMP; break;
//    default:      mu=MU_GPS; omge=OMGE;     break;
//    }
    mu = MU_GPS;
    omge = OMGE;
    M=eph->M0+(sqrt(mu/(eph->A*eph->A*eph->A))+eph->deln)*tk;

    for (n=0,E=M,Ek=0.0;fabs(E-Ek)>RTOL_KEPLER&&n<MAX_ITER_KEPLER;n++) {
        Ek=E; E-=(E-eph->e*sin(E)-M)/(1.0-eph->e*cos(E));
    }
    if (n>=MAX_ITER_KEPLER) {
//        trace(2,"eph2pos: kepler iteration overflow sat=%2d\n",eph->sat);
        return;
    }
    sinE=sin(E); cosE=cos(E);

//    trace(4,"kepler: sat=%2d e=%8.5f n=%2d del=%10.3e\n",eph->sat,eph->e,n,E-Ek);

    u=atan2(sqrt(1.0-eph->e*eph->e)*sinE,cosE-eph->e)+eph->omg;
    r=eph->A*(1.0-eph->e*cosE);
    i=eph->i0+eph->idot*tk;
    sin2u=sin(2.0*u); cos2u=cos(2.0*u);
    u+=eph->cus*sin2u+eph->cuc*cos2u;
    r+=eph->crs*sin2u+eph->crc*cos2u;
    i+=eph->cis*sin2u+eph->cic*cos2u;
    x=r*cos(u); y=r*sin(u); cosi=cos(i);

    /* beidou geo satellite */
//    if (sys==SYS_CMP&&(eph->flag==2||(eph->flag==0&&prn<=5))) {
//        O=eph->OMG0+eph->OMGd*tk-omge*eph->toes;
//        sinO=sin(O); cosO=cos(O);
//        xg=x*cosO-y*cosi*sinO;
//        yg=x*sinO+y*cosi*cosO;
//        zg=y*sin(i);
//        sino=sin(omge*tk); coso=cos(omge*tk);
//        rs[0]= xg*coso+yg*sino*COS_5+zg*sino*SIN_5;
//        rs[1]=-xg*sino+yg*coso*COS_5+zg*coso*SIN_5;
//        rs[2]=-yg*SIN_5+zg*COS_5;
//    }
//    else {
        O=eph->OMG0+(eph->OMGd-omge)*tk-omge*eph->toes;
        sinO=sin(O); cosO=cos(O);
        pos[0]=x*cosO-y*cosi*sinO;
        pos[1]=x*sinO+y*cosi*cosO;
        pos[2]=y*sin(i);
//    }
    tk= (t - eph->toc).toSec();
    *dts=eph->f0+eph->f1*tk+eph->f2*tk*tk;

    /* relativity correction */
    *dts-=2.0*sqrt(mu*eph->A)*eph->e*sinE/SQR(CLIGHT);
}
