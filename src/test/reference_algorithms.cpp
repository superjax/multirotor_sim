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

/* ionosphere model ------------------------------------------------------------
* From RTKLIB rtkcmn.c
* compute ionospheric delay by broadcast ionosphere model (klobuchar model)
* args   : gtime_t t        I   time (gpst)
*          double *ion      I   iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} (JJ: ALWAYS USE DEFAULT)
*          double *pos      I   receiver position {lat,lon,h} (rad,m)
*          double *azel     I   azimuth/elevation angle {az,el} (rad)
* return : ionospheric delay (L1) (m)
*-----------------------------------------------------------------------------*/
double ionmodel(const GTime &t,const double *pos, const double *azel)
{

    static const double CLIGHT = 299792458.0;
    const double ion_default[]={ /* 2004/1/1 */
        0.1118E-07,-0.7451E-08,-0.5961E-07, 0.1192E-06,
        0.1167E+06,-0.2294E+06,-0.1311E+06, 0.1049E+07
    };
    double tt,f,psi,phi,lam,amp,per,x;
    int week;

//    if (pos[2]<-1E3||azel[1]<=0) return 0.0;
//    if (norm(ion,8)<=0.0)
    const double* ion=ion_default;

    /* earth centered angle (semi-circle) */
    psi=0.0137/(azel[1]/M_PI+0.11)-0.022;

    /* subionospheric latitude/longitude (semi-circle) */
    phi=pos[0]/M_PI+psi*cos(azel[0]);
    if      (phi> 0.416) phi= 0.416;
    else if (phi<-0.416) phi=-0.416;
    lam=pos[1]/M_PI+psi*sin(azel[0])/cos(phi*M_PI);
    /* geomagnetic latitude (semi-circle) */
    phi+=0.064*cos((lam-1.617)*M_PI);

    /* local time (s) */
    tt=43200.0*lam+t.sec;
    tt-=floor(tt/86400.0)*86400.0; /* 0<=tt<86400 */

    /* slant factor */
    f=1.0+16.0*pow(0.53-azel[1]/M_PI,3.0);

    /* ionospheric delay */
    amp=ion[0]+phi*(ion[1]+phi*(ion[2]+phi*ion[3]));
    per=ion[4]+phi*(ion[5]+phi*(ion[6]+phi*ion[7]));
    amp=amp<    0.0?    0.0:amp;
    per=per<72000.0?72000.0:per;
    x=2.0*M_PI*(tt-50400.0)/per;

    return CLIGHT*f*(fabs(x)<1.57 ? 5E-9+amp*(1.0+x*x*(-0.5+x*x/24.0)) : 5E-9);
}
