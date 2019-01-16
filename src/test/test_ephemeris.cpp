#include <gtest/gtest.h>

#include "multirotor_sim/ephemeris.h"
#include "multirotor_sim/test_common.h"

#define SQR(x) (x*x)

void eph2pos(const GTime& t, const Ephemeris* eph, Vector3d& pos)
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
    double dts=eph->f0+eph->f1*tk+eph->f2*tk*tk;

    /* relativity correction */
    dts-=2.0*sqrt(mu*eph->A)*eph->e*sinE/SQR(CLIGHT);
}

TEST (Ephemeris, RTKLIB)
{
    GTime time;
    Ephemeris eph;
    time.week = 86400.00 / DateTime::SECONDS_IN_WEEK;
    time.sec = 86400.00 - (time.week * DateTime::SECONDS_IN_WEEK);

    eph.A = SQR(5153.79589081);
    eph.toe.week = 93600.0 / DateTime::SECONDS_IN_WEEK;
    eph.toe.sec = 93600.0 - (eph.toe.week * DateTime::SECONDS_IN_WEEK);
    eph.toes = 93600.0;
    eph.deln =  0.465376527657e-08;
    eph.M0 =  1.05827953357;
    eph.e =  0.00223578442819;
    eph.omg =  2.06374037770;
    eph.cus =  0.177137553692e-05;
    eph.cuc =  0.457651913166e-05;
    eph.crs =  88.6875000000;
    eph.crc =  344.96875;
    eph.cis = -0.856816768646e-07;
    eph.cic =  0.651925802231e-07;
    eph.idot =  0.342514267094e-09;
    eph.i0 =  0.961685061380;
    eph.OMG0 =  1.64046615454;
    eph.OMGd = -0.856928551657e-08;

    Vector3d oracle_pos, oracle_vel, oracle_pos2;
    Vector3d new_pos, new_vel;
    Vector3d truth_pos, truth_vel;
    GTime t2 = time + 0.01;
    truth_pos << -12611434.19782218519,
            -13413103.97797041226,
            19062913.07357876760;
    truth_vel <<  266.280379332602,
            -2424.768347293139,
            -1529.762077704072;
    eph2pos(time, &eph, oracle_pos);
    eph2pos(t2, &eph, oracle_pos2);
    oracle_vel = (oracle_pos2 - oracle_pos) / 0.01;

    Vector2d clock;
    eph.computePositionVelocityClock(time, new_pos, new_vel, clock);

    EXPECT_MAT_NEAR(oracle_pos, new_pos, 1e-5);
    EXPECT_MAT_NEAR(oracle_vel, new_vel, 1e-2);
    EXPECT_MAT_NEAR(new_pos, truth_pos, 1e-5);
    EXPECT_MAT_NEAR(new_vel, truth_vel, 1e-5);
}
