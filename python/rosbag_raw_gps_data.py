import rospy, rosbag
from inertial_sense.msg import GNSSEphemeris, GlonassEphemeris, GNSSObservation, GPS
import sys, getopt, os
from tqdm import tqdm
import numpy as np

gtime_t = np.dtype([
    ('time', np.int64),
    ('sec', np.float64),
])

eph_t = np.dtype([
    ('sat', np.int32),
    ('iode', np.int32),
    ('iodc', np.int32),
    ('sva', np.int32),
    ('svh', np.int32),
    ('week', np.int32),
    ('code', np.int32),
    ('flag', np.int32),
    ('toe', gtime_t),
    ('toc', gtime_t),
    ('ttr', gtime_t),
    ('A', np.float64),
    ('e', np.float64),
    ('i0', np.float64),
    ('OMG0', np.float64),
    ('omg', np.float64),
    ('M0', np.float64),
    ('deln', np.float64),
    ('OMGd', np.float64),
    ('idot', np.float64),
    ('crc', np.float64),
    ('crs', np.float64),
    ('cuc', np.float64),
    ('cus', np.float64),
    ('cic', np.float64),
    ('cis', np.float64),
    ('toes', np.float64),
    ('fit', np.float64),
    ('f0', np.float64),
    ('f1', np.float64),
    ('f2', np.float64),
    ('tgd', (np.float64, 4)),
    ('Adot', np.float64),
    ('ndot', np.float64),
])

geph_t = np.dtype([
    ('sat', np.int32),
    ('iode', np.int32),
    ('frq', np.int32),
    ('svh', np.int32),
    ('sva', np.int32),
    ('age', np.int32),
    ('toe', gtime_t),
    ('tof', gtime_t),
    ('pos', (np.float64, 3)),
    ('vel', (np.float64, 3)),
    ('acc', (np.float64, 3)),
    ('taun', np.float64),
    ('gamn', np.float64),
    ('dtaun', np.float64)])

obsd_t = np.dtype([
    ('time', gtime_t),
    ('sat', np.uint8),
    ('rcv', np.uint8),
    ('SNR', np.uint8),
    ('LLI', np.uint8),
    ('code', np.uint8),
    ('qualL', np.uint8),
    ('qualP', np.uint8),
    ('reserved', np.uint8),
    ('L', np.float64),
    ('P', np.float64),
    ('D', np.float32)
])

pos_t = np.dtype([
    ('time', gtime_t),
    ('pos_ecef', (np.float64, 3))
])


def read_rosbag(rosbag_filename, output_folder):
    bag = rosbag.Bag(rosbag_filename, 'r')
    eph = np.empty(bag.get_message_count("/gps/eph"), dtype=eph_t)
    geph = np.empty(bag.get_message_count("/gps/geph"), dtype=geph_t)
    obs = np.empty(bag.get_message_count("/gps/obs"), dtype=obsd_t)
    pos = np.empty((bag.get_message_count("/gps")), dtype=pos_t)

    topics = ["/gps/eph", "/gps/geph","/gps/obs", "/gps"]

    eph_i, geph_i, obs_i, pos_i = 0, 0, 0, 0
    for topic, msg, t in tqdm(bag.read_messages(topics=topics), total=bag.get_message_count(topics)):
        if msg._type == 'inertial_sense/GNSSObservation':
            obs[obs_i]['time']['time'] = msg.time.time
            obs[obs_i]['time']['sec'] = msg.time.sec
            obs[obs_i]['sat'] = msg.sat
            obs[obs_i]['rcv'] = msg.rcv
            obs[obs_i]['SNR'] = msg.SNR
            obs[obs_i]['LLI'] = msg.LLI
            obs[obs_i]['code'] = msg.code
            obs[obs_i]['qualL'] = msg.qualL
            obs[obs_i]['qualP'] = msg.qualP
            obs[obs_i]['L'] = msg.L
            obs[obs_i]['P'] = msg.P
            obs[obs_i]['D'] = msg.D
            obs_i += 1
        elif msg._type == 'inertial_sense/GNSSEphemeris':
            eph[eph_i]['sat'] = msg.sat
            eph[eph_i]['iode'] = msg.iode
            eph[eph_i]['iodc'] = msg.iodc
            eph[eph_i]['sva'] = msg.sva
            eph[eph_i]['svh'] = msg.svh
            eph[eph_i]['week'] = msg.week
            eph[eph_i]['code'] = msg.code
            eph[eph_i]['flag'] = msg.flag
            eph[eph_i]['toe']['time'] = msg.toe.time
            eph[eph_i]['toc']['time'] = msg.toc.time
            eph[eph_i]['ttr']['time'] = msg.ttr.time
            eph[eph_i]['toe']['sec'] = msg.toe.sec
            eph[eph_i]['toc']['sec'] = msg.toc.sec
            eph[eph_i]['ttr']['sec'] = msg.ttr.sec
            eph[eph_i]['A'] = msg.A
            eph[eph_i]['e'] = msg.e
            eph[eph_i]['i0'] = msg.i0
            eph[eph_i]['OMG0'] = msg.OMG0
            eph[eph_i]['omg'] = msg.omg
            eph[eph_i]['M0'] = msg.M0
            eph[eph_i]['deln'] = msg.deln
            eph[eph_i]['OMGd'] = msg.OMGd
            eph[eph_i]['idot'] = msg.idot
            eph[eph_i]['crc'] = msg.crc
            eph[eph_i]['crs'] = msg.crs
            eph[eph_i]['cuc'] = msg.cuc
            eph[eph_i]['cus'] = msg.cus
            eph[eph_i]['cic'] = msg.cic
            eph[eph_i]['cis'] = msg.cis
            eph[eph_i]['toes'] = msg.toes
            eph[eph_i]['fit'] = msg.fit
            eph[eph_i]['f0'] = msg.f0
            eph[eph_i]['f1'] = msg.f1
            eph[eph_i]['f2'] = msg.f2
            eph[eph_i]['tgd'] = msg.tgd
            eph[eph_i]['Adot'] = msg.Adot
            eph[eph_i]['ndot'] = msg.ndot
            eph_i += 1
        elif msg._type == 'inertial_sense/GlonassEphemeris':
            geph[geph_i]['sat'] = msg.sat
            geph[geph_i]['iode'] = msg.iode
            geph[geph_i]['iode'] = msg.iode
            geph[geph_i]['frq'] = msg.frq
            geph[geph_i]['svh'] = msg.svh
            geph[geph_i]['sva'] = msg.sva
            geph[geph_i]['age'] = msg.age
            geph[geph_i]['toe']['time'] = msg.toe.time
            geph[geph_i]['tof']['time'] = msg.tof.time
            geph[geph_i]['toe']['sec'] = msg.toe.sec
            geph[geph_i]['tof']['sec'] = msg.tof.sec
            geph[geph_i]['pos'] = msg.pos
            geph[geph_i]['vel'] = msg.vel
            geph[geph_i]['acc'] = msg.acc
            geph[geph_i]['taun'] = msg.taun
            geph[geph_i]['gamn'] = msg.gamn
            geph[geph_i]['dtaun'] = msg.dtaun
            geph_i += 1
        elif msg._type == 'inertial_sense/GPS':
            pass
            pos[pos_i]['time']['time'] = msg.header.stamp.secs
            pos[pos_i]['time']['sec'] = msg.header.stamp.nsecs*1e9
            pos[pos_i]['pos_ecef'] = [msg.posEcef.x, msg.posEcef.y, msg.posEcef.z]

    # write the binaries
    eph.tofile(output_folder + "/eph.dat")
    geph.tofile(output_folder + "/geph.dat")
    obs.tofile(output_folder + "/obs.dat")
    pos.tofile(output_folder + "/pos.dat")



if __name__ == '__main__':
    input_filename = ''
    output_folder = ''
    try:
        opts, args = getopt.getopt(sys.argv[1:], r"f:o:")
    except:
        print 'rosbag_raw_gps_data.py -f <input bag> -o <output folder>'
        sys.exit(0)
    for opt, arg in opts:
        if opt == '-f':
            input_filename = arg
        elif opt == '-o':
            output_folder = arg

    read_rosbag(os.path.expanduser(input_filename), output_folder)


