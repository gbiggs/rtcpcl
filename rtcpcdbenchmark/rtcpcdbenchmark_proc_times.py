#!/usr/bin/env python

import math
import optparse
import os
import sys
import traceback


BUNNY_SIZE_POINTS = 361
BUNNY_SIZE_BYTES = 5776


def calc_stats(data):
    if not data:
        return 0, 0
    if len(data) == 1:
        return data[0], 0
    n = 0
    mean = 0
    M2 = 0
    for x in data:
        n = n + 1
        delta = x - mean
        mean = mean + delta / n
        M2 = M2 + delta * (x - mean)
    variance_n = M2 / n
    variance = M2 / ( n - 1)
    return mean, variance


def load_write(filename):
    # Start, Packed, Written
    times = {}
    with open(filename, 'r') as f:
        f.readline()
        for l in f:
            l = l.split(',')
            entry = []
            seq = int(l[0])
            if seq != -1:
                for v in l[1:]:
                    entry.append(int(v.strip()))
                times[seq] = entry
    return times


def load_read(filename):
    # Start, Read, Unpacked
    times = {}
    with open(filename, 'r') as f:
        f.readline()
        for l in f:
            l = l.split(',')
            entry = []
            seq = int(l[-1])
            if seq != -1:
                for v in l[:-1]:
                    entry.append(int(v.strip()))
                times[seq] = entry
    return times


def load_echo(filename):
    # Start, Read, Unpacked, Packed, Written, Sequence ID
    times = {}
    with open(filename, 'r') as f:
        f.readline()
        for l in f:
            l = l.split(',')
            entry = []
            seq = int(l[-1])
            if seq != -1:
                for v in l[:-1]:
                    entry.append(int(v.strip()))
                times[seq] = entry
    return times


def load_bw(filename):
    fields = []
    data = []
    with open(filename, 'r') as f:
        l = f.readline()
        for v in l.split(','):
            fields.append(v.strip())
        l = f.readline()
        for v in l.split(','):
            data.append(int(v.strip()))
    return dict(zip(fields, data))


def write_stats(times):
    total = []
    pack = []
    write = []
    for sid in times:
        entry = times[sid]
        total.append(entry[2] - entry[0])
        pack.append(entry[1] - entry[0])
        write.append(entry[2] - entry[1])
    return {'Writer:Total (ns)': total,
            'Writer:Pack (ns)': pack,
            'Writer:Write (ns': write}


def read_stats(times):
    total = []
    read = []
    unpack = []
    for sid in times:
        entry = times[sid]
        total.append(entry[2] - entry[0])
        read.append(entry[1] - entry[0])
        unpack.append(entry[2] - entry[1])
    return {'Reader:Total (ns)': total,
            'Reader:Read (ns)': read,
            'Reader:Unpack (ns)': unpack}


def round_trip_stats(write_times, read_times):
    total = []
    gap = []
    wtor = []
    for sid in write_times:
        if sid not in read_times:
            continue
        w_entry = write_times[sid]
        r_entry = read_times[sid]
        total.append(r_entry[2] - w_entry[0])
        gap.append(r_entry[0] - w_entry[2])
        wtor.append(r_entry[1] - w_entry[1])
    return {'Round trip:Total (ns)': total,
            'Round trip:Execution gap (ns)': gap,
            'Round trip:Write to read (ns)': wtor}


def bw_stats(tag, data):
    time_diff = data['End time'] - data['Start time']
    time_diff_f = time_diff / 1e9
    cloudsps = data['Clouds'] / time_diff_f
    pointsps = data['Points'] / time_diff_f
    bytesps = data['Bytes'] / time_diff_f
    bunniesps = (data['Bytes'] / float(data['Cloud size (bytes)']) /
        time_diff_f)
    return {tag + 'Test time (ns)': time_diff,
            tag + 'Test time (s)': time_diff_f,
            tag + 'Clouds/s': cloudsps,
            tag + 'Points/s': pointsps,
            tag + 'Bytes/s': bytesps,
            tag + 'MB/s': bytesps / 1024 / 1024.0,
            tag + 'Bunnies/s': bunniesps}


def print_stats(stats, title):
    print '{0: <30}{1: <30}{2: <30}{3: <30}'.format(title, 'Mean', 'Variance',
            'Standard deviation')
    print '{0:=<29} {0:=<29} {0:=<29} {0:=<29}'.format('')
    for s in stats:
        keys = s.keys()
        keys.sort()
        for k in keys:
            if type(s[k]) is list:
                m, v = calc_stats(s[k])
            else:
                m = s[k]
                v = 0
            print '{0: <30}{1: <30}{2: <30}{3: <30}'.format(k, m, v,
                    math.sqrt(v))
        print '{0:-<29} {0:-<29} {0:-<29} {0:-<29}'.format('')


def main():
    usage = 'Usage:: %prog [options]\n'\
        'Calculate timing statistics of a transport benchmark run.'
    parser = optparse.OptionParser(usage=usage, version='1.0')
    parser.add_option('-b', '--read_bw', dest='read_bw', action='store',
            type='string', default='',
            help='File name of the read bandwidth data. [Default: %default]')
    parser.add_option('-c', '--echo_bw', dest='echo_bw', action='store',
            type='string', default='',
            help='File name of the echo bandwidth data. [Default: %default]')
    parser.add_option('-e', '--echo-times', dest='echo_times', action='store',
            type='string', default='',
            help='File name of the echo data. [Default: %default]')
    parser.add_option('-i', '--write_bw', dest='write_bw', action='store',
            type='string', default='',
            help='File name of the write bandwidth data. [Default: %default]')
    parser.add_option('-r', '--read-times', dest='read_times', action='store',
            type='string', default='',
            help='File name of the read data. [Default: %default]')
    parser.add_option('-t', '--trans-name', dest='trans_name',
            action='store', type='string', default='CORBA',
            help='Name of the transport tested. [Default: %default]')
    parser.add_option('-w', '--write-times', dest='write_times',
            action='store', type='string', default='',
            help='File name of the write data. [Default: %default]')
    parser.add_option('-v', '--verbose', dest='verbose', action='store_true',
            default=False,
            help='Output verbose information. [Default: %default]')

    try:
        options, args = parser.parse_args()
    except optparse.OptionError, e:
        print >>sys.stderr, 'OptionError:', e
        return 1

    try:
        if options.write_times:
            write_times = load_write(options.write_times)
            w_stats = write_stats(write_times)
        else:
            w_stats = {}
        if options.read_times:
            read_times = load_read(options.read_times)
            r_stats = read_stats(read_times)
        else:
            r_stats = {}

        if options.echo_times:
            echo_times = load_echo(options.echo_times)

        if options.write_times and options.read_times:
            rt_stats = round_trip_stats(write_times, read_times)
        else:
            rt_stats = {}

        if options.echo_bw:
            echo_bw = load_bw(options.echo_bw)
            echo_bw_stats = bw_stats('Echo ', echo_bw)
        else:
            echo_bw_stats = {}
        if options.write_bw:
            write_bw = load_bw(options.write_bw)
            write_bw_stats = bw_stats('Write ', write_bw)
        else:
            write_bw_stats = {}
        if options.read_bw:
            read_bw = load_bw(options.read_bw)
            read_bw_stats = bw_stats('Read ', read_bw)
        else:
            read_bw_stats = {}

        print_stats([w_stats, r_stats, rt_stats, echo_bw_stats, write_bw_stats,
            read_bw_stats], options.trans_name)
    except Exception, e:
        if options.verbose:
            traceback.print_exc()
        print >>sys.stderr, '{0}: {1}'.format(os.path.basename(sys.argv[0]), e)
        return 1
    return 0


if __name__ == '__main__':
    main()

