#!/usr/bin/env python3
"""Offline tests for roll trend."""
import pathlib
import subprocess
import tempfile


def main():
    root = pathlib.Path(__file__).resolve().parents[1]
    code = r'''
#include <hugmy/control/roll_trend_gate.h>
#include <algorithm>
#include <cassert>
#include <cstdio>
int main() {
 const double d=M_PI/180.;hugmy::RollTrendGate g;
 for(int i=0;i<=50;++i)g.update(true,10*d,0,.02*i);
 assert(g.ready && !g.worsening);
 g.reset();for(int i=0;i<=50;++i)g.update(true,(9+.06*i)*d,3*d,.02*i);
 assert(g.ready && g.worsening);
 g.reset();for(int i=0;i<=50;++i)g.update(true,(-9-.06*i)*d,-3*d,.02*i);
 assert(g.ready && g.worsening);
 g.reset();for(int i=0;i<=50;++i)g.update(true,(9+.06*i)*d,-3*d,.02*i);
 assert(!g.worsening); // Disagreeing gyro cannot certify growth.
 g.update(false,12*d,0,1.02);assert(!g.ready && !g.worsening);
 g.update(true,12*d,0,2);g.update(true,NAN,0,2.02);assert(!g.ready);
 g.update(true,12*d,0,3);g.update(true,12*d,0,3.5);assert(!g.ready); // Gap resets.
 g.update(true,12*d,0,2);assert(!g.ready); // Clock reset.
 puts("PASS: signed roll trend, agreement/fault/reset gates");
}
'''
    with tempfile.TemporaryDirectory(prefix='hugmy_roll_bottom_') as directory:
        binary = str(pathlib.Path(directory)/'test')
        subprocess.run(['g++','-std=c++14','-Wall','-Wextra','-I',str(root/'include'),'-x','c++','-','-o',binary],
                       input=code,text=True,check=True)
        subprocess.run([binary],check=True)


if __name__ == '__main__':
    main()
