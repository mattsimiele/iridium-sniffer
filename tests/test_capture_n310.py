"""Fake UHD transport tests: no radio or UHD installation needed."""
import importlib.util
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch, MagicMock
import numpy as np

spec = importlib.util.spec_from_file_location('capture', Path(__file__).resolve().parents[1]/'tools/capture_n310.py')
c = importlib.util.module_from_spec(spec)
spec.loader.exec_module(c)
RATE = 2_500_000
EPOCH = 1_790_000_000

class TimeSpec:
    def __init__(self, seconds, fraction=0):
        self.seconds, self.fraction = int(seconds), fraction
    def get_full_secs(self): return self.seconds
    def get_frac_secs(self): return self.fraction

class Sensor:
    def __init__(self, value): self.value = value
    def to_bool(self): return self.value
    def to_int(self): return self.value

class Metadata:
    def strerror(self): return self.error_code

class Command:
    def __init__(self, mode): self.mode = mode

class Streamer:
    def __init__(self, packets):
        self.packets = iter(packets)
        self.commands = []
    def issue_stream_cmd(self, command): self.commands.append(command)
    def recv(self, buf, md, timeout):
        delta, count, error, stamp, value = next(self.packets)
        md.time_spec = TimeSpec(EPOCH, delta/RATE)
        md.has_time_spec = stamp
        md.error_code = error
        md.out_of_sequence = error == 'sequence'
        buf[0,:count] = value
        return count

UHD = NS(types=NS(TuneRequest=lambda f:f, TimeSpec=TimeSpec, RXMetadata=Metadata, StreamCMD=Command,
    StreamMode=NS(start_cont='start',stop_cont='stop'), RXMetadataErrorCode=NS(none='none')))

class Radio:
    def get_mboard_sensor(self, name, board): return Sensor(True)

class CaptureTests(unittest.TestCase):
    def run_stream(self, packets, count=4, radio=None):
        stream = Streamer(packets)
        out = io.BytesIO()
        meta = c.metadata({'rate':RATE,'frequency':1626000000},{},'fake N310')
        events = []
        error = None
        try:
            c.receive(radio or Radio(),stream,UHD,np,out,lambda event,**kw:events.append((event,kw)),
                      meta,count/RATE,EPOCH,clock=lambda:0)
        except c.CaptureError as exc:
            error = str(exc)
        self.assertEqual(stream.commands[-1].mode,'stop')
        return out.getvalue(),meta,events,error

    def test_contiguous_and_iq_layout(self):
        raw,meta,events,error = self.run_stream([(0,2,'none',True,0.25-0.5j),(2,2,'none',True,-0.25+0.5j)])
        self.assertIsNone(error)
        self.assertEqual(np.frombuffer(raw,dtype='<i2').tolist(),[8192,-16384]*2+[-8192,16384]*2)
        self.assertEqual(meta['global']['n310_capture:samples_written'],4)
        self.assertEqual(meta['captures'][0]['n310_capture:hardware_tick'],EPOCH*RATE)
        self.assertTrue(meta['captures'][0]['core:datetime'].endswith('Z'))

    def test_gap_and_overlap_not_appended(self):
        for delta in (3,1):
            raw,meta,events,error = self.run_stream([(0,2,'none',True,0j),(delta,2,'none',True,0j)])
            self.assertIn('discontinuity',error)
            self.assertEqual(len(raw),8)
            self.assertEqual(events[-1][1]['delta_samples'],delta-2)

    def test_bad_first_timestamp(self):
        raw,_,_,error = self.run_stream([(1,4,'none',True,0j)])
        self.assertIn('discontinuity',error)
        self.assertEqual(raw,b'')

    def test_transport_failures(self):
        for error in ('overflow','timeout','late','sequence','bad_packet'):
            raw,_,_,failure = self.run_stream([(0,4,error,True,0j)])
            self.assertIn('UHD receive error',failure)
            self.assertEqual(raw,b'')

    def test_missing_time_and_zero_count(self):
        for stamp,count in ((False,4),(True,0)):
            raw,_,_,error = self.run_stream([(0,count,'none',stamp,0j)])
            self.assertIsNotNone(error)
            self.assertEqual(raw,b'')

    def test_final_buffer_truncated(self):
        raw,meta,_,error = self.run_stream([(0,8,'none',True,0j)],count=3)
        self.assertIsNone(error)
        self.assertEqual(len(raw),12)
        self.assertEqual(meta['global']['n310_capture:samples_written'],3)

    def test_nonfinite_and_clipping(self):
        for value in (complex(float('nan'),0), 2+0j):
            _,_,_,error = self.run_stream([(0,4,'none',True,value)])
            self.assertIsNotNone(error)

    def test_lock_loss(self):
        class Unlocked(Radio):
            def get_mboard_sensor(self,name,board): return Sensor(False)
        raw,_,_,error = self.run_stream([],radio=Unlocked())
        self.assertIn('lock lost',error)
        self.assertEqual(raw,b'')

    def test_full_epoch_one_sample_resolution(self):
        a = TimeSpec(EPOCH,0.5)
        b = TimeSpec(EPOCH,0.5+1/RATE)
        self.assertEqual(c.ticks(b,RATE)-c.ticks(a,RATE),1)

    def test_stable_lock_reset_and_timeout(self):
        now = [0]
        class Intermittent(Radio):
            def get_mboard_sensor(self,name,board): return Sensor(now[0] != 2)
        def sleep(n): now[0] += n
        c.wait_lock(Intermittent(),10,3,lambda *a,**k:None,lambda:now[0],sleep)
        self.assertEqual(now[0],6)
        now[0]=0
        with self.assertRaises(c.CaptureError):
            c.wait_lock(Intermittent(),5,3,lambda *a,**k:None,lambda:now[0],sleep)

    def test_epoch_sync(self):
        class GPS(Radio):
            def get_mboard_sensor(self,name,board): return Sensor(EPOCH if name=='gps_time' else True)
            def set_time_next_pps(self,ts,board): self.scheduled=ts
            def get_time_last_pps(self,board): return TimeSpec(EPOCH)
        radio = GPS()
        with patch.object(c.time,'time',return_value=EPOCH),patch.object(c.time,'sleep'):
            c.sync_epoch(radio,UHD,lambda *a,**k:None)
        self.assertEqual(radio.scheduled.get_full_secs(),EPOCH+1)
        with patch.object(c.time,'time',return_value=EPOCH+18):
            with self.assertRaises(c.CaptureError): c.sync_epoch(radio,UHD,lambda *a,**k:None)

    def test_disk_error_stops_stream(self):
        class FullDisk(io.BytesIO):
            def write(self, data): raise OSError('disk full')
        stream=Streamer([(0,4,'none',True,0j)])
        meta=c.metadata({'rate':RATE,'frequency':1626000000},{},'fake')
        with self.assertRaises(OSError):
            c.receive(Radio(),stream,UHD,np,FullDisk(),lambda *a,**k:None,meta,4/RATE,EPOCH,clock=lambda:0)
        self.assertEqual(stream.commands[-1].mode,'stop')
        self.assertEqual(meta['global']['n310_capture:samples_written'],0)

    def test_pps_mismatch_rejected(self):
        class WrongPPS(Radio):
            def get_mboard_sensor(self,name,board): return Sensor(EPOCH if name=='gps_time' else True)
            def set_time_next_pps(self,ts,board): pass
            def get_time_last_pps(self,board): return TimeSpec(EPOCH-1)
        with patch.object(c.time,'time',return_value=EPOCH),patch.object(c.time,'sleep'):
            with self.assertRaises(c.CaptureError): c.sync_epoch(WrongPPS(),UHD,lambda *a,**k:None)

    def test_capture_complete_and_failed_metadata(self):
        for fail in (False,True):
            with tempfile.TemporaryDirectory() as d:
                a=c.parser().parse_args(['--args','addr=fake','--output',str(Path(d)/'run'),'--duration','1'])
                radio=MagicMock()
                radio.get_num_mboards.return_value=1
                radio.get_mboard_name.return_value='N310'
                radio.get_mboard_sensor_names.return_value=['gps_locked','ref_locked','gps_time']
                radio.get_rx_rate.return_value=RATE
                radio.get_rx_freq.return_value=a.freq
                radio.set_rx_freq.return_value=NS(actual_rf_freq=a.freq,actual_dsp_freq=0)
                radio.get_rx_gain.return_value=30
                radio.get_rx_bandwidth.return_value=5e6
                radio.get_rx_antenna.return_value='RX2'
                radio.get_master_clock_rate.return_value=125e6
                radio.get_clock_source.return_value='gpsdo'
                radio.get_time_source.return_value='gpsdo'
                radio.get_pp_string.return_value='fake N310'
                radio.get_time_now.return_value=TimeSpec(EPOCH-3)
                uhd=NS(usrp=NS(MultiUSRP=lambda args:radio,StreamArgs=lambda *args:NS()),
                       types=UHD.types,get_version_string=lambda:'fake UHD')
                def recv(*args):
                    args[4].write(b'12345678')
                    args[6]['global']['n310_capture:samples_written']=2
                    if fail: raise c.CaptureError('injected gap')
                with patch.object(c,'wait_lock'),patch.object(c,'sync_epoch'),patch.object(c,'receive',side_effect=recv),patch('sys.stderr',io.StringIO()):
                    if fail:
                        with self.assertRaises(c.CaptureError): c.capture(a,uhd,np)
                    else: c.capture(a,uhd,np)
                meta=json.loads((a.output/'capture.sigmf-meta').read_text())
                self.assertEqual(meta['global']['n310_capture:valid'],not fail)
                self.assertEqual(meta['global']['n310_capture:status'],'failed' if fail else 'complete')
                self.assertEqual(meta['global']['n310_capture:samples_written'],2)
                self.assertEqual((a.output/'capture.sigmf-data').read_bytes(),b'12345678')
                # No overwrite of an existing run, even via the library entry point.
                with self.assertRaises(FileExistsError): c.capture(a,uhd,np)

    def test_atomic_metadata(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'capture.sigmf-meta'
            meta=c.metadata({'rate':RATE},{},'fake')
            c.save_meta(path,meta)
            self.assertFalse(json.loads(path.read_text())['global']['n310_capture:valid'])
            self.assertFalse(path.with_suffix('.tmp').exists())

if __name__ == '__main__': unittest.main()
