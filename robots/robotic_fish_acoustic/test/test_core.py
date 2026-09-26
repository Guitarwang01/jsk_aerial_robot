import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from robotic_fish_acoustic.core import CLASSES, FEATURES, Model, Estimator, Status, adapt


def batch(t, values=(1., 2., 1.), old=False, calibrated=True):
    samples = []
    for channel, value in enumerate(values):
        stamp = NS(sec=int(t), nanosec=int((t-int(t))*1e9))
        if old:
            samples.append(NS(channel=channel, header=NS(stamp=stamp), calibrated_voltage=value,
                              calibration_applied=calibrated, calibration_id='test', raw=100,
                              gain_control_voltage_valid=True, gain_control_voltage=.9))
        else:
            samples.append(NS(channel_id=channel, timestamp=stamp, volt_cali=value,
                              status_cali=calibrated, cali_id='test', adc_code=100,
                              status_dac_feedback=True, dac_volt=.9))
    return NS(samples=samples)


class CoreTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        path = Path(self.temp.name) / 'fixture.json'
        # Synthetic fixture only: prescribed FRD probabilities .1,.2,.3,.4.
        import math
        path.write_text(json.dumps(dict(classes=CLASSES, features=FEATURES, frame='FRD',
            weights=[[0.]*4]*4, mean=[0.]*4, scale=[1.]*4,
            bias=[math.log(p) for p in [.1,.2,.3,.4]], temperature=1., window_s=.5,
            model_id='SYNTHETIC_TEST_ONLY')))
        self.model = Model(path)

    def tearDown(self):
        self.temp.cleanup()

    def test_frd_marginals(self):
        r = Estimator(self.model).push(batch(10.), 10.)
        p = r['probabilities']
        self.assertAlmostEqual(p[2]+p[3], .7)
        self.assertAlmostEqual(p[0]+p[1], .3)
        self.assertTrue(r['updated'])

    def test_fallback_keeps_new_result(self):
        r = Estimator(self.model).push(batch(10., calibrated=False), 10.)
        self.assertTrue(r['updated'])
        self.assertTrue(r['status'] & Status.CALIBRATION_FALLBACK)

    def test_hold_does_not_retimestamp(self):
        e = Estimator(self.model)
        first = e.push(batch(10.), 10.)
        held = e.push(NS(samples=[]), 11.)
        self.assertEqual(held['probabilities'], first['probabilities'])
        self.assertEqual(held['estimate_stamp'], 10.)
        self.assertFalse(held['updated'])
        self.assertTrue(held['status'] & Status.STALE)

    def test_missing_model_never_fakes_probability(self):
        r = Estimator().push(batch(10.), 10.)
        self.assertIn('features', r)
        self.assertIsNone(r['probabilities'])
        self.assertFalse(r['has_estimate'])

    def test_old_new_numeric_equivalence(self):
        old, new = Estimator(self.model), Estimator(self.model)
        for i in range(30):
            t = 10 + i*.05
            a, b = old.push(batch(t, old=True), t), new.push(batch(t), t)
            self.assertEqual(a['features'], b['features'])
            self.assertEqual(a['probabilities'], b['probabilities'])
        self.assertNotEqual(a['timestamp_semantics'], b['timestamp_semantics'])

    def test_rewind_resets_window(self):
        e = Estimator(self.model)
        e.push(batch(20.), 20.)
        r = e.push(batch(10.), 10.)
        self.assertTrue(r['status'] & Status.TIME_RESET)
        self.assertEqual(r['valid_sample_count'], 1)

    def test_zero_signal_holds(self):
        e = Estimator(self.model)
        r = e.push(batch(10., values=(0.,0.,0.)), 10.)
        self.assertFalse(r['updated'])
        self.assertTrue(r['status'] & Status.LOW_SIGNAL)

    def test_quality_survives_heartbeat(self):
        e = Estimator(self.model)
        e.push(batch(10.), 10.)
        e.push(NS(samples=[]), 10.01)
        self.assertTrue(e.snapshot(10.02)['status'] & Status.INVALID_INPUT)

    def test_low_nonzero_signal_still_computes(self):
        r = Estimator(self.model).push(batch(10., values=(.00001, .00002, .00001)), 10.)
        self.assertTrue(r['updated'])
        self.assertTrue(r['status'] & Status.LOW_SIGNAL)


if __name__ == '__main__':
    unittest.main()
