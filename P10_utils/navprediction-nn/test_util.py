import logging
from unittest import TestCase
from util import timeit, new_logger


class TestUtil(TestCase):
    def test_init_log(self):
        log = new_logger()
        # check whether info is a method
        self.assertIsInstance(log, logging.Logger)

    def test_timeit(self):
        @timeit
        def f():
            return 1

        # check whether f is still working correctly
        self.assertEqual(f(), 1)
