#! python3
import unittest
import doctest

def load_tests(loader, tests, ignore):
    tests.addTests(doctest.DocTestSuite('async_helpers.shared'))
    tests.addTests(doctest.DocTestSuite('async_helpers.pipe'))
    return tests


if __name__ == '__main__':
	unittest.main()
